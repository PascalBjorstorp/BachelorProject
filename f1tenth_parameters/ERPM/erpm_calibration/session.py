"""Session orchestration for the ERPM longitudinal campaign."""
from __future__ import annotations
import datetime as dt, hashlib, json, math, os, re, shutil, signal, subprocess, sys, time
from pathlib import Path
from typing import Any, Callable
from .bagging import BagProcess, start_bag, stop_bag
from .config import copy_config_file, dump_json, dump_yaml, load_yaml
from .config_transaction import VescModeTransaction
from .stages import TrialCounter, run_stage, candidate_velocity_verification, candidate_accel_verification, candidate_cross_axis_verification
from .ui import banner, disk_line, require_ready

class StackProcess:
    def __init__(self, root:Path, config:Path, log:Path, candidate_patch:Path|None=None, candidate_mode:str|None=None) -> None:
        self.root=root; self.config=config; self.log=log; self.candidate_patch=candidate_patch; self.candidate_mode=candidate_mode; self.proc:subprocess.Popen|None=None; self.handle=None
    def start(self)->None:
        self.log.parent.mkdir(parents=True,exist_ok=True); self.handle=self.log.open('w',encoding='utf-8')
        cmd=['python3',str(self.root/'launch'/'calibration_stack.py'),'--config',str(self.config)]
        if self.candidate_patch is not None:
            if self.candidate_mode not in {'velocity','accel'}: raise RuntimeError('candidate stack needs velocity or accel mode')
            cmd += ['--candidate-patch',str(self.candidate_patch),'--candidate-mode',self.candidate_mode]
        self.proc=subprocess.Popen(cmd,stdout=self.handle,stderr=subprocess.STDOUT,start_new_session=True,text=True)
        time.sleep(2.0)
        if self.proc.poll() is not None:
            self.handle.close(); raise RuntimeError('calibration stack failed to start:\n'+self.log.read_text(encoding='utf-8',errors='replace'))
    def stop(self)->None:
        if self.proc and self.proc.poll() is None:
            try: os.killpg(self.proc.pid,signal.SIGINT)
            except ProcessLookupError: pass
            try: self.proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                try: os.killpg(self.proc.pid,signal.SIGTERM)
                except ProcessLookupError: pass
                self.proc.wait(timeout=5)
        if self.handle: self.handle.close(); self.handle=None

class SessionRunner:
    def __init__(self, root:Path, config_path:Path, runs_dir:Path|None=None,resume:Path|None=None,workspace:Path|None=None)->None:
        self.root=root.resolve(); self.config_path=config_path.resolve(); self.config=load_yaml(self.config_path); self.resume=resume
        self.runs_dir=(runs_dir or self.root/self.config['session']['runs_dir']).resolve(); self.session=self._prepare_session(); self.runtime=self._load_runtime(); self.stack:StackProcess|None=None; self.launch_index=0
        self.workspace=self._workspace(workspace); self.transaction=VescModeTransaction(calibration_root=self.root,session_dir=self.session,workspace=self.workspace,config_relpath=self.config['workspace']['vesc_config_relpath'],build_command=list(self.config['workspace']['colcon_build_command']),profiles=self.config['profiles'])
        self.counter=TrialCounter(self.config)

    def _verify_site_envelope(self) -> dict[str, Any]:
        """Reject a full-envelope campaign configured for an inadequate site.

        The supplied 15 m room remains useful for steering and low-speed work,
        but it is not a defensible site for the configured 3.0 m/s, high-current
        acceleration/braking identification.  This is a developer-set preflight
        declaration, archived in the session, not a claim that the runner can
        infer physical room length itself.
        """
        site = self.config.get('site', {})
        required = bool(site.get('require_full_envelope_track', True))
        usable = float(site.get('straight_usable_length_m', 0.0))
        minimum = float(site.get('minimum_full_envelope_length_m', 0.0))
        margin = float(site.get('setup_margin_m', 0.0))
        if required and usable < minimum:
            raise RuntimeError(
                f'full-envelope site preflight failed: usable straight={usable:.1f} m, '
                f'minimum configured={minimum:.1f} m. Use the full track or explicitly '                'reconfigure a limited-envelope campaign; do not run high-demand stages here.'
            )
        env = self.config.get('operating_envelope', {})
        try:
            drive_current = float(env.get('approved_drive_test_current_a', 0.0))
            brake_current = float(env.get('approved_brake_test_current_a', 0.0))
        except (TypeError, ValueError) as exc:
            raise RuntimeError('approved drive/brake test currents must be explicit finite positive values') from exc
        if not (drive_current > 0.0 and brake_current > 0.0):
            raise RuntimeError(
                'full-envelope current preflight failed (this is a one-time DEVELOPER config step, not a '
                'code change, and not the third-party operator). Edit operating_envelope.'
                'approved_drive_test_current_a and approved_brake_test_current_a in '
                'config/erpm_calibration.yaml to reviewed positive amps, e.g. ~0.5-0.7 of the motor '
                'continuous current rating and never above the VESC current_max. Once set, the operator '
                'runs the campaign with no further blocking. The tool refuses to invent a current ceiling '
                f'because it bounds motor/battery safety (got drive={drive_current}, brake={brake_current}).'
            )

        # Prove every configured high-demand condition can collect a minimum
        # transient before its conservative speed/stopping boundary. This keeps
        # an infeasible high-current/high-entry-speed request from appearing
        # only after hours of earlier collection.
        duration_spec = env.get('high_demand_pulse_duration_s', {})
        min_capture = float(env.get('dynamic_capture_min_s', 0.20))
        max_speed = float(env.get('maximum_test_speed_mps', 0.0))
        max_drive_accel = float(env.get('maximum_test_accel_mps2', 0.0))
        max_brake_accel = float(env.get('maximum_test_brake_mps2', 0.0))
        if not (max_speed > 0.0 and max_drive_accel > 0.0 and max_brake_accel > 0.0):
            raise RuntimeError('full-envelope preflight requires finite positive maximum_test_speed/accel/brake limits')
        duration_rows = []
        for section in ('raw_current_training', 'raw_current_holdout'):
            spec = self.config[section]
            for polarity in ('drive', 'brake'):
                entries = spec.get(f'{polarity}_conditions')
                if entries is None:
                    entries = [
                        {'initial_speed_mps': v, 'current_fractions': spec[f'{polarity}_current_fractions']}
                        for v in spec[f'{polarity}_initial_speeds_mps']
                    ]
                for entry in entries:
                    initial = float(entry['initial_speed_mps'])
                    for fraction in map(float, entry['current_fractions']):
                        base = float(duration_spec['low_fraction'] if fraction <= 0.25 else duration_spec['medium_fraction'] if fraction <= 0.55 else duration_spec['high_fraction'])
                        if polarity == 'drive':
                            headroom = max_speed - float(env.get('speed_guard_margin_mps', 0.0)) - initial
                            accel_cap = max_drive_accel * fraction
                        else:
                            headroom = initial - float(env.get('brake_guard_margin_mps', 0.0))
                            accel_cap = max_brake_accel * fraction
                        feasible = min(base, headroom / max(accel_cap, 1e-9))
                        duration_rows.append({
                            'section': section, 'polarity': polarity,
                            'initial_speed_mps': initial, 'current_fraction': fraction,
                            'requested_base_duration_s': base,
                            'conservative_feasible_duration_s': feasible,
                            'coverage_feasible': bool(feasible + 1e-9 >= min_capture),
                        })
        invalid = [row for row in duration_rows if not row['coverage_feasible']]
        if invalid:
            raise RuntimeError(
                'full-envelope dynamic schedule is infeasible before the declared speed/stopping boundary. '
                f'First invalid condition: {invalid[0]}. Use a longer track, a higher independently reviewed speed envelope, '
                'or remove the unsafe condition; do not proceed with partial high-demand evidence.'
            )
        result = {
            'usable_straight_m': usable,
            'minimum_required_m': minimum,
            'setup_margin_m': margin,
            'full_envelope_required': required,
            'approved_drive_test_current_a': drive_current,
            'approved_brake_test_current_a': brake_current,
            'surface_description': str(site.get('surface_description', 'unspecified')),
            'dynamic_condition_schedule': duration_rows,
            'minimum_dynamic_capture_s': min_capture,
            'passed': (not required) or usable >= minimum,
        }
        dump_yaml(self.session / 'preflight' / 'site_envelope.yaml', result)
        return result

    def _workspace(self,requested:Path|None)->Path:
        if requested: return requested.expanduser().resolve()
        return self.root.parents[1]
    def _prepare_session(self)->Path:
        if self.resume:
            p=self.resume.resolve()
            if not (p/'session_manifest.yaml').is_file(): raise FileNotFoundError(f'not an ERPM calibration session: {p}')
            return p
        stamp=dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ'); p=self.runs_dir/f'{stamp}_erpm_longitudinal_calibration'; p.mkdir(parents=True,exist_ok=False)
        copy_config_file(self.config_path,p/'calibration_config_snapshot.yaml'); copy_config_file(self.root/'config'/'topics.yaml',p/'recording_policy_snapshot.yaml'); copy_config_file(self.root/'config'/'bag_qos_overrides.yaml',p/'bag_qos_overrides_snapshot.yaml')
        dump_yaml(p/'session_manifest.yaml',{'session_id':p.name,'created_utc':stamp,'status':'in_progress','workspace':str(self._workspace(None)),'stages':{}}); return p
    def _manifest(self)->dict[str,Any]: return load_yaml(self.session/'session_manifest.yaml')
    def _update(self,**kw:Any)->None:
        m=self._manifest(); m.update(kw); dump_yaml(self.session/'session_manifest.yaml',m)
    def _load_runtime(self)->dict[str,Any]:
        p=self.session/'runtime_state.json'
        return json.loads(p.read_text(encoding='utf-8')) if p.is_file() else {}
    def _save_runtime(self)->None: dump_json(self.session/'runtime_state.json',self.runtime)
    def _disk(self)->None:
        free=shutil.disk_usage(self.session).free/(1024**3); print(disk_line(str(self.session)))
        if free<float(self.config['session']['min_free_disk_gb']): raise RuntimeError(f'need {self.config["session"]["min_free_disk_gb"]} GiB free for stage-targeted MCAP recording')
    def _stop_stack(self)->None:
        if self.stack: self.stack.stop(); self.stack=None
    def _launch(self,profile:str,*,candidate_patch:Path|None=None,candidate_mode:str|None=None)->None:
        self._stop_stack(); self.launch_index+=1; snapshot=self.session/'calibration_config_snapshot.yaml'; log=self.session/'stack_logs'/f'{self.launch_index:02d}_{profile}.log'
        geometry={'launch_index':self.launch_index,'profile':profile,'calibration_config_snapshot':str(snapshot),'hardware':load_yaml(snapshot)['hardware'],'candidate_patch':str(candidate_patch) if candidate_patch else None,'candidate_mode':candidate_mode}; dump_yaml(self.session/'environment'/f'launch_{self.launch_index:02d}.yaml',geometry)
        self.stack=StackProcess(self.root,snapshot,log,candidate_patch=candidate_patch,candidate_mode=candidate_mode); self.stack.start()
    def _cli(self,args:list[str],out:Path,timeout:float=12)->str:
        out.parent.mkdir(parents=True,exist_ok=True)
        try:
            r=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout,check=False); out.write_text(r.stdout,encoding='utf-8')
            if r.returncode: raise RuntimeError(r.stdout.strip())
            return r.stdout
        except subprocess.TimeoutExpired as e: out.write_text(f'TIMEOUT {args!r}\n{e!r}\n',encoding='utf-8'); raise RuntimeError('CLI timeout') from e
    @staticmethod
    def _float(text:str)->float:
        found=re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?',text)
        if not found: raise RuntimeError(f'no numeric ROS parameter response: {text!r}')
        return float(found[-1])
    def _param(self,node:str,name:str,dir:Path)->float:
        return self._float(self._cli(['ros2','param','get',node,name],dir/f'{node.strip("/").replace("/","__")}__{name}.txt'))
    def _verify_live_profile(self,profile:str)->dict[str,Any]:
        e=self.session/'preflight'/f'launch_{self.launch_index:02d}_{profile}'; values={'accel_to_current_gain':self._param('/ackermann_to_vesc_node','accel_to_current_gain',e),'accel_to_brake_gain':self._param('/ackermann_to_vesc_node','accel_to_brake_gain',e),'speed_to_erpm_gain':self._param('/ackermann_to_vesc_node','speed_to_erpm_gain',e),'speed_to_erpm_offset':self._param('/ackermann_to_vesc_node','speed_to_erpm_offset',e)}
        info={}
        for topic in ['/commands/motor/speed','/commands/motor/current','/commands/motor/brake']:
            text=self._cli(['ros2','topic','info','--verbose',topic],e/(topic.strip('/').replace('/','__')+'.txt')); m=re.search(r'Publisher count:\s*(\d+)',text); info[topic]=int(m.group(1)) if m else None
        errors=[]
        preflight=self.config['preflight']
        if profile.startswith('vel'):
            expected_drive=float(preflight['required_accel_to_current_gain_vel'])
            expected_brake=float(preflight['required_accel_to_brake_gain_vel'])
            expected_offset=float(preflight['required_speed_to_erpm_offset_vel'])
            if abs(values['accel_to_current_gain']-expected_drive)>1e-9 or abs(values['accel_to_brake_gain']-expected_brake)>1e-9:
                errors.append('VEL_TO_ERPM profile is not active: acceleration gains differ from required zero values')
            if abs(values['speed_to_erpm_offset']-expected_offset)>1e-9:
                errors.append('VEL_TO_ERPM static map offset is non-zero; the calibration requires ERPM(0)=0')
        if profile.startswith('accel') and (values['accel_to_current_gain']<=0 or values['accel_to_brake_gain']<=0):
            errors.append('ACCEL_TO_CURRENT profile is not active: bootstrap/candidate gains non-positive')
        max_publishers=int(preflight['max_selector_publishers'])
        bad=[topic for topic,c in info.items() if c is None or c>max_publishers or c<1]
        if bad: errors.append(f'motor selector publisher ownership failed (expected exactly {max_publishers} publisher): '+', '.join(bad))
        result={'profile':profile,'observed':values,'publisher_counts':info,'errors':errors}; dump_yaml(e/'profile_check.yaml',result)
        if errors: raise RuntimeError('; '.join(errors))
        return result
    def _done(self,name:str)->bool: return self._manifest().get('stages',{}).get(name,{}).get('status')=='completed'
    def _run_stage(self,name:str,group:str,fn:Callable[[Path],dict[str,Any]])->dict[str,Any]|None:
        if self._done(name): print(f'Skipping completed stage: {name}'); return self.runtime.get(name)
        stage=self.session/name
        if stage.exists():
            archive=self.session/'incomplete_attempts'/f'{name}_{dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")}'; archive.parent.mkdir(parents=True,exist_ok=True); shutil.move(str(stage),str(archive))
        self._disk(); bag:BagProcess|None=None; result=None; status='failed'; error:BaseException|None=None; verify=None
        try:
            policy=load_yaml(self.session/'recording_policy_snapshot.yaml')
            required_topics=list(policy['required'][group])
            recorded_topics=list(dict.fromkeys(required_topics + list(policy.get('redundancy_topics', []))))
            bag=start_bag(stage,policy['recording'],required_topics,recorded_topics,self.root); print(f'MCAP recording started: {bag.bag_dir}'); result=fn(stage); status='completed'
        except KeyboardInterrupt as exc: status='interrupted'; error=exc
        except BaseException as exc: status='failed'; error=exc
        finally:
            if bag:
                verify=stop_bag(bag)
                if status=='completed' and not verify['ok']: status='failed'
            m=self._manifest(); m.setdefault('stages',{})[name]={'status':status,'directory':str(stage.relative_to(self.session)),'bag_verification':verify}; dump_yaml(self.session/'session_manifest.yaml',m); print(f'MCAP recording stopped for {name}: {status}')
        if error: raise error
        if verify and not verify['ok']: raise RuntimeError(f'{name} missing required topics: {verify["missing_or_empty_required_topics"]}')
        assert result is not None; self.runtime[name]=result; self._save_runtime(); return result
    def _initial_map(self, original: dict[str, Any]) -> tuple[float, float]:
        g = original.get('speed_to_erpm_gain')
        if not isinstance(g, (int, float)) or float(g) <= 0:
            raise RuntimeError('original speed_to_erpm_gain must be positive')
        # All raw ERPM stages use a constrained zero intercept.  The original
        # offset is preserved in the byte-exact backup but never used to create
        # an experimental wheel-speed target.
        return float(g), 0.0
    def _run_offline_analysis(self) -> None:
        log = self.session / 'analysis' / 'run_analysis.log'
        log.parent.mkdir(exist_ok=True)
        result = subprocess.run(['python3', str(self.root / 'analysis' / 'run_analysis.py'), str(self.session)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        log.write_text(result.stdout, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(f'offline analysis failed; inspect {log}')

    def _run_analysis_script(self, script_name: str, log_name: str) -> None:
        log = self.session / 'analysis' / log_name
        log.parent.mkdir(exist_ok=True)
        result = subprocess.run(
            [sys.executable, str(self.root / 'analysis' / script_name), str(self.session)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False,
        )
        log.write_text(result.stdout, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(f'{script_name} failed; inspect {log}')

    def _export_stage_for_analysis(self, stage: str) -> None:
        bag = self.session / stage / 'bag'
        if not bag.is_dir():
            raise RuntimeError(f'missing bag for stage {stage}: {bag}')
        export = subprocess.run(
            [sys.executable, str(self.root / 'analysis' / 'export_bag.py'), str(bag)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False,
        )
        if export.returncode:
            raise RuntimeError(f'export_bag.py failed for {stage}')
        if stage != '00_command_chain_audit':
            lidar = subprocess.run(
                [sys.executable, str(self.root / 'analysis' / 'estimate_lidar_motion.py'), str(bag),
                 '--config', str(self.session / 'calibration_config_snapshot.yaml')],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False,
            )
            if lidar.returncode:
                raise RuntimeError(f'estimate_lidar_motion.py failed for {stage}')

    @staticmethod
    def _finite_or(value: Any, fallback: float) -> float:
        try:
            candidate = float(value)
        except (TypeError, ValueError):
            return float(fallback)
        return candidate if math.isfinite(candidate) else float(fallback)

    def _interim_velocity_patch(self) -> tuple[dict[str, Any], dict[str, Any]]:
        report = load_yaml(self.session / 'analysis' / 'erpm_speed_map_report.yaml')
        command_kind = str(report['command_map']['selected_model'])
        quadratic = float(report.get('candidate_speed_to_erpm_quadratic', 0.0))
        odom_scale = self._finite_or(report.get('candidate_odom_speed_scale'), self._finite_or(report.get('current_odom_speed_scale'), 1.0))
        full = {
            'ackermann_to_vesc_node': {
                'ros__parameters': {
                    'speed_command_model': command_kind,
                    'speed_to_erpm_gain': float(report['candidate_speed_to_erpm_gain']),
                    'speed_to_erpm_quadratic': quadratic if command_kind == 'quadratic' else 0.0,
                    'speed_to_erpm_offset': 0.0,
                    'slow_start_threshold': float(report['candidate_slow_start_threshold_mps']),
                    'slow_start_increment': float(report['candidate_slow_start_increment_mps']),
                    'stop_speed_deadzone': float(report['candidate_stop_speed_deadzone_mps']),
                }
            },
            'vesc_to_odom_node': {
                'ros__parameters': {
                    'odom_speed_scale': odom_scale,
                    'speed_deadband': float(report['candidate_odom_speed_deadband_mps']),
                }
            },
        }
        flat = self._flatten_candidate(full)
        return full, flat

    def _run_velocity_checkpoint(self) -> tuple[dict[str, Any], dict[str, Any]]:
        print('Running interim speed-map checkpoint from Stages 1-5 to improve subsequent setup-speed targets and runtime odometry...')
        for stage in (
            '01_longitudinal_observability',
            '02_low_speed_launch',
            '03_raw_erpm_map_training',
            '04_raw_erpm_map_holdout',
            '05_vel_to_erpm_pipeline_audit',
        ):
            self._export_stage_for_analysis(stage)
        self._run_analysis_script('fit_speed_map.py', 'checkpoint_fit_speed_map.log')
        full, flat = self._interim_velocity_patch()
        self.runtime['interim_velocity_patch'] = flat
        self._save_runtime()
        return full, flat

    def _interim_accel_patch(self, velocity_patch_full: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        speed = load_yaml(self.session / 'analysis' / 'erpm_speed_map_report.yaml')
        drag = load_yaml(self.session / 'analysis' / 'coastdown_drag_report.yaml')
        current = load_yaml(self.session / 'analysis' / 'current_acceleration_report.yaml')
        full = {
            'ackermann_to_vesc_node': {
                'ros__parameters': {
                    **velocity_patch_full.get('ackermann_to_vesc_node', {}).get('ros__parameters', {}),
                    'acceleration_command_model': 'scalar',
                    'accel_to_current_gain': float(current['candidate_accel_to_current_gain']),
                    'accel_to_brake_gain': float(current['candidate_accel_to_brake_gain']),
                    'accel_deadzone': float(current['candidate_accel_deadzone_mps2']),
                    'accel_drag_coulomb': float(drag['accel_drag_coulomb_mps2']),
                    'accel_drag_viscous': float(drag['accel_drag_viscous_per_s']),
                    'accel_drag_quadratic': float(drag['accel_drag_quadratic_per_m']),
                    'max_drive_current': float(self.config['operating_envelope']['approved_drive_test_current_a']),
                    'max_brake_current': float(self.config['operating_envelope']['approved_brake_test_current_a']),
                    'speed_to_braking_max': float(self.config['operating_envelope']['approved_brake_test_current_a']),
                    'stop_speed_deadzone': float(speed['candidate_stop_speed_deadzone_mps']),
                    'speed_to_erpm_offset': 0.0,
                }
            },
            'vesc_to_odom_node': {
                'ros__parameters': {
                    **velocity_patch_full.get('vesc_to_odom_node', {}).get('ros__parameters', {}),
                }
            },
        }
        flat = self._flatten_candidate(full)
        return full, flat

    def _run_forward_motion_checkpoint(self, velocity_patch_full: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        print('Running forward-motion checkpoint from Stages 6-9 to learn drag and bootstrap acceleration gains before Stage 10...')
        for stage in (
            '06_raw_erpm_response',
            '07_coastdown',
            '08_raw_current_training',
            '09_raw_current_holdout',
        ):
            self._export_stage_for_analysis(stage)
        for script, log_name in (
            ('fit_erpm_response.py', 'checkpoint_fit_erpm_response.log'),
            ('fit_coastdown.py', 'checkpoint_fit_coastdown.log'),
            ('fit_current_model.py', 'checkpoint_fit_current_model.log'),
            ('fit_traction_transients.py', 'checkpoint_fit_traction_transients.log'),
        ):
            self._run_analysis_script(script, log_name)
        full, flat = self._interim_accel_patch(velocity_patch_full)
        self.runtime['interim_accel_patch'] = flat
        self._save_runtime()
        return full, flat

    @staticmethod
    def _flatten_candidate(patch: dict[str, Any]) -> dict[str, Any]:
        flat: dict[str, Any] = {}
        for node in ('/**', 'ackermann_to_vesc_node', 'vesc_to_odom_node'):
            section = patch.get(node, {}) if isinstance(patch, dict) else {}
            params = section.get('ros__parameters', {}) if isinstance(section, dict) else {}
            if isinstance(params, dict):
                flat.update(params)
        return flat

    def _run_temporary_candidate_verification(self) -> dict[str, Any]:
        summary_path = self.session / 'analysis' / 'longitudinal_candidate_summary.yaml'
        if not summary_path.is_file():
            raise RuntimeError('analysis did not produce longitudinal_candidate_summary.yaml')
        summary = load_yaml(summary_path)
        if not bool(summary.get('accepted_for_temporary_candidate_verification')):
            return {'status': 'not_run_candidate_rejected', 'summary': str(summary_path)}
        patch = self._flatten_candidate(load_yaml(self.session / 'analysis' / 'candidate_vesc_patch.yaml'))
        candidate_gain = float(patch['speed_to_erpm_gain'])
        candidate_offset = float(patch.get('speed_to_erpm_offset', 0.0))
        self._stop_stack()
        self.transaction.apply_profile('vel_to_erpm_candidate', candidate_patch=patch)
        self._launch('vel_to_erpm_candidate', candidate_patch=self.session/'analysis'/'candidate_vesc_patch.yaml', candidate_mode='velocity')
        self._verify_live_profile('vel_to_erpm_candidate')
        self._run_stage('11_candidate_velocity_verification', 'candidate_ackermann_vel',
                        lambda d: candidate_velocity_verification(self.config, d, self.counter))
        self._stop_stack()
        self.transaction.apply_profile('accel_to_current_candidate', candidate_patch=patch)
        self._launch('accel_to_current_candidate', candidate_patch=self.session/'analysis'/'candidate_vesc_patch.yaml', candidate_mode='accel')
        self._verify_live_profile('accel_to_current_candidate')
        candidate_patch_full = load_yaml(self.session / 'analysis' / 'candidate_vesc_patch.yaml')
        self._run_stage('12_candidate_accel_verification', 'candidate_ackermann_accel',
                        lambda d: candidate_accel_verification(
                            self.config, d, candidate_gain, candidate_offset, self.counter, candidate_patch_full
                        ))
        # Stage 13 is a speed-estimation hold-out and must run with the candidate
        # VEL_TO_ERPM command map, not under the ACCEL_TO_CURRENT profile used by
        # Stage 12.  Re-launching here keeps its controller semantics unambiguous.
        self._stop_stack()
        self.transaction.apply_profile('vel_to_erpm_candidate', candidate_patch=patch)
        self._launch('vel_to_erpm_candidate_cross_axis', candidate_patch=self.session/'analysis'/'candidate_vesc_patch.yaml', candidate_mode='velocity')
        self._verify_live_profile('vel_to_erpm_candidate_cross_axis')
        self._run_stage('13_candidate_cross_axis_verification', 'candidate_ackermann_vel',
                        lambda d: candidate_cross_axis_verification(self.config, d, self.counter))
        verify_log = self.session / 'analysis' / 'verify_candidate.log'
        logs = []
        for stage in ('11_candidate_velocity_verification', '12_candidate_accel_verification', '13_candidate_cross_axis_verification'):
            bag = self.session / stage / 'bag'
            exported = subprocess.run(['python3', str(self.root / 'analysis' / 'export_bag.py'), str(bag)],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
            logs.append(exported.stdout)
            if exported.returncode: raise RuntimeError(f'candidate bag export failed: {stage}')
            matched = subprocess.run(['python3', str(self.root / 'analysis' / 'estimate_lidar_motion.py'), str(bag),
                                     '--config', str(self.session / 'calibration_config_snapshot.yaml')],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
            logs.append(matched.stdout)
            if matched.returncode: raise RuntimeError(f'candidate LiDAR motion estimate failed: {stage}')
        verify_log.write_text('\n'.join(logs), encoding='utf-8')
        result = subprocess.run(['python3', str(self.root / 'analysis' / 'verify_candidate.py'), str(self.session)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        (self.session / 'analysis' / 'verify_candidate.log').write_text(result.stdout, encoding='utf-8')
        if result.returncode: raise RuntimeError('candidate verification analysis failed')
        # Fit the cornering longitudinal-slip correction from the Stage 13 arcs so
        # odometry is trustworthy while turning, not only on straights. Additive:
        # a failure here is logged but does not discard the verified candidate.
        slip = subprocess.run(['python3', str(self.root / 'analysis' / 'fit_turn_slip.py'), str(self.session)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        (self.session / 'analysis' / 'fit_turn_slip.log').write_text(slip.stdout, encoding='utf-8')
        report = load_yaml(self.session / 'analysis' / 'candidate_deployment_verification_report.yaml')
        accepted = bool(report.get('accepted_for_permanent_review'))
        return {
            'status': 'completed_candidate_accepted_for_review' if accepted else 'completed_candidate_rejected',
            'report': report,
        }

    def run_preflight_only(self) -> None:
        banner('ERPM FULL-ENVELOPE PREFLIGHT',
               'Checks workspace, disk, site length and reviewed current limits without touching VESC YAML or launching ROS.')
        print(f'Session directory: {self.session}')
        try:
            self._disk()
            site = self._verify_site_envelope()
            self._update(
                status='preflight_ready',
                preflight_only=True,
                completed_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                site_preflight=site,
            )
            print('Preflight passed. No VESC configuration changes, builds, ROS launches or bag recordings were started.')
            print('Run the normal calibration command when the full track, reviewed current limits and operator are ready.')
        except Exception:
            self._update(
                status='failed_preflight',
                preflight_only=True,
                completed_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
            )
            raise

    def _emit_config_only_patch(self) -> None:
        """Write the directly-deployable vesc.yaml parameters (Tier 1).

        These keys are already supported by the production C++ nodes
        (``ackermann_to_vesc`` / ``vesc_to_odom``): the ERPM/odom scalar gain,
        odom scale, accel/brake current gains, the coast-down drag feed-forward
        (which makes ``a=0`` hold speed instead of coasting) and the current
        clamps. They can be copied straight into ``vesc.yaml`` with NO C++ edit.
        Quadratic/LUT command maps, the fused estimator, the traction surface and
        the turn-slip correction are NOT here — they need the C++ port and are
        validated by the Tier-2 candidate-port run.
        """
        a = self.session / 'analysis'
        speed = load_yaml(a / 'erpm_speed_map_report.yaml')
        drag = load_yaml(a / 'coastdown_drag_report.yaml')
        current = load_yaml(a / 'current_acceleration_report.yaml')
        command_kind = str(speed['command_map']['selected_model'])
        gain = float(speed['candidate_speed_to_erpm_gain'])
        odom_scale = self._finite_or(speed.get('candidate_odom_speed_scale'),
                                     self._finite_or(speed.get('current_odom_speed_scale'), 1.0))
        env = self.config['operating_envelope']
        patch = {
            'note': ('Tier-1 config-only parameters. The production C++ already supports these keys; '
                     'copy them into vesc.yaml with NO code edit. Linear command map only — quadratic/LUT '
                     'maps, fused odometry, the traction surface and the turn-slip correction require the '
                     'C++ port and are produced/validated by erpm_candidate_port.py.'),
            'directly_config_deployable': bool(command_kind == 'linear'),
            'selected_command_map': command_kind,
            'ackermann_to_vesc_node': {'ros__parameters': {
                'speed_to_erpm_gain': gain,
                'speed_to_erpm_offset': 0.0,
                'accel_to_current_gain': float(current['candidate_accel_to_current_gain']),
                'accel_to_brake_gain': float(current['candidate_accel_to_brake_gain']),
                'accel_deadzone': float(current['candidate_accel_deadzone_mps2']),
                'accel_drag_coulomb': float(drag['accel_drag_coulomb_mps2']),
                'accel_drag_viscous': float(drag['accel_drag_viscous_per_s']),
                'accel_drag_quadratic': float(drag['accel_drag_quadratic_per_m']),
                'max_drive_current': float(env['approved_drive_test_current_a']),
                'max_brake_current': float(env['approved_brake_test_current_a']),
                'slow_start_threshold': float(speed['candidate_slow_start_threshold_mps']),
                'slow_start_increment': float(speed['candidate_slow_start_increment_mps']),
                'stop_speed_deadzone': float(speed['candidate_stop_speed_deadzone_mps']),
            }},
            'vesc_to_odom_node': {'ros__parameters': {
                'speed_to_erpm_gain': gain,
                'speed_to_erpm_offset': 0.0,
                'odom_speed_scale': odom_scale,
                'speed_deadband': float(speed['candidate_odom_speed_deadband_mps']),
            }},
        }
        if command_kind != 'linear':
            patch['warning'] = (f"selected command map is '{command_kind}', which the current C++ command "
                                "conversion cannot represent; the linear gain above is the safe config-only "
                                "fallback. Run erpm_candidate_port.py and the C++ port to deploy the full map.")
        dump_yaml(a / 'config_only_vesc_patch.yaml', patch)
        print(f'Tier-1 config-only patch written: {a / "config_only_vesc_patch.yaml"}')

    def run(self, *, identification: bool = True, candidate: bool = True) -> None:
        title = ('FULL ERPM / LONGITUDINAL CALIBRATION' if (identification and candidate)
                 else 'ERPM CONFIG-DEPLOYABLE CALIBRATION (TIER 1)' if identification
                 else 'ERPM CANDIDATE SHADOW VERIFICATION (TIER 2 — needs C++ port to deploy)')
        banner(f'PREPARING {title}', 'The runner owns temporary profiles, all builds, dedicated launch, MCAP bags, and automatic restoration.')
        print(f'Session directory: {self.session}'); print('The operator does not edit vesc.yaml, build, launch ROS or start rosbag manually.')
        try:
            site = self._verify_site_envelope()
            self._update(site_preflight=site)
            tx=self.transaction.begin(); gain,offset=self._initial_map(tx['original_values']); self.runtime['initial_speed_map']={'gain':gain,'offset':offset}; self._save_runtime(); self._update(vesc_config_transaction=tx)
            if identification:
                self.transaction.apply_profile('vel_to_erpm'); self._launch('vel_to_erpm'); self._verify_live_profile('vel_to_erpm')
                require_ready('Type READY to begin Stage 0, or ABORT')
                for name, group in [
                    ('00_command_chain_audit', 'command_audit'),
                    ('01_longitudinal_observability', 'raw_erpm'),
                    ('02_low_speed_launch', 'raw_erpm'),
                    ('03_raw_erpm_map_training', 'raw_erpm'),
                    ('04_raw_erpm_map_holdout', 'raw_erpm'),
                    ('05_vel_to_erpm_pipeline_audit', 'ackermann_vel'),
                ]:
                    self._run_stage(name, group, lambda d, n=name: run_stage(n, self.config, d, gain, offset, self.counter))

                self._stop_stack()
                velocity_patch_full, velocity_patch_flat = self._run_velocity_checkpoint()
                self.transaction.apply_profile('vel_to_erpm_interim', candidate_patch=velocity_patch_flat)
                self._launch('vel_to_erpm_interim')
                self._verify_live_profile('vel_to_erpm_interim')

                for name, group in [
                    ('06_raw_erpm_response', 'raw_erpm'),
                    ('07_coastdown', 'raw_current'),
                    ('08_raw_current_training', 'raw_current'),
                    ('09_raw_current_holdout', 'raw_current'),
                ]:
                    self._run_stage(
                        name, group,
                        lambda d, n=name: run_stage(n, self.config, d, gain, offset, self.counter, speed_command_patch=velocity_patch_full),
                    )

                self._stop_stack()
                accel_patch_full, accel_patch_flat = self._run_forward_motion_checkpoint(velocity_patch_full)
                self.transaction.apply_profile('accel_to_current_interim', candidate_patch=accel_patch_flat)
                self._launch('accel_to_current_interim')
                self._verify_live_profile('accel_to_current_interim')
                self._run_stage(
                    '10_accel_to_current_interface', 'ackermann_accel',
                    lambda d: run_stage('10_accel_to_current_interface', self.config, d, gain, offset, self.counter, speed_command_patch=accel_patch_full),
                )
                self._stop_stack()
                print('Running strict offline analysis...')
                self._run_offline_analysis()
                self._emit_config_only_patch()
            elif not (self.session / 'analysis' / 'longitudinal_candidate_summary.yaml').is_file():
                # Tier-2 on a resumed identification session: regenerate the
                # candidate from the existing raw bags if it is not present.
                print('Regenerating candidate from existing raw bags...')
                self._run_offline_analysis()

            if candidate:
                verification = self._run_temporary_candidate_verification()
                verification_accepted = bool(verification.get('report', {}).get('accepted_for_permanent_review'))
                final_status = 'completed' if verification_accepted else 'completed_candidate_rejected'
                self._update(status=final_status, completed_utc=dt.datetime.now(dt.timezone.utc).isoformat(), candidate_verification=verification)
                if verification_accepted:
                    banner('CANDIDATE SHADOW VERIFICATION COMPLETE')
                    print('Candidate passed all coverage, RMSE and signed-bias gates. It is eligible for human permanent-review (C++ port) only.')
                else:
                    banner('CANDIDATE REJECTED')
                    print('Raw evidence is preserved, but the candidate did not pass complete deployment verification.')
            else:
                self._update(status='completed_config_only', completed_utc=dt.datetime.now(dt.timezone.utc).isoformat())
                banner('TIER 1 COMPLETE — CONFIG-DEPLOYABLE PARAMETERS READY')
                print('Apply analysis/config_only_vesc_patch.yaml to vesc.yaml. No C++ edit is required for these.')
                print('For quadratic/LUT maps, fused odometry, the traction surface and the turn-slip correction,')
                print('run erpm_candidate_port.py on this session (those need the C++ port).')
            print(f'All raw bags and outputs are in: {self.session}')
            print('The original VESC configuration will now be restored automatically. Calibrated values are never installed permanently by this runner.')
        except KeyboardInterrupt:
            self._update(status='interrupted'); print(f'Interrupted; raw bags preserved. Resume with --resume {self.session}'); raise
        except Exception:
            self._update(status='failed'); raise
        finally:
            self._stop_stack(); restored=self.transaction.restore(build=True)
            if restored: self._update(vesc_config_transaction=restored); print('Original VESC YAML restored byte-for-byte and workspace rebuilt.')

    def run_identification(self) -> None:
        """Tier 1: gather data, fit, and emit config-only vesc.yaml parameters."""
        self.run(identification=True, candidate=False)

    def run_candidate_port(self) -> None:
        """Tier 2: candidate shadow verification (needs the C++ port to deploy)."""
        self.run(identification=False, candidate=True)
