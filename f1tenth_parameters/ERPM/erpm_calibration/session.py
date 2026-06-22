"""Session orchestration for the all-topics ERPM longitudinal campaign."""
from __future__ import annotations
import datetime as dt, hashlib, json, os, re, shutil, signal, subprocess, time
from pathlib import Path
from typing import Any, Callable
from .bagging import BagProcess, start_bag, stop_bag
from .config import copy_config_file, dump_json, dump_yaml, load_yaml
from .config_transaction import VescModeTransaction
from .stages import TrialCounter, run_stage, candidate_velocity_verification, candidate_accel_verification
from .ui import banner, disk_line, require_ready

class StackProcess:
    def __init__(self, root:Path, config:Path, log:Path) -> None:
        self.root=root; self.config=config; self.log=log; self.proc:subprocess.Popen|None=None; self.handle=None
    def start(self)->None:
        self.log.parent.mkdir(parents=True,exist_ok=True); self.handle=self.log.open('w',encoding='utf-8')
        self.proc=subprocess.Popen(['python3',str(self.root/'launch'/'calibration_stack.py'),'--config',str(self.config)],stdout=self.handle,stderr=subprocess.STDOUT,start_new_session=True,text=True)
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
        if free<float(self.config['session']['min_free_disk_gb']): raise RuntimeError(f'need {self.config["session"]["min_free_disk_gb"]} GiB free for all-topics MCAP recording')
    def _stop_stack(self)->None:
        if self.stack: self.stack.stop(); self.stack=None
    def _launch(self,profile:str)->None:
        self._stop_stack(); self.launch_index+=1; snapshot=self.session/'calibration_config_snapshot.yaml'; log=self.session/'stack_logs'/f'{self.launch_index:02d}_{profile}.log'
        geometry={'launch_index':self.launch_index,'profile':profile,'calibration_config_snapshot':str(snapshot),'hardware':load_yaml(snapshot)['hardware']}; dump_yaml(self.session/'environment'/f'launch_{self.launch_index:02d}.yaml',geometry)
        self.stack=StackProcess(self.root,snapshot,log); self.stack.start()
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
            if abs(values['accel_to_current_gain']-expected_drive)>1e-9 or abs(values['accel_to_brake_gain']-expected_brake)>1e-9:
                errors.append('VEL_TO_ERPM profile is not active: acceleration gains differ from required zero values')
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
            policy=load_yaml(self.session/'recording_policy_snapshot.yaml'); bag=start_bag(stage,policy['recording'],list(policy['required'][group]),self.root); print(f'MCAP recording started: {bag.bag_dir}'); result=fn(stage); status='completed'
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
    def _initial_map(self,original:dict[str,Any])->tuple[float,float]:
        g=original.get('speed_to_erpm_gain'); o=original.get('speed_to_erpm_offset')
        if not isinstance(g,(int,float)) or float(g)<=0: raise RuntimeError('original speed_to_erpm_gain must be positive')
        return float(g),float(o or 0.0)
    def _run_offline_analysis(self) -> None:
        log = self.session / 'analysis' / 'run_analysis.log'
        log.parent.mkdir(exist_ok=True)
        result = subprocess.run(['python3', str(self.root / 'analysis' / 'run_analysis.py'), str(self.session)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        log.write_text(result.stdout, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(f'offline analysis failed; inspect {log}')

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
        self._launch('vel_to_erpm_candidate')
        self._verify_live_profile('vel_to_erpm_candidate')
        self._run_stage('11_candidate_velocity_verification', 'ackermann_vel',
                        lambda d: candidate_velocity_verification(self.config, d, self.counter))
        self._stop_stack()
        self.transaction.apply_profile('accel_to_current_candidate', candidate_patch=patch)
        self._launch('accel_to_current_candidate')
        self._verify_live_profile('accel_to_current_candidate')
        self._run_stage('12_candidate_accel_verification', 'ackermann_accel',
                        lambda d: candidate_accel_verification(self.config, d, candidate_gain, candidate_offset, self.counter))
        verify_log = self.session / 'analysis' / 'verify_candidate.log'
        logs = []
        for stage in ('11_candidate_velocity_verification', '12_candidate_accel_verification'):
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
        report = load_yaml(self.session / 'analysis' / 'candidate_deployment_verification_report.yaml')
        return {'status': 'completed', 'report': report}

    def run(self)->None:
        banner('PREPARING FULL ERPM / LONGITUDINAL CALIBRATION','The runner owns temporary profiles, all builds, dedicated launch, MCAP bags, and automatic restoration.')
        print(f'Session directory: {self.session}'); print('The operator does not edit vesc.yaml, build, launch ROS or start rosbag manually.')
        try:
            tx=self.transaction.begin(); gain,offset=self._initial_map(tx['original_values']); self.runtime['initial_speed_map']={'gain':gain,'offset':offset}; self._save_runtime(); self._update(vesc_config_transaction=tx)
            self.transaction.apply_profile('vel_to_erpm'); self._launch('vel_to_erpm'); self._verify_live_profile('vel_to_erpm')
            require_ready('Type READY to begin Stage 0, or ABORT')
            stages=[
              ('00_command_chain_audit','command_audit'),('01_longitudinal_observability','raw_erpm'),('02_low_speed_launch','raw_erpm'),('03_raw_erpm_map_training','raw_erpm'),('04_raw_erpm_map_holdout','raw_erpm'),('05_vel_to_erpm_pipeline_audit','ackermann_vel'),('06_raw_erpm_response','raw_erpm'),('07_coastdown','raw_current'),('08_raw_current_training','raw_current'),('09_raw_current_holdout','raw_current')]
            for name,group in stages: self._run_stage(name,group,lambda d,n=name: run_stage(n,self.config,d,gain,offset,self.counter))
            self._stop_stack(); self.transaction.apply_profile('accel_to_current_bootstrap'); self._launch('accel_to_current_bootstrap'); self._verify_live_profile('accel_to_current_bootstrap')
            self._run_stage('10_accel_to_current_interface','ackermann_accel',lambda d:run_stage('10_accel_to_current_interface',self.config,d,gain,offset,self.counter))
            self._stop_stack()
            print('Running strict offline analysis to generate a temporary candidate and its acceptance decision...')
            self._run_offline_analysis()
            verification = self._run_temporary_candidate_verification()
            self._update(status='completed',completed_utc=dt.datetime.now(dt.timezone.utc).isoformat(),candidate_verification=verification)
            banner('ERPM CALIBRATION AND TEMPORARY CANDIDATE VERIFICATION COMPLETE')
            print(f'All raw bags, candidate outputs and verification evidence are in: {self.session}')
            print('The original VESC configuration will now be restored automatically. Candidate values are never installed permanently by this runner.')
        except KeyboardInterrupt:
            self._update(status='interrupted'); print(f'Interrupted; raw bags preserved. Resume with --resume {self.session}'); raise
        except Exception:
            self._update(status='failed'); raise
        finally:
            self._stop_stack(); restored=self.transaction.restore(build=True)
            if restored: self._update(vesc_config_transaction=restored); print('Original VESC YAML restored byte-for-byte and workspace rebuilt.')
