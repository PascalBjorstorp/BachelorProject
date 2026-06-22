"""Reversible multi-profile VESC configuration transaction for ERPM calibration.

The calibration campaign deliberately exercises two controller modes.  The
operator never edits source YAML: this transaction snapshots `vesc.yaml`,
installs a temporary VEL_TO_ERPM profile, later installs a temporary
ACCEL_TO_CURRENT bootstrap/candidate profile, rebuilds after each change, and
restores the original *byte-for-byte* at completion or recovery.
"""
from __future__ import annotations
import hashlib, json, os, shutil, subprocess, tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml

class ConfigTransactionError(RuntimeError): pass

class VescModeTransaction:
    LOCK_NAME = '.ERPM_CALIBRATION_RECOVERY.json'
    def __init__(self, *, calibration_root: Path, session_dir: Path, workspace: Path | None,
                 config_relpath: str, build_command: list[str], profiles: dict[str, Any]) -> None:
        self.root=calibration_root.resolve(); self.session_dir=session_dir.resolve(); self.config_relpath=config_relpath
        self.workspace=self._resolve_workspace(workspace); self.config_path=(self.workspace/config_relpath).resolve()
        self.build_command=list(build_command); self.profiles=profiles
        self.directory=self.session_dir/'vesc_config_transaction'; self.backup=self.directory/'vesc.yaml.original_bytes'
        self.history=self.directory/'profile_history.json'; self.lock=self.workspace/self.LOCK_NAME
        self.active=False

    def _resolve_workspace(self, requested: Path | None) -> Path:
        candidates=[]
        if requested: candidates.append(requested.expanduser().resolve())
        if os.environ.get('ERPM_CALIBRATION_WORKSPACE'): candidates.append(Path(os.environ['ERPM_CALIBRATION_WORKSPACE']).expanduser().resolve())
        candidates.extend(self.root.parents)
        for candidate in candidates:
            if (candidate/self.config_relpath).is_file(): return candidate
        raise ConfigTransactionError(f'cannot locate {self.config_relpath}; pass --workspace')

    @staticmethod
    def _sha(path: Path) -> str:
        h=hashlib.sha256(); h.update(path.read_bytes()); return h.hexdigest()
    @staticmethod
    def _params(doc: dict[str, Any], key: str) -> dict[str, Any]:
        try: result=doc[key]['ros__parameters']
        except Exception as exc: raise ConfigTransactionError(f'missing {key}.ros__parameters in VESC config') from exc
        if not isinstance(result,dict): raise ConfigTransactionError(f'{key}.ros__parameters not mapping')
        return result
    def _read_backup_yaml(self) -> dict[str, Any]:
        doc=yaml.safe_load(self.backup.read_text(encoding='utf-8'))
        if not isinstance(doc,dict): raise ConfigTransactionError('backup VESC config not YAML mapping')
        return doc
    def _atomic_write(self, doc: dict[str, Any]) -> None:
        with tempfile.NamedTemporaryFile('w',encoding='utf-8',dir=self.config_path.parent,prefix='.erpm_',suffix='.tmp',delete=False) as f:
            yaml.safe_dump(doc,f,sort_keys=False); tmp=Path(f.name)
        os.replace(tmp,self.config_path)
    def _write_lock(self, payload: dict[str,Any]) -> None:
        self.lock.write_text(json.dumps(payload,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    def _metadata(self) -> dict[str,Any]:
        try: return json.loads(self.history.read_text(encoding='utf-8'))
        except Exception: return {'workspace':str(self.workspace),'source_config':str(self.config_path),'backup':str(self.backup),'profiles':[]}
    def _save_metadata(self, record: dict[str,Any]) -> None:
        self.directory.mkdir(parents=True,exist_ok=True); self.history.write_text(json.dumps(record,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    def _build(self, log_name: str) -> None:
        if shutil.which(self.build_command[0]) is None: raise ConfigTransactionError(f'build executable missing: {self.build_command[0]}')
        self.directory.mkdir(parents=True,exist_ok=True); log=self.directory/log_name
        with log.open('w',encoding='utf-8') as h:
            result=subprocess.run(self.build_command,cwd=self.workspace,stdout=h,stderr=subprocess.STDOUT,text=True,check=False)
        if result.returncode:
            tail='\n'.join(log.read_text(encoding='utf-8',errors='replace').splitlines()[-80:])
            raise ConfigTransactionError(f'colcon build failed while applying ERPM calibration profile; log tail:\n{tail}')
    def begin(self) -> dict[str,Any]:
        if self.lock.exists(): raise ConfigTransactionError(f'unrestored ERPM transaction exists; run --recover for {self.workspace}')
        if not self.config_path.is_file(): raise ConfigTransactionError(f'VESC config missing: {self.config_path}')
        self.directory.mkdir(parents=True,exist_ok=True); shutil.copy2(self.config_path,self.backup)
        doc=self._read_backup_yaml(); ack=self._params(doc,'ackermann_to_vesc_node'); glob=self._params(doc,'/**'); odom=self._params(doc,'vesc_to_odom_node')
        metadata={'state':'active','created_utc':datetime.now(timezone.utc).isoformat(),'workspace':str(self.workspace),'source_config':str(self.config_path),'backup':str(self.backup),'original_sha256':self._sha(self.backup),'original_values':{
          'speed_to_erpm_gain':glob.get('speed_to_erpm_gain'),'speed_to_erpm_offset':glob.get('speed_to_erpm_offset'),'servo_min':glob.get('servo_min'),'servo_max':glob.get('servo_max'),'current_max':glob.get('current_max'),'brake_max':glob.get('brake_max'),
          'accel_to_current_gain':ack.get('accel_to_current_gain'),'accel_to_brake_gain':ack.get('accel_to_brake_gain'),'accel_deadzone':ack.get('accel_deadzone'),
          'accel_drag_coulomb':ack.get('accel_drag_coulomb'),'accel_drag_viscous':ack.get('accel_drag_viscous'),'accel_drag_quadratic':ack.get('accel_drag_quadratic'),
          'slow_start_threshold':ack.get('slow_start_threshold'),'slow_start_increment':ack.get('slow_start_increment'),'stop_speed_deadzone':ack.get('stop_speed_deadzone'),
          'speed_to_braking_max':ack.get('speed_to_braking_max'),'odom_speed_scale':odom.get('odom_speed_scale'),'speed_deadband':odom.get('speed_deadband'),
        },'profiles':[],'restore_required':True}
        self._write_lock(metadata); self._save_metadata(metadata); self.active=True
        return metadata
    def original_values(self) -> dict[str,Any]:
        return dict(self._metadata().get('original_values',{}))
    def apply_profile(self, name: str, *, candidate_patch: dict[str,Any] | None = None) -> dict[str,Any]:
        if not self.active and not self.lock.exists(): raise ConfigTransactionError('transaction not begun')
        doc=self._read_backup_yaml(); glob=self._params(doc,'/**'); ack=self._params(doc,'ackermann_to_vesc_node'); odom=self._params(doc,'vesc_to_odom_node')
        original=self.original_values(); patch={'profile':name}
        if name=='vel_to_erpm':
            ack['operation_mode']='VEL_TO_ERPM'; ack['accel_to_current_gain']=0.0; ack['accel_to_brake_gain']=0.0
            # The static ERPM map is constrained through (0, 0).  A low-speed
            # launch threshold is handled by slow-start logic, never by a global
            # ERPM intercept that would corrupt VESC-derived odometry.
            glob['speed_to_erpm_offset']=0.0
            patch.update({'operation_mode':'VEL_TO_ERPM','accel_to_current_gain':0.0,'accel_to_brake_gain':0.0,'speed_to_erpm_offset':0.0})
        elif name in {'accel_to_current_bootstrap','accel_to_current_candidate','accel_to_current_interim'}:
            spec=self.profiles['accel_to_current_bootstrap']
            drive=original.get('accel_to_current_gain')
            brake=original.get('accel_to_brake_gain')
            if not isinstance(drive,(int,float)) or float(drive)<=0: drive=float(spec['fallback_accel_to_current_gain'])
            if not isinstance(brake,(int,float)) or float(brake)<=0: brake=float(spec['fallback_accel_to_brake_gain'])
            ack['operation_mode']='ACCEL_TO_CURRENT'; ack['accel_to_current_gain']=float(drive); ack['accel_to_brake_gain']=float(brake)
            # Keep the zero-speed ERPM invariant across profiles. Although
            # ACCEL_TO_CURRENT does not command motor speed through this map,
            # vesc_to_odom continues to consume it.
            glob['speed_to_erpm_offset']=0.0
            patch.update({'operation_mode':'ACCEL_TO_CURRENT','accel_to_current_gain':float(drive),'accel_to_brake_gain':float(brake),'speed_to_erpm_offset':0.0})
            if name in {'accel_to_current_candidate','accel_to_current_interim'}:
                if not candidate_patch: raise ConfigTransactionError('candidate profile needs candidate patch')
                for key in ['accel_to_current_gain','accel_to_brake_gain','accel_deadzone','accel_drag_coulomb','accel_drag_viscous','accel_drag_quadratic','max_drive_current','max_brake_current','max_regen_input_current','slow_start_threshold','slow_start_increment','stop_speed_deadzone','speed_to_braking_max']:
                    if key in candidate_patch: ack[key]=candidate_patch[key]
                # A candidate can change the scalar gain but may never
                # reintroduce an ERPM intercept: E(0) = 0 is a hard model
                # constraint, not an optional candidate value.
                if 'speed_to_erpm_gain' in candidate_patch:
                    glob['speed_to_erpm_gain']=candidate_patch['speed_to_erpm_gain']
                glob['speed_to_erpm_offset']=0.0
                for key in ['odom_speed_scale','speed_deadband']:
                    if key in candidate_patch: odom[key]=candidate_patch[key]
                patch['candidate_patch']=candidate_patch
        elif name in {'vel_to_erpm_candidate','vel_to_erpm_interim'}:
            if not candidate_patch: raise ConfigTransactionError('candidate VEL profile needs candidate patch')
            ack['operation_mode']='VEL_TO_ERPM'; ack['accel_to_current_gain']=0.0; ack['accel_to_brake_gain']=0.0
            glob['speed_to_erpm_offset']=0.0
            if 'speed_to_erpm_gain' in candidate_patch:
                glob['speed_to_erpm_gain']=candidate_patch['speed_to_erpm_gain']
            glob['speed_to_erpm_offset']=0.0
            for key in ['slow_start_threshold','slow_start_increment','stop_speed_deadzone','speed_to_braking_max']:
                if key in candidate_patch: ack[key]=candidate_patch[key]
            for key in ['odom_speed_scale','speed_deadband']:
                if key in candidate_patch: odom[key]=candidate_patch[key]
            patch.update({'operation_mode':'VEL_TO_ERPM','candidate_patch':candidate_patch})
        else: raise ConfigTransactionError(f'unknown profile: {name}')
        self._atomic_write(doc)
        record=self._metadata(); record['profiles'].append({'name':name,'applied_utc':datetime.now(timezone.utc).isoformat(),'patch':patch,'source_sha256_after_patch':self._sha(self.config_path)})
        self._write_lock(record); self._save_metadata(record); self._build(f'build_{len(record["profiles"]):02d}_{name}.log')
        return {'active_profile':name,'patch':patch,'source_config':str(self.config_path)}
    def restore(self, *, build: bool=True) -> dict[str,Any] | None:
        if not self.lock.exists(): return None
        try:
            record=self._metadata()
            if not self.backup.is_file(): raise ConfigTransactionError(f'backup missing: {self.backup}')
            shutil.copy2(self.backup,self.config_path)
            if build: self._build('build_restore.log')
            record['state']='restored'; record['restored_utc']=datetime.now(timezone.utc).isoformat(); record['restore_required']=False; record['restored_sha256']=self._sha(self.config_path)
            self._save_metadata(record); self.lock.unlink(missing_ok=True); self.active=False
            return record
        except Exception:
            # Lock intentionally remains as a recovery interlock.
            raise
    @classmethod
    def recover(cls, *, calibration_root: Path, workspace: Path | None, config_relpath: str, build_command: list[str]) -> None:
        root=calibration_root.resolve(); candidates=[]
        if workspace: candidates.append(workspace.expanduser().resolve())
        candidates.extend(root.parents)
        lock=None
        for c in candidates:
            p=c/cls.LOCK_NAME
            if p.is_file(): lock=p; break
        if lock is None: raise ConfigTransactionError('no ERPM calibration recovery lock found')
        data=json.loads(lock.read_text(encoding='utf-8')); source=Path(data['source_config']); backup=Path(data['backup'])
        if not backup.is_file(): raise ConfigTransactionError(f'recovery backup missing: {backup}')
        shutil.copy2(backup,source)
        log=backup.parent/'build_recovery.log'
        with log.open('w',encoding='utf-8') as h:
            result=subprocess.run(build_command,cwd=Path(data['workspace']),stdout=h,stderr=subprocess.STDOUT,text=True,check=False)
        if result.returncode: raise ConfigTransactionError(f'recovery restored source but build failed; inspect {log}')
        lock.unlink(missing_ok=True)
