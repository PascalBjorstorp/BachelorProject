"""ROS runtime primitives for longitudinal ERPM/current calibration.

Runtime checks use VESC ERPM and existing odometry only to schedule captures and
protect the car.  All parameter fits use raw LiDAR scan-matched longitudinal
velocity offline; odometry never serves as its own calibration reference.
"""
from __future__ import annotations
import math, time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, String
from .events import EventPublisher
from .straight_assist import from_config as _straight_assist_from_config

@dataclass
class Latest:
    imu_ax: float=math.nan; imu_ay: float=math.nan; imu_gz: float=math.nan
    odom_vx: float=math.nan; odom_vy: float=math.nan; odom_yaw: float=math.nan
    candidate_odom_vx: float=math.nan; candidate_odom_vy: float=math.nan
    erpm: float=math.nan; motor_current_a: float=math.nan; input_current_a: float=math.nan
    battery_v: float=math.nan; motor_temp_c: float=math.nan; fet_temp_c: float=math.nan
    selected_speed_erpm: float=math.nan; selected_current_a: float=math.nan; selected_brake_a: float=math.nan
    scan_count: int=0; seen:set[str]=field(default_factory=set)

class CalibrationNode(Node):
    def __init__(self, name: str, config: dict[str,Any]) -> None:
        super().__init__(name); self.cfg=config; self.latest=Latest(); self._samples={}; self._window=False
        self.straight_assist=_straight_assist_from_config(config); self._sa_last_t=None
        self.event=EventPublisher(self,config['session']['event_topic'])
        self.drive_pub=self.create_publisher(AckermannDriveStamped,'/drive',50)
        self.raw_speed_pub=self.create_publisher(Float64,'/erpm_calibration/motor_raw_speed',100)
        self.raw_current_pub=self.create_publisher(Float64,'/erpm_calibration/motor_raw_current',100)
        self.raw_brake_pub=self.create_publisher(Float64,'/erpm_calibration/motor_raw_brake',100)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub=self.create_publisher(String,'/erpm_calibration/motor_mode',qos)
        self.create_subscription(Imu,'/sensors/imu/raw',self._imu,400)
        self.create_subscription(Odometry,'/ego_racecar/odom',self._odom,400)
        self.create_subscription(Odometry,'/erpm_calibration/candidate_odom',self._candidate_odom,400)
        self.create_subscription(LaserScan,'/scan',self._scan,100)
        self.create_subscription(Float64,'/erpm_calibration/motor_selected_speed',self._selected_speed,200)
        self.create_subscription(Float64,'/erpm_calibration/motor_selected_current',self._selected_current,200)
        self.create_subscription(Float64,'/erpm_calibration/motor_selected_brake',self._selected_brake,200)
        try:
            from vesc_msgs.msg import VescStateStamped
            self.create_subscription(VescStateStamped,'/sensors/core',self._vesc,400)
        except Exception as exc: self.get_logger().error(f'Cannot import VescStateStamped: {exc}')

    def _rec(self, **values:float) -> None:
        if not self._window: return
        for k,v in values.items():
            if k in self._samples: self._samples[k].append(float(v))
    def _imu(self,msg:Imu)->None:
        self.latest.imu_ax=float(msg.linear_acceleration.x); self.latest.imu_ay=float(msg.linear_acceleration.y); self.latest.imu_gz=float(msg.angular_velocity.z); self.latest.seen.add('imu')
        self._rec(imu_ax=self.latest.imu_ax,imu_ay=self.latest.imu_ay,imu_gz=self.latest.imu_gz)
    def _odom(self,msg:Odometry)->None:
        self.latest.odom_vx=float(msg.twist.twist.linear.x); self.latest.odom_vy=float(msg.twist.twist.linear.y)
        q=msg.pose.pose.orientation; self.latest.odom_yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
        self.latest.seen.add('odom')
        self._rec(odom_vx=self.latest.odom_vx,odom_vy=self.latest.odom_vy,odom_yaw=self.latest.odom_yaw)

    def _candidate_odom(self,msg:Odometry)->None:
        self.latest.candidate_odom_vx=float(msg.twist.twist.linear.x); self.latest.candidate_odom_vy=float(msg.twist.twist.linear.y); self.latest.seen.add('candidate_odom')
        self._rec(candidate_odom_vx=self.latest.candidate_odom_vx,candidate_odom_vy=self.latest.candidate_odom_vy)
    def _scan(self,msg:LaserScan)->None:
        self.latest.scan_count+=1; self.latest.seen.add('scan')
    def _vesc(self,msg:Any)->None:
        st=msg.state; self.latest.erpm=float(getattr(st,'speed',math.nan)); self.latest.motor_current_a=float(getattr(st,'current_motor',math.nan)); self.latest.input_current_a=float(getattr(st,'current_input',math.nan)); self.latest.battery_v=float(getattr(st,'voltage_input',math.nan)); self.latest.motor_temp_c=float(getattr(st,'temp_motor',math.nan)); self.latest.fet_temp_c=float(getattr(st,'temp_fet',math.nan)); self.latest.seen.add('vesc')
        self._rec(erpm=self.latest.erpm,motor_current_a=self.latest.motor_current_a,input_current_a=self.latest.input_current_a,battery_v=self.latest.battery_v,motor_temp_c=self.latest.motor_temp_c,fet_temp_c=self.latest.fet_temp_c)
    def _selected_speed(self,msg:Float64)->None:
        self.latest.selected_speed_erpm=float(msg.data); self.latest.seen.add('selected_speed'); self._rec(selected_speed_erpm=float(msg.data))
    def _selected_current(self,msg:Float64)->None:
        self.latest.selected_current_a=float(msg.data); self.latest.seen.add('selected_current'); self._rec(selected_current_a=float(msg.data))
    def _selected_brake(self,msg:Float64)->None:
        self.latest.selected_brake_a=float(msg.data); self.latest.seen.add('selected_brake'); self._rec(selected_brake_a=float(msg.data))

    def spin(self, duration_s:float)->None:
        end=time.monotonic()+max(0.0,duration_s)
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=0.0); time.sleep(0.0005)
    def wait_for(self, required:set[str], timeout_s:float=15.0, primer:Callable[[],None]|None=None)->None:
        start=time.monotonic()
        while time.monotonic()-start<timeout_s:
            # The selector's echo streams (selected_*) only appear once it has
            # received a mode, so an optional primer re-publishes that mode each
            # iteration; the selector then mirrors it back and the chain is seen.
            if primer is not None: primer()
            self.spin(0.05)
            if required.issubset(self.latest.seen): return
        raise RuntimeError('missing required streams: '+', '.join(sorted(required-self.latest.seen)))
    def begin_window(self,*fields:str)->None: self._samples={f:[] for f in fields}; self._window=True
    def end_window(self)->dict[str,float]:
        self._window=False; out={}
        for key,vals in self._samples.items():
            a=np.asarray(vals,dtype=float); out[f'{key}_mean']=float(np.nanmean(a)) if a.size else math.nan; out[f'{key}_std']=float(np.nanstd(a)) if a.size else math.nan; out[f'{key}_count']=int(a.size)
        return out
    def _mode(self, mode:str)->None:
        if mode not in {'ackermann','raw_erpm','raw_current','raw_brake','neutral'}: raise ValueError(f'bad motor mode: {mode}')
        msg=String(); msg.data=mode; self.mode_pub.publish(msg); self.event.emit('motor_selector_mode',mode=mode)
    @staticmethod
    def _float(v:float)->Float64:
        m=Float64(); m.data=float(v); return m
    def reset_straight_assist(self)->None:
        self.straight_assist.reset(); self._sa_last_t=None
    def _straight_trim(self)->float:
        now=time.monotonic(); dt=0.0 if self._sa_last_t is None else now-self._sa_last_t; self._sa_last_t=now
        trim=self.straight_assist.step(heading=self.latest.odom_yaw,speed=self.latest.odom_vx,dt=dt)
        self._rec(straight_assist_trim_rad=trim); return trim
    def _drive_message(self, speed_mps:float, acceleration_mps2:float, steering_angle_rad:float|None=None, *, straight_assist_allowed:bool=True)->None:
        base=float(self.cfg['session']['steering_angle_rad'] if steering_angle_rad is None else steering_angle_rad)
        # Closed-loop straight-assist: when the commanded steering is straight,
        # add a bounded trim that nulls measured yaw/lateral drift so the car
        # actually holds a line (overcoming steering slack/centre offset). For an
        # intentional non-zero angle (e.g. cornering arcs), pass it through and
        # keep the assist neutral so it cannot fight the commanded turn.
        if straight_assist_allowed and self.straight_assist.enabled and abs(base)<=1e-3:
            steer=base+self._straight_trim()
        else:
            self.reset_straight_assist(); steer=base
        msg=AckermannDriveStamped(); msg.header.stamp=self.get_clock().now().to_msg(); msg.drive.speed=float(speed_mps); msg.drive.acceleration=float(acceleration_mps2); msg.drive.steering_angle=float(steer); self.drive_pub.publish(msg)
    def ackermann(self, speed_mps:float, acceleration_mps2:float=0.0, steering_angle_rad:float|None=None, *, straight_assist_allowed:bool=True)->None:
        self._mode('ackermann'); self._drive_message(speed_mps,acceleration_mps2,steering_angle_rad,straight_assist_allowed=straight_assist_allowed)
    def _steering_keepalive(self, *, straight_assist_allowed:bool=True)->None:
        # AckermannToVesc must receive a normal zero-speed / zero-angle command
        # so it continues to publish the installed steering map. Its motor output
        # is remapped to selector input and ignored whenever a raw motor source is
        # active, so this cannot contend with the calibration command.
        self._drive_message(0.0,0.0,straight_assist_allowed=straight_assist_allowed)
    def raw_erpm(self, erpm:float)->None:
        self._mode('raw_erpm'); self._steering_keepalive(); self.raw_speed_pub.publish(self._float(erpm))
    def raw_current(self, current_a: float) -> None:
        if not math.isfinite(current_a) or current_a < -1e-9:
            raise RuntimeError(f'raw drive-current request {current_a!r} is invalid')
        self._mode('raw_current'); self._steering_keepalive(); self.raw_current_pub.publish(self._float(current_a))

    def raw_brake(self, brake_a: float) -> None:
        if not math.isfinite(brake_a) or brake_a < -1e-9:
            raise RuntimeError(f'raw brake-current request {brake_a!r} is invalid')
        self._mode('raw_brake'); self._steering_keepalive(); self.raw_brake_pub.publish(self._float(brake_a))
    def neutral(self)->None:
        self._mode('neutral'); self._steering_keepalive(straight_assist_allowed=False); self.spin(0.04); self._mode('neutral')
    def fail_stop(self)->None:
        try:
            cfg=self.cfg.get('motion_startup',{})
            brake_a=max(0.0,float(cfg.get('failed_attempt_brake_current_a',0.0)))
            brake_s=max(0.0,float(cfg.get('failed_attempt_brake_s',0.0)))
            settle_s=max(0.0,float(cfg.get('failed_attempt_settle_s',0.0)))
            hz=float(self.cfg['session']['command_publish_hz']); period=1.0/hz
            self.reset_straight_assist()
            if brake_a>0.0 and brake_s>0.0:
                self.event.emit('failed_attempt_stop_begin',brake_current_a=brake_a,duration_s=brake_s)
                self._mode('raw_brake')
                end=time.monotonic()+brake_s
                while time.monotonic()<end:
                    self._steering_keepalive(straight_assist_allowed=False)
                    self.raw_brake_pub.publish(self._float(brake_a))
                    self.spin(period)
                self.event.emit('failed_attempt_stop_end')
            self.neutral()
            if settle_s>0.0:
                self.spin(settle_s)
        except Exception as exc:
            self.get_logger().error(f'failed to publish failed-attempt stop: {exc}')
    def command(self, kind:str, target:float, *, speed_hint:float=0.0, steering_angle_rad:float|None=None)->None:
        if kind=='raw_erpm': self.raw_erpm(target)
        elif kind=='raw_current': self.raw_current(target)
        elif kind=='raw_brake': self.raw_brake(target)
        elif kind=='ackermann_speed': self.ackermann(target,0.0,steering_angle_rad)
        elif kind=='ackermann_accel': self.ackermann(speed_hint,target,steering_angle_rad)
        elif kind=='neutral': self.neutral()
        else: raise ValueError(kind)
    def _safety(self, motion:bool)->None:
        s=self.cfg['session']
        if math.isfinite(self.latest.battery_v) and self.latest.battery_v<float(s['battery_min_v']): raise RuntimeError(f'battery below safety threshold: {self.latest.battery_v:.2f} V')
        if math.isfinite(self.latest.motor_temp_c) and self.latest.motor_temp_c>float(s['max_motor_temp_c']): raise RuntimeError(f'motor thermal threshold exceeded: {self.latest.motor_temp_c:.1f} C')
        if math.isfinite(self.latest.fet_temp_c) and self.latest.fet_temp_c>float(s['max_fet_temp_c']): raise RuntimeError(f'FET thermal threshold exceeded: {self.latest.fet_temp_c:.1f} C')
        if motion and math.isfinite(self.latest.odom_vx) and abs(self.latest.odom_vx)>float(s['hard_speed_limit_mps']): raise RuntimeError(f'odom exceeded hard speed limit: {self.latest.odom_vx:.3f} m/s')
    def hold(self, *, kind:str,target:float,duration_s:float,phase:str,segment_id:str,trial_id:str|None,capture:bool=True,speed_hint:float=0.0,steering_angle_rad:float|None=None,window_fields:tuple[str,...]=(), **meta:Any)->dict[str,float]:
        """Hold one nominal command and emit a self-describing capture window.

        The steering angle is recorded on both phase boundaries.  This matters for
        candidate cross-axis validation: the offline verifier must key its grid on
        the commanded condition rather than infer steering from the noisy command
        stream after the fact.
        """
        hz=float(self.cfg['session']['command_publish_hz']); period=1.0/hz
        active_steering=float(self.cfg['session']['steering_angle_rad'] if steering_angle_rad is None else steering_angle_rad)
        self.event.emit('phase_start',phase=phase,segment_id=segment_id,trial_id=trial_id,capture=capture,command_kind=kind,command_target=float(target),speed_hint_mps=float(speed_hint),steering_angle_rad=active_steering,**meta)
        if window_fields: self.begin_window(*window_fields)
        end=time.monotonic()+duration_s
        try:
            while time.monotonic()<end:
                self.command(kind,target,speed_hint=speed_hint,steering_angle_rad=active_steering); self.spin(period); self._safety(motion=kind!='neutral')
        except BaseException:
            self.fail_stop()
            raise
        result=self.end_window() if window_fields else {}
        self.event.emit('phase_end',phase=phase,segment_id=segment_id,trial_id=trial_id,capture=capture,command_kind=kind,command_target=float(target),speed_hint_mps=float(speed_hint),steering_angle_rad=active_steering,**result,**meta)
        return result
    def establish_raw_erpm(self, *, target_erpm:float, segment_id:str,trial_id:str, startup_speed_mps:float|None=None)->dict[str,Any]:
        cfg=self.cfg['motion_startup']; hz=float(self.cfg['session']['command_publish_hz']); dt=1.0/hz
        start=time.monotonic(); minimum=float(cfg['minimum_startup_s']); deadline=start+float(cfg['stability_timeout_s']); hist=deque(); exclusion=False
        initial_scan_count = self.latest.scan_count; self.reset_straight_assist()
        use_ack_startup = (
            str(cfg.get('raw_erpm_startup_mode', 'raw_erpm')).lower() == 'ackermann_speed'
            and startup_speed_mps is not None
            and math.isfinite(float(startup_speed_mps))
        )
        startup_speed = float(startup_speed_mps) if use_ack_startup else math.nan
        startup_mode = 'ackermann_speed_then_raw_erpm' if use_ack_startup else 'raw_erpm'
        self.event.emit('motion_startup_begin',segment_id=segment_id,trial_id=trial_id,mode=startup_mode,target_erpm=float(target_erpm),startup_speed_mps=startup_speed,minimum_startup_s=minimum)
        try:
            while time.monotonic()<deadline:
                if use_ack_startup:
                    self.ackermann(startup_speed,0.0,straight_assist_allowed=False)
                else:
                    self.raw_erpm(target_erpm)
                self.spin(dt); self._safety(motion=(abs(startup_speed)>1e-3 if use_ack_startup else abs(target_erpm)>1))
                now=time.monotonic()
                if now-start>=minimum and not exclusion:
                    self.event.emit('motion_startup_excluded_end',segment_id=segment_id,trial_id=trial_id); exclusion=True
                if now-start<minimum or not (math.isfinite(self.latest.odom_vx) and math.isfinite(self.latest.imu_ax)): continue
                if not use_ack_startup and not math.isfinite(self.latest.erpm): continue
                hist.append((now,self.latest.erpm,self.latest.odom_vx,self.latest.imu_ax))
                while hist and now-hist[0][0]>float(cfg['stability_window_s']): hist.popleft()
                # Evaluate once a full window of post-startup time has elapsed, over the
                # most recent <=window_s samples. The old `now-hist[0][0]<window_s` check
                # almost never fired (the popleft trim above already drops anything older
                # than window_s), so the gate timed out even on a clean steady pass.
                if now-start-minimum<float(cfg['stability_window_s']) or len(hist)<3: continue
                e=np.asarray([x[1] for x in hist]); v=np.asarray([x[2] for x in hist]); a=np.asarray([x[3] for x in hist]);
                err=max(float(cfg['raw_erpm_absolute_error']),float(cfg['raw_erpm_relative_error_fraction'])*abs(target_erpm))
                # A real scan must arrive after the command. This is an online observability
                # guard only; final velocity is still calculated from the recorded raw scans.
                scan_observed = self.latest.scan_count > initial_scan_count
                if use_ack_startup:
                    stable=(scan_observed and abs(float(np.median(v))-startup_speed)<=float(cfg.get('max_startup_speed_error_mps',0.20)) and float(np.std(v))<=float(cfg['max_odom_speed_std_mps']) and abs(float(np.median(a)))<=float(cfg['max_abs_imu_ax_mps2']))
                else:
                    stable=(scan_observed and abs(float(np.median(e))-target_erpm)<=err and float(np.std(e))<=float(cfg['max_erpm_std']) and float(np.std(v))<=float(cfg['max_odom_speed_std_mps']) and abs(float(np.median(a)))<=float(cfg['max_abs_imu_ax_mps2']))
                if stable:
                    settle_s=max(0.0,float(cfg.get('raw_erpm_switch_settle_s',0.0))) if use_ack_startup else 0.0
                    if settle_s>0.0:
                        settle_end=time.monotonic()+settle_s
                        while time.monotonic()<settle_end:
                            self.raw_erpm(target_erpm); self.spin(dt); self._safety(motion=abs(target_erpm)>1)
                    out={'stable':True,'elapsed_s':time.monotonic()-start,'startup_mode':startup_mode,'startup_speed_mps':startup_speed,'erpm_median':float(np.nanmedian(e)),'erpm_std':float(np.nanstd(e)),'odom_vx_median':float(np.median(v)),'odom_vx_std':float(np.std(v)),'imu_ax_median':float(np.median(a)),'scan_observed_after_command':scan_observed,'samples':len(hist),'raw_erpm_switch_settle_s':settle_s}; self.event.emit('motion_stable',segment_id=segment_id,trial_id=trial_id,**out); return out
        except BaseException:
            self.fail_stop()
            raise
        e=np.asarray([x[1] for x in hist],dtype=float); v=np.asarray([x[2] for x in hist],dtype=float); a=np.asarray([x[3] for x in hist],dtype=float)
        scan_observed=self.latest.scan_count>initial_scan_count
        out={'stable':False,'elapsed_s':time.monotonic()-start,'startup_mode':startup_mode,'startup_speed_mps':startup_speed,'samples':len(hist),'scan_observed_after_command':scan_observed,'erpm_median':float(np.nanmedian(e)) if e.size else math.nan,'erpm_std':float(np.nanstd(e)) if e.size else math.nan,'odom_vx_median':float(np.nanmedian(v)) if v.size else math.nan,'odom_vx_std':float(np.nanstd(v)) if v.size else math.nan,'imu_ax_median':float(np.nanmedian(a)) if a.size else math.nan}
        if use_ack_startup and v.size:
            out['startup_speed_error_mps']=abs(float(np.nanmedian(v))-startup_speed)
        self.event.emit('motion_stability_timeout',segment_id=segment_id,trial_id=trial_id,**out); self.fail_stop(); return out
    def establish_ackermann_speed(self, *, speed_mps:float,segment_id:str,trial_id:str,steering_angle_rad:float|None=None)->dict[str,Any]:
        # Uses odom/IMU only for an operational startup gate. Offline ground-speed
        # calibration uses scan-matched LiDAR velocity, never this odometry.
        cfg=self.cfg['motion_startup']; hz=float(self.cfg['session']['command_publish_hz']); dt=1.0/hz; start=time.monotonic(); minimum=float(cfg['minimum_startup_s']); deadline=start+float(cfg['stability_timeout_s']); hist=deque(); exclusion=False
        initial_scan_count = self.latest.scan_count; self.reset_straight_assist()
        self.event.emit('motion_startup_begin',segment_id=segment_id,trial_id=trial_id,mode='ackermann_speed',target_speed_mps=float(speed_mps),minimum_startup_s=minimum)
        try:
            while time.monotonic()<deadline:
                self.ackermann(speed_mps,0.0,steering_angle_rad,straight_assist_allowed=False); self.spin(dt); self._safety(motion=abs(speed_mps)>1e-3); now=time.monotonic()
                if now-start>=minimum and not exclusion: self.event.emit('motion_startup_excluded_end',segment_id=segment_id,trial_id=trial_id); exclusion=True
                if now-start<minimum or not (math.isfinite(self.latest.odom_vx) and math.isfinite(self.latest.imu_ax)): continue
                hist.append((now,self.latest.odom_vx,self.latest.imu_ax))
                while hist and now-hist[0][0]>float(cfg['stability_window_s']): hist.popleft()
                # Evaluate once a full window of post-startup time has elapsed, over the
                # most recent <=window_s samples. The old `now-hist[0][0]<window_s` check
                # almost never fired (the popleft trim above already drops anything older
                # than window_s), so the gate timed out even on a clean steady pass.
                if now-start-minimum<float(cfg['stability_window_s']) or len(hist)<3: continue
                v=np.asarray([x[1] for x in hist]); a=np.asarray([x[2] for x in hist]); scan_observed=self.latest.scan_count>initial_scan_count; stable=scan_observed and float(np.std(v))<=float(cfg['max_odom_speed_std_mps']) and abs(float(np.median(a)))<=float(cfg['max_abs_imu_ax_mps2'])
                if stable:
                    out={'stable':True,'elapsed_s':now-start,'odom_vx_median':float(np.median(v)),'odom_vx_std':float(np.std(v)),'imu_ax_median':float(np.median(a)),'scan_observed_after_command':scan_observed,'samples':len(hist)}; self.event.emit('motion_stable',segment_id=segment_id,trial_id=trial_id,**out); return out
        except BaseException:
            self.fail_stop()
            raise
        v=np.asarray([x[1] for x in hist],dtype=float); a=np.asarray([x[2] for x in hist],dtype=float); scan_observed=self.latest.scan_count>initial_scan_count
        out={'stable':False,'elapsed_s':time.monotonic()-start,'samples':len(hist),'scan_observed_after_command':scan_observed,'odom_vx_median':float(np.nanmedian(v)) if v.size else math.nan,'odom_vx_std':float(np.nanstd(v)) if v.size else math.nan,'imu_ax_median':float(np.nanmedian(a)) if a.size else math.nan}
        self.event.emit('motion_stability_timeout',segment_id=segment_id,trial_id=trial_id,**out); self.fail_stop(); return out

def start_node(name:str,cfg:dict[str,Any],required:set[str])->CalibrationNode:
    rclpy.init(args=None); node=CalibrationNode(name,cfg)
    # Prime the selector with the neutral mode while waiting so its selected_*
    # echo streams (which are part of `required`) can actually appear.
    try: node.wait_for(required, primer=lambda: node._mode('neutral')); node.neutral(); node.spin(0.15); return node
    except Exception: node.destroy_node(); rclpy.shutdown(); raise

def finish_node(node:CalibrationNode)->None:
    try: node.neutral(); time.sleep(0.15)
    finally: node.destroy_node(); rclpy.shutdown()
