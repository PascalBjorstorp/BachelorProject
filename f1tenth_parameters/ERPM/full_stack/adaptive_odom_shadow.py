#!/usr/bin/env python3
"""Shadow implementation of the selected causal longitudinal odometry model.

It is used only inside the reversible candidate-verification launch.  It
publishes `/erpm_calibration/candidate_odom`, allowing AckermannToVesc to use
the candidate speed estimate without replacing the production VescToOdom node.
The exact equations and parameter names are the production C++ port contract.
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

EPS=1e-9


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class AdaptiveOdomShadow(Node):
    def __init__(self, patch_path: Path) -> None:
        super().__init__('adaptive_odom_shadow')
        data=yaml.safe_load(patch_path.read_text(encoding='utf-8')) or {}
        self.p=dict(data.get('vesc_to_odom_node',{}).get('ros__parameters',{}))
        self.model=str(self.p.get('longitudinal_speed_model','static_linear'))
        self.wheel_model=str(self.p.get('odom_wheel_model','poly'))
        self.lin=float(self.p.get('odom_erpm_to_speed_linear',0.0))
        self.quad=float(self.p.get('odom_erpm_to_speed_quadratic',0.0))
        self.erpm_knots=list(map(float,self.p.get('odom_erpm_lut_erpm',[])))
        self.speed_knots=list(map(float,self.p.get('odom_erpm_lut_speed_mps',[])))
        self.tau=max(float(self.p.get('odom_accel_filter_tau_s',.05)), 1e-4)
        self.imu_ax_bias=float(self.p.get('odom_imu_ax_bias_mps2',0.0))
        self.limit=max(float(self.p.get('odom_speed_correction_limit_mps',1.5)),0.0)
        self.deadband=max(float(self.p.get('speed_deadband',.05)),0.0)
        self.c=[float(self.p.get(k,0.0)) for k in [
            'odom_correction_accel_drive','odom_correction_accel_brake',
            'odom_correction_current_drive','odom_correction_current_brake',
            'odom_correction_accel_speed_drive','odom_correction_accel_speed_brake']]
        self.w_coast=float(self.p.get('odom_fusion_wheel_weight_coast',.9))
        self.w_high=float(self.p.get('odom_fusion_wheel_weight_high_demand',.2))
        self.a_transition=max(float(self.p.get('odom_fusion_accel_transition_mps2',1.4)),.01)
        self.w_min=float(self.p.get('odom_fusion_min_wheel_weight',.04))
        self.ax=0.0; self.ax_time=None; self.v=0.0; self.x=0.0; self.last_state_t=None
        self.pub=self.create_publisher(Odometry,'/erpm_calibration/candidate_odom',50)
        self.debug={n:self.create_publisher(Float64,f'/erpm_calibration/candidate_odom/{n}',50) for n in ['wheel_static','wheel_corrected','imu_ax_filtered','wheel_weight','speed_estimate']}
        self.create_subscription(Imu,'/sensors/imu/raw',self.on_imu,50)
        self.create_subscription(VescStateStamped,'/sensors/core',self.on_state,50)

    def _pub(self,name:str,value:float)->None:
        msg=Float64(); msg.data=float(value); self.debug[name].publish(msg)

    def on_imu(self,msg:Imu)->None:
        now=stamp_seconds(msg.header.stamp)
        raw=float(msg.linear_acceleration.x) - self.imu_ax_bias
        if self.ax_time is None:
            self.ax=raw
        else:
            dt=max(0.0,min(.1,now-self.ax_time)); alpha=dt/(self.tau+dt)
            self.ax += alpha*(raw-self.ax)
        self.ax_time=now

    def static_speed(self,erpm:float)->float:
        if self.wheel_model=='lut' and len(self.erpm_knots)>=2 and len(self.erpm_knots)==len(self.speed_knots):
            sign=1.0 if erpm>=0 else -1.0; x=abs(erpm); xs=self.erpm_knots; ys=self.speed_knots
            if x<=xs[-1]: val=float(__import__('numpy').interp(x,xs,ys))
            else: val=ys[-1]+(ys[-1]-ys[-2])/max(xs[-1]-xs[-2],EPS)*(x-xs[-1])
            return sign*val
        return self.lin*erpm + self.quad*erpm*abs(erpm)

    def corrected_speed(self,static:float,current:float)->float:
        if self.model not in {'adaptive_wheel','fused_adaptive'}:
            return static
        gate=max(0.0,min(1.0,abs(static)/.20)); ap=max(self.ax,0.0); an=max(-self.ax,0.0); ip=max(current,0.0); ib=max(-current,0.0)
        corr=gate*(self.c[0]*ap+self.c[1]*an+self.c[2]*ip+self.c[3]*ib+self.c[4]*ap*abs(static)+self.c[5]*an*abs(static))
        return static+max(-self.limit,min(self.limit,corr))

    def on_state(self,msg:VescStateStamped)->None:
        t=stamp_seconds(msg.header.stamp)
        if self.last_state_t is None:
            self.last_state_t=t
        dt=max(0.0,min(.1,t-self.last_state_t)); self.last_state_t=t
        static=self.static_speed(float(msg.state.speed)); corrected=self.corrected_speed(static,float(msg.state.current_motor))
        weight=1.0
        if self.model=='fused_adaptive':
            pred=self.v+dt*self.ax
            blend=math.exp(-abs(self.ax)/self.a_transition)
            weight=max(self.w_min,min(1.0,self.w_high+(self.w_coast-self.w_high)*blend))
            self.v=pred+weight*(corrected-pred)
        else:
            self.v=corrected
        if abs(self.v)<self.deadband and abs(self.ax)<.20: self.v=0.0
        self.x += self.v*dt
        out=Odometry(); out.header.stamp=msg.header.stamp; out.header.frame_id='ego_racecar/odom'; out.child_frame_id='ego_racecar/base_link'
        out.pose.pose.position.x=float(self.x); out.pose.pose.orientation=Quaternion(w=1.0); out.twist.twist.linear.x=float(self.v)
        self.pub.publish(out)
        self._pub('wheel_static',static); self._pub('wheel_corrected',corrected); self._pub('imu_ax_filtered',self.ax); self._pub('wheel_weight',weight); self._pub('speed_estimate',self.v)


def main()->int:
    parser=argparse.ArgumentParser(); parser.add_argument('--candidate-patch',type=Path,required=True); args=parser.parse_args()
    rclpy.init(); node=AdaptiveOdomShadow(args.candidate_patch.resolve())
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0

if __name__=='__main__': raise SystemExit(main())
