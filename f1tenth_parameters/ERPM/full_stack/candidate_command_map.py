#!/usr/bin/env python3
"""Candidate zero-intercept desired-speed -> ERPM command adapter for shadow VEL tests."""
from __future__ import annotations
import argparse
from pathlib import Path
import rclpy, yaml
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

class CandidateCommandMap(Node):
    def __init__(self, patch_path:Path)->None:
        super().__init__('candidate_command_map')
        data=yaml.safe_load(patch_path.read_text(encoding='utf-8')) or {}
        p=dict(data.get('ackermann_to_vesc_node',{}).get('ros__parameters',{}))
        self.kind=str(p.get('speed_command_model','linear')); self.g=float(p.get('speed_to_erpm_gain',0)); self.q=float(p.get('speed_to_erpm_quadratic',0)); self.x=list(map(float,p.get('speed_command_lut_speed_mps',[]))); self.y=list(map(float,p.get('speed_command_lut_erpm',[])))
        self.pub=self.create_publisher(Float64,'/erpm_calibration/motor_from_ackermann/speed',100)
        self.create_subscription(AckermannDriveStamped,'/ackermann_cmd',self.cb,100)
    def cb(self,msg:AckermannDriveStamped)->None:
        v=float(msg.drive.speed)
        if self.kind=='lut' and len(self.x)>=2 and len(self.x)==len(self.y):
            import numpy as np
            sign=1 if v>=0 else -1; av=abs(v); val=float(np.interp(av,self.x,self.y))
            if av>self.x[-1]: val=self.y[-1]+(self.y[-1]-self.y[-2])/max(self.x[-1]-self.x[-2],1e-9)*(av-self.x[-1])
            erpm=sign*val
        else:
            erpm=self.g*v+self.q*v*abs(v)
        # Exact zero speed means exact zero ERPM: no static intercept.
        out=Float64(); out.data=float(erpm); self.pub.publish(out)
def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('--candidate-patch',type=Path,required=True); a=p.parse_args()
    rclpy.init(); n=CandidateCommandMap(a.candidate_patch.resolve())
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0
if __name__=='__main__':raise SystemExit(main())
