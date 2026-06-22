#!/usr/bin/env python3
"""Export scalar diagnostic fields from an ERPM calibration MCAP bag to Parquet.

All raw topics—including LaserScan arrays, TF, diagnostics and parameter events—
remain in MCAP. Scalar Parquet files are analysis conveniences only.
"""
from __future__ import annotations
import argparse,json,math
from pathlib import Path

def header_ns(message):
    h=getattr(message,'header',None); s=getattr(h,'stamp',None)
    return None if s is None else int(s.sec)*1_000_000_000+int(s.nanosec)
def parse(value:str)->dict:
    try:
        x=json.loads(value); return x if isinstance(x,dict) else {'value':x}
    except Exception: return {'raw':value,'parse_error':True}
def main()->int:
    p=argparse.ArgumentParser(); p.add_argument('bag',type=Path); a=p.parse_args(); bag=a.bag.resolve();
    if not (bag/'metadata.yaml').is_file(): raise SystemExit(f'not bag: {bag}')
    try:
        import pandas as pd, rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc: raise SystemExit('source ROS workspace and install pandas/pyarrow: '+repr(exc))
    derived=bag.parent/'derived'; derived.mkdir(exist_ok=True)
    names=['events','imu','vesc','odom','drive','ackermann','motor_command','motor_raw_speed','motor_raw_current','motor_raw_brake','motor_selected_speed','motor_selected_current','motor_selected_brake','motor_selector_status','candidate_odom','candidate_odom_debug','candidate_accel_debug','scan_index','tf_index','parameter_event_index','topic_index']
    rows={n:[] for n in names}; reader=rosbag2_py.SequentialReader(); reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}; classes={}
    while reader.has_next():
        topic,raw,bag_ns=reader.read_next(); t=types.get(topic)
        if not t: continue
        try: msg=deserialize_message(raw,classes.setdefault(t,get_message(t)))
        except Exception as exc: rows['topic_index'].append({'topic':topic,'bag_ns':int(bag_ns),'type':t,'decode_error':repr(exc)}); continue
        common={'topic':topic,'bag_ns':int(bag_ns),'header_ns':header_ns(msg),'type':t}; rows['topic_index'].append({**common,'decode_error':None})
        if topic=='/sensors/imu/raw': rows['imu'].append({**common,'ax':msg.linear_acceleration.x,'ay':msg.linear_acceleration.y,'az':msg.linear_acceleration.z,'gx':msg.angular_velocity.x,'gy':msg.angular_velocity.y,'gz':msg.angular_velocity.z})
        elif topic in {'/ego_racecar/odom','/erpm_calibration/candidate_odom'}:
            tw=msg.twist.twist; po=msg.pose.pose; key='candidate_odom' if topic=='/erpm_calibration/candidate_odom' else 'odom'; rows[key].append({**common,'x':po.position.x,'y':po.position.y,'vx':tw.linear.x,'vy':tw.linear.y,'vz':tw.linear.z,'wz':tw.angular.z})
        elif topic=='/sensors/core':
            st=msg.state; rows['vesc'].append({**common,'erpm':getattr(st,'speed',math.nan),'motor_current':getattr(st,'current_motor',math.nan),'input_current':getattr(st,'current_input',math.nan),'battery_voltage':getattr(st,'voltage_input',math.nan),'temp_motor':getattr(st,'temp_motor',math.nan),'temp_fet':getattr(st,'temp_fet',math.nan),'duty_cycle':getattr(st,'duty_cycle',math.nan),'fault_code':getattr(st,'fault_code',math.nan)})
        elif topic in {'/drive','/ackermann_cmd'}:
            key='drive' if topic=='/drive' else 'ackermann'; rows[key].append({**common,'speed':msg.drive.speed,'acceleration':msg.drive.acceleration,'steering_angle':msg.drive.steering_angle,'steering_angle_velocity':msg.drive.steering_angle_velocity})
        elif topic in {'/commands/motor/speed','/commands/motor/current','/commands/motor/brake'}: rows['motor_command'].append({**common,'command':topic.rsplit('/',1)[-1],'value':msg.data})
        elif topic=='/erpm_calibration/motor_raw_speed': rows['motor_raw_speed'].append({**common,'value':msg.data})
        elif topic=='/erpm_calibration/motor_raw_current': rows['motor_raw_current'].append({**common,'value':msg.data})
        elif topic=='/erpm_calibration/motor_raw_brake': rows['motor_raw_brake'].append({**common,'value':msg.data})
        elif topic=='/erpm_calibration/motor_selected_speed': rows['motor_selected_speed'].append({**common,'value':msg.data})
        elif topic=='/erpm_calibration/motor_selected_current': rows['motor_selected_current'].append({**common,'value':msg.data})
        elif topic=='/erpm_calibration/motor_selected_brake': rows['motor_selected_brake'].append({**common,'value':msg.data})
        elif topic.startswith('/erpm_calibration/candidate_odom/'):
            rows['candidate_odom_debug'].append({**common,'signal':topic.rsplit('/',1)[-1],'value':msg.data})
        elif topic.startswith('/erpm_calibration/candidate_accel/'):
            rows['candidate_accel_debug'].append({**common,'signal':topic.rsplit('/',1)[-1],'value':msg.data})
        elif topic=='/erpm_calibration/motor_selector_status': rows['motor_selector_status'].append({**common,**parse(msg.data)})
        elif topic=='/scan': rows['scan_index'].append({**common,'frame_id':msg.header.frame_id,'range_count':len(msg.ranges),'finite_range_count':sum(math.isfinite(x) for x in msg.ranges),'angle_min':msg.angle_min,'angle_max':msg.angle_max,'angle_increment':msg.angle_increment,'time_increment':msg.time_increment,'scan_time':msg.scan_time,'range_min':msg.range_min,'range_max':msg.range_max})
        elif topic in {'/tf','/tf_static'}: rows['tf_index'].append({**common,'transform_count':len(msg.transforms)})
        elif topic=='/parameter_events': rows['parameter_event_index'].append({**common,'node':getattr(msg,'node',''),'new_parameter_count':len(getattr(msg,'new_parameters',[])),'changed_parameter_count':len(getattr(msg,'changed_parameters',[])),'deleted_parameter_count':len(getattr(msg,'deleted_parameters',[]))})
        elif topic=='/erpm_calibration/event': rows['events'].append({**common,**parse(msg.data)})
    for n,data in rows.items():
        if data: pd.DataFrame(data).to_parquet(derived/f'{n}.parquet',index=False)
    counts={}
    for row in rows['topic_index']:
        if not row.get('decode_error'): counts[row['topic']]=counts.get(row['topic'],0)+1
    summary={'bag_dir':str(bag),'topic_types':types,'row_counts':{k:len(v) for k,v in rows.items()},'decoded_message_counts_by_topic':dict(sorted(counts.items())),'raw_scan_arrays_preserved_in_mcap':True,'all_unexported_topics_remain_in_mcap':True}
    (derived/'export_summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True,default=str)+'\n'); print(json.dumps(summary,indent=2,sort_keys=True,default=str)); return 0
if __name__=='__main__': raise SystemExit(main())
