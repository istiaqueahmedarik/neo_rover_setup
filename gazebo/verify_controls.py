#!/usr/bin/env python3
"""Inject Qt input into the real GUI, bypassing Wayland's external-input restrictions."""
import os,time,subprocess,signal,json,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from gz.transport import Node
from gz.msgs.pose_pb2 import Pose
from gz.msgs.int32_pb2 import Int32
R=Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID']='78';os.environ['GZ_PARTITION']='my_rover_controls'
config=R/'logs/controls_gui.config';config.write_text((R/'gazebo/gui.config').read_text()+'''\n<plugin filename="InputProbe" name="Input probe"><gz-gui><property type="string" key="state">floating</property><property type="double" key="width">5</property><property type="double" key="height">5</property><property type="bool" key="showTitleBar">false</property></gz-gui></plugin>''')
log=(R/'logs/controls_simulator.log').open('w');proc=subprocess.Popen([str(R/'start.sh'),'--gui-config',str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=rclpy.create_node('controls_verification');latest={};phase={'value':0};snapshots={}
n.create_subscription(Odometry,'/simulation/ground_truth',lambda m:latest.update(truth=m),qos_profile_sensor_data)
gz=Node()
def camera(m):latest['camera']=(np.array([m.position.x,m.position.y,m.position.z]),np.array([m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z]))
gz.subscribe(Pose,'/gui/camera/pose',camera);gz.subscribe(Int32,'/gui/input_probe',lambda m:phase.update(value=m.data))
try:
 end=time.monotonic()+70
 while time.monotonic()<end:
  if proc.poll() is not None:raise RuntimeError((R/'logs/controls_simulator.log').read_text()[-2500:])
  rclpy.spin_once(n,timeout_sec=.005)
  k=phase['value']
  if k>=10 and k not in snapshots and 'camera' in latest and 'truth' in latest:
   t=latest['truth'].pose.pose;p=t.position;q=t.orientation
   snapshots[k]={'position':np.array([p.x,p.y,p.z]),'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'camera':latest['camera']}
  if 20 in snapshots:break
 assert all(k in snapshots for k in [10,11,12,13,15,16,17,18,19,20]),list(snapshots)
 S=snapshots;angle=lambda a,b:2*math.acos(float(np.clip(abs(np.dot(a,b)),-1,1)))
 report={'input_method':'Qt key/focus/mouse events injected into the actual Gazebo QQuickWindow; ordinary GUI handlers, ROS bridge and physics run unchanged','w_forward_displacement_m':float(np.dot((S[11]['position']-S[10]['position'])[:2],[math.cos(S[10]['yaw']),math.sin(S[10]['yaw'])])),'release_drift_m':float(np.linalg.norm(S[12]['position']-S[11]['position'])),'a_turn_yaw_rad':math.atan2(math.sin(S[13]['yaw']-S[12]['yaw']),math.cos(S[13]['yaw']-S[12]['yaw'])),'focus_out_drift_m':float(np.linalg.norm(S[16]['position']-S[15]['position'])),'mouse_orbit_angle_rad':angle(S[16]['camera'][1],S[17]['camera'][1]),'orbit_snapback_angle_rad':angle(S[17]['camera'][1],S[18]['camera'][1]),'mouse_pan_displacement_m':float(np.linalg.norm(S[19]['camera'][0]-S[18]['camera'][0])),'right_drag_zoom_displacement_m':float(np.linalg.norm(S[20]['camera'][0]-S[19]['camera'][0]))}
 (R/'logs/controls_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 assert report['w_forward_displacement_m']>.4 and report['a_turn_yaw_rad']>.6,report
 assert report['release_drift_m']<.1 and report['focus_out_drift_m']<.1,report
 assert report['mouse_orbit_angle_rad']>.05 and report['orbit_snapback_angle_rad']<.01,report
 assert report['mouse_pan_displacement_m']>.05 and report['right_drag_zoom_displacement_m']>.05,report
 print('PASS: GUI W/A, key release, focus-out stop, mouse orbit/pan/zoom, no camera snapback')
finally:
 n.destroy_node();rclpy.shutdown()
 if proc.poll() is None:
  proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=25)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()
