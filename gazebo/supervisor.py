#!/usr/bin/env python3
"""Start/stop the complete rover stack with free mouse navigation."""
import argparse,signal,subprocess,time,sys,os,json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from gz.transport import Node
from gz.msgs.world_control_pb2 import WorldControl
from gz.msgs.boolean_pb2 import Boolean
ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--headless',action='store_true');parser.add_argument('--duration',type=float,default=0);parser.add_argument('--gui-config',type=Path,default=ROOT/'gazebo/gui.config');args=parser.parse_args()
    simbin=Path(get_package_prefix('gz_sim_vendor'))/'opt/gz_sim_vendor/libexec/gz/sim10'
    ros=Path(get_package_prefix('ros_gz_bridge'))
    statefile=ROOT/'gazebo/state_publisher.yaml';statefile.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'use_sim_time':True,'robot_description':(ROOT/'gazebo/leo_lidar.urdf').read_text()}}}))
    transport=Node()
    children={};running=True;code=0;start=time.monotonic()
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def spawn(name,cmd):
        # Separate process groups prevent terminal SIGINT reaching a child twice.
        children[name]=subprocess.Popen(cmd,start_new_session=True)
        print(f'[my_rover] {name} PID {children[name].pid}',flush=True)
    def terminate(name,timeout=8):
        p=children.get(name)
        if p and p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        if p:print(f'[my_rover] {name} stopped ({p.returncode})',flush=True)
    try:
        spawn('server',[str(simbin/'gz-sim-main'),'-s','-r','--headless-rendering','--physics-engine','gz-physics-dartsim-plugin',str(ROOT/'gazebo/mdrs.sdf')])
        spawn('bridge',[str(ros/'lib/ros_gz_bridge/parameter_bridge'),'--ros-args','-r','__node:=rover_bridge','-p',f'config_file:={ROOT/"gazebo/bridge.yaml"}'])
        spawn('state_publisher',[str(ros/'lib/robot_state_publisher/robot_state_publisher'),'--ros-args','--params-file',str(statefile)])
        spawn('runtime',[sys.executable,str(ROOT/'gazebo/runtime.py')])
        if not args.headless:
            spawn('gui',[str(simbin/'gz-sim-gui-client'),'--render-engine-gui','ogre2','--gui-config',str(args.gui_config),str(ROOT/'gazebo/mdrs.sdf')])
        print('[my_rover] Leo + 2km MDRS: /cmd_vel, 3D LiDAR /lidar/points, RealSense D435 stereo /camera/*, odometry, TF. Ctrl+C stops everything.',flush=True)
        while running:
            now=time.monotonic()
            if args.duration and now-start>=args.duration:break
            for name,p in children.items():
                if p.poll() is not None:
                    code=0 if name=='gui' and p.returncode==0 else 1
                    print(f'[my_rover] {name} exited ({p.returncode})',flush=True);running=False;break
            time.sleep(.05)
    finally:
        # Quiesce both bridge directions before destroying ROS/Gazebo transport.
        # The installed ros_gz_bridge can crash if Gazebo disappears mid-callback.
        terminate('runtime')
        server=children.get('server')
        if server and server.poll() is None:
            try:
                req=WorldControl();req.pause=True
                transport.request('/world/mdrs/control',req,WorldControl,Boolean,1500)
                time.sleep(.3)
            except Exception as e:print('[my_rover] pause on shutdown:',e,flush=True)
        terminate('bridge');terminate('state_publisher');terminate('gui');terminate('server')
        (ROOT/'logs/gazebo_shutdown.json').write_text(json.dumps({name:p.returncode for name,p in children.items()},indent=2))
    return code
if __name__=='__main__':sys.exit(main())
