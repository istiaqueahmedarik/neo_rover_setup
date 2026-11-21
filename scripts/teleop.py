#!/usr/bin/env python3
import select,sys,termios,tty,time
import rclpy
from geometry_msgs.msg import Twist
rclpy.init();node=rclpy.create_node('rover_keyboard');pub=node.create_publisher(Twist,'/cmd_vel',10)
settings=termios.tcgetattr(sys.stdin)
print('W/S: forward/reverse   A/D: turn   Space: stop   Q: quit\nHold a key to keep moving (stops after 0.4 seconds without input).')
v=w=0.;last=0
try:
    tty.setcbreak(sys.stdin.fileno())
    while rclpy.ok():
        if select.select([sys.stdin],[],[],.05)[0]:
            key=sys.stdin.read(1).lower();last=time.monotonic()
            if key in ('q','\x03'):break
            v,w={'w':(.5,0.),'s':(-.5,0.),'a':(0.,.6),'d':(0.,-.6)}.get(key,(0.,0.))
        if time.monotonic()-last>.4:v=w=0.
        msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg);rclpy.spin_once(node,timeout_sec=0)
finally:
    for _ in range(3):pub.publish(Twist());time.sleep(.03)
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings);node.destroy_node();rclpy.shutdown()
