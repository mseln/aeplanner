#!/usr/bin/env python
import rospy
import math

from sfml import sf
from geometry_msgs.msg import TwistStamped

def publish_cmd_vel(key_map):
    cmd_vel = TwistStamped()

    if(key_map['w']):
        cmd_vel.twist.linear.x += 0.5
    if(key_map['s']):
        cmd_vel.twist.linear.x -= 0.5

    if(key_map['a']):
        cmd_vel.twist.linear.y += 0.5
    if(key_map['d']):
        cmd_vel.twist.linear.y -= 0.5

    if(key_map['z']):
        cmd_vel.twist.linear.z += 0.5
    if(key_map['x']):
        cmd_vel.twist.linear.z -= 0.5

    if(key_map['q']):
        cmd_vel.twist.angular.z += 1
    if(key_map['e']):
        cmd_vel.twist.angular.z -= 1

    pub.publish(cmd_vel)

def key_poller():

    shutdown = False

    key_map = {}
    for c in range(97, 123):
        key_map[chr(c)] = False

    rate = rospy.Rate(10)

    w = sf.RenderWindow(sf.VideoMode(640, 480), "tele-op", sf.Style.TITLEBAR | sf.Style.RESIZE)
    while not rospy.is_shutdown() and not shutdown:
        for event in w.events:
            # Handle shutdown
            if type(event) is sf.KeyEvent and event.code is sf.Keyboard.ESCAPE:
                w.close()
                shutdown = True

            if type(event) is sf.KeyEvent and 0 <= event.code and event.code < 26:
                key = chr(event.code + 97)
                if(event.pressed):
                    key_map[key] = True

                if(event.released):
                    key_map[key] = False

        publish_cmd_vel(key_map)
        sf.sleep(sf.seconds(0.05))




if __name__ == '__main__':
    rospy.init_node("teleop")
    pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=10)
    key_poller()


