#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from threading import Lock

class ControlFromWinSampleNode(object):
    def __init__(self, topic_name_from_win, topic_name_cmd_vel):
        rospy.Subscriber(topic_name_from_win, String,
                         self.from_win_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(
            topic_name_cmd_vel, Twist, queue_size=1)
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist_lock = Lock()

    def spin(self):
        self.twist_lock.acquire()
        self.cmd_vel_pub.publish(self.twist)
        self.twist_lock.release()

    def from_win_callback(self, recv):
        try:
            self.twist_lock.acquire()
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rospy.loginfo("%s:Recv message '%s'", rospy.get_name(), recv.data)
            if recv.data == "forward":
                self.twist.linear.x = 0.3
            elif recv.data == "back":
                self.twist.linear.x = -0.3
            elif recv.data == "left":
                self.twist.angular.z = math.radians(30)
            elif recv.data == "right":
                self.twist.angular.z = math.radians(-30)
            else:
                rospy.logwarn("%s:Unknown command '%s' stop.", rospy.get_name(), recv.data)
            self.twist_lock.release()
        except Exception as e:
            rospy.logerr("%s:%s", rospy.get_name(), str(e))


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # 起動直後は rospy.Time.now() がゼロを返す．

    process_rate = rospy.get_param("~process_rate", 20.0)
    rate = rospy.Rate(process_rate)
    node = ControlFromWinSampleNode("/from_windows", "/cmd_vel")
    rospy.loginfo("%s:Start with process rate %f Hz",
                  rospy.get_name(), process_rate)

    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
