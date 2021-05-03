#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import actionlib
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from utils import navigation, wait_string_message


class CommunicationWithWinSampleNode(object):
    def __init__(self, topic_name_from_win, topic_name_from_ros, topic_name_cmd_vel, move_base_name="move_base"):
        self.topic_name_from_win = topic_name_from_win
        self.to_win_pub = rospy.Publisher(
            topic_name_from_ros, String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(
            topic_name_cmd_vel, Twist, queue_size=1)
        self.move_base_name = move_base_name
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def main(self):
        sleep_time = 1
        node_name = rospy.get_name()
        ac = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        # Waiting action server for navigation
        while not ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo(
                "%s:Waiting for the move_base action server to come up", node_name)
        rospy.loginfo("%s:The server %s comes up",
                      node_name, self.move_base_name)
        navigation(ac, 1.15, 2.42, math.radians(90))
        rospy.sleep(5)
        # Send message to windows
        for i in range(0, 10):
            self.to_win_pub.publish("Hello! this is ROS " + str(i))
            rospy.sleep(sleep_time)
        tm = rospy.get_time()
        # Recieve messsage from windows
        for i in range(0, 10):
            message_from_win = wait_string_message(self.topic_name_from_win, 2)
            if message_from_win:
                rospy.loginfo("%s:Receive from win(%d):%s",
                              node_name, i, message_from_win)
            if rospy.get_time() - tm > 20:
                break


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = CommunicationWithWinSampleNode(
        "/from_windows", "/from_ros", "/cmd_vel")
    rospy.loginfo("%s:Started", rospy.get_name())

    node.main()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
