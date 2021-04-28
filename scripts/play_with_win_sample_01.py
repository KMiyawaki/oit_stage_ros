#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import actionlib
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion


class PlayWithWinSampleNode(object):
    def __init__(self, topic_name_pairs, move_base_name="move_base"):
        self.topic_name_pairs = topic_name_pairs
        self.move_base_name = move_base_name

    def recv_message(self, topic_name, timeout):
        try:
            return rospy.wait_for_message(topic_name, String, timeout)
        except Exception as e:
            rospy.logdebug(str(e))
        return None

    def make_navigation_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        return goal

    def navigation(self, action_server, x, y, theta):
        node_name = rospy.get_name()
        rospy.loginfo("%s:Sending goal", node_name)
        action_server.send_goal(self.make_navigation_goal(x, y, theta))
        finished = action_server.wait_for_result(rospy.Duration(30))
        state = action_server.get_state()
        if finished:
            rospy.loginfo("%s:Finished: (%d)", node_name, state)
        else:
            rospy.loginfo("%s:Time out: (%d)", node_name, state)

    def play_game_A(self, frame_interval):
        node_name = rospy.get_name()
        message_wait_limit = 30
        # Prepare to play windows game A (Rock, Paper, Scissors)
        topic_name_from_win, topic_name_from_ros = self.topic_name_pairs[0]
        to_win_pub = rospy.Publisher(topic_name_from_ros, String, queue_size=1)
        # Start game sequence A
        rospy.loginfo("%s:Try to start game A", node_name)
        tm = rospy.get_time()
        message_from_win = ""
        while rospy.get_time() - tm < message_wait_limit and not message_from_win:
            # Send game start signal to windows game A
            to_win_pub.publish("Start your game!")
            rospy.sleep(frame_interval)
            # Recieve messsage from windows
            recv = self.recv_message(topic_name_from_win, 2)
            if recv:
                rospy.loginfo("%s:Receive from win:%s",
                              node_name, recv.data)
                message_from_win = recv.data
        rospy.sleep(3)
        # Select robot's hand_type
        hand_type = random.choice(["Rock", "Paper", "Scissors"])
        # Reset timer and clear windows message
        tm = rospy.get_time()
        message_from_win = ""
        while rospy.get_time() - tm < message_wait_limit and not message_from_win:
            # Send robot hand_type to windows game A
            rospy.loginfo("%s:Robot select '%s'",
                          node_name, hand_type)
            to_win_pub.publish(hand_type)
            rospy.sleep(frame_interval)
            # Recieve messsage from windows
            recv = self.recv_message(topic_name_from_win, 2)
            if recv:
                rospy.loginfo("%s:Receive from win:%s",
                              node_name, recv.data)
                message_from_win = recv.data
        rospy.sleep(3)
        if "win" in message_from_win:
            rospy.loginfo("Yeah!!")
            return True
        else:
            rospy.loginfo("Ouch!!")
            return False

    def play_game_B(self, frame_interval):
        node_name = rospy.get_name()
        message_wait_limit = 30
        # Prepare to play windows game B (Look that way, Acchimuite Hoi)
        topic_name_from_win, topic_name_from_ros = self.topic_name_pairs[1]
        to_win_pub = rospy.Publisher(topic_name_from_ros, String, queue_size=1)
        # Start game sequence B
        rospy.loginfo("%s:Try to start game B", node_name)
        tm = rospy.get_time()
        message_from_win = ""
        while rospy.get_time() - tm < message_wait_limit and not message_from_win:
            # Send game start signal to windows game B
            to_win_pub.publish("Start your game!")
            rospy.sleep(frame_interval)
            # Recieve messsage from windows
            recv = self.recv_message(topic_name_from_win, 2)
            if recv:
                rospy.loginfo("%s:Receive from win:%s",
                              node_name, recv.data)
                message_from_win = recv.data
        rospy.sleep(3)
        # Select face_direction
        face_direction = random.choice(["up", "down", "left", "right"])
        # Reset timer and clear windows message
        tm = rospy.get_time()
        message_from_win = ""
        while rospy.get_time() - tm < message_wait_limit and not message_from_win:
            # Send robot hand_type to windows game B
            rospy.loginfo("%s:Robot select '%s'",
                          node_name, face_direction)
            to_win_pub.publish(face_direction)
            rospy.sleep(frame_interval)
            # Recieve messsage from windows
            recv = self.recv_message(topic_name_from_win, 2)
            if recv:
                rospy.loginfo("%s:Receive from win:%s",
                              node_name, recv.data)
                message_from_win = recv.data
        rospy.sleep(3)
        if "win" in message_from_win:
            rospy.loginfo("Yeah!!")
            return True
        else:
            rospy.loginfo("Ouch!!")
            return False

    def main(self):
        frame_interval = 1
        node_name = rospy.get_name()
        ac = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        # Waiting action server for navigation
        while not ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo(
                "%s:Waiting for the move_base action server to come up", node_name)
        rospy.loginfo("%s:The server %s comes up",
                      node_name, self.move_base_name)
        rospy.sleep(5)
        # Play game A
        result_A = self.play_game_A(frame_interval)
        # Go to next point
        self.navigation(ac, 1.15, 2.42, math.radians(90))
        # Play game A
        result_B = self.play_game_B(frame_interval)
        # Show game results
        rospy.loginfo("/* GAME RESULTS */")
        rospy.loginfo("/* GAME (A):%s */",
                      'Robot win' if result_A else 'Robot lose')
        rospy.loginfo("/* GAME (B):%s */",
                      'Robot win' if result_B else 'Robot lose')
        rospy.loginfo("/* ------------ */")


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = PlayWithWinSampleNode(
        [
            ("/from_windows_a", "/from_ros_a"),
            ("/from_windows_b", "/from_ros_b")
        ])
    rospy.loginfo("%s:Started", rospy.get_name())

    node.main()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
