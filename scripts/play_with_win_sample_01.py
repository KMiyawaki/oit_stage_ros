#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import random
import actionlib
import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils import make_navigation_goal, navigation, wait_string_message, RosWinMessenger


class PlayWithWinSampleNode(object):
    def __init__(self, topic_name_pairs, topic_name_robot_face_type, move_base_name="move_base"):
        self.topic_name_pairs = topic_name_pairs
        self.move_base_name = move_base_name
        self.pub_robot_face_type = rospy.Publisher(
            topic_name_robot_face_type, String, queue_size=10)

    def show_face(self, result):
        if "win" in result:
            rospy.loginfo("Yeah!!")
            self.pub_robot_face_type.publish("happy")
            return True
        else:
            rospy.loginfo("Ouch!!")
            self.pub_robot_face_type.publish("sad")
            return False

    def play_game_rps(self):
        node_name = rospy.get_name()
        timeout = 30
        # Prepare to play windows game Rock, Paper, Scissors
        topic_name_from_win, topic_name_from_ros = self.topic_name_pairs[0]
        to_win_pub = rospy.Publisher(topic_name_from_ros, String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, topic_name_from_win)
        # Start game sequence
        rospy.loginfo("%s:Try to start game Rock, Paper, Scissors", node_name)
        # Send game start signal to windows game Rock, Paper, Scissors, and wait user's hand type.
        message_from_win = messenger.wait_response(
            "Start your game!", None, timeout)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start windows game Rock, Paper, Scissors", node_name)
            return False
        rospy.sleep(3)
        # Select robot's hand_type
        hand_type = random.choice(["rock", "paper", "scissors"])
        rospy.loginfo("%s:Robot selects '%s'", node_name, hand_type)
        # Send robot's choice to windows game Rock, Paper, Scissors, and wait game result
        message_from_win = messenger.wait_response(
            hand_type, ["win", "lose", "even"], timeout)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr(
                "%s:Timeout. can't get windows game Rock, Paper, Scissors result", node_name)
            return False
        rospy.sleep(1)
        return self.show_face(message_from_win)

    def play_game_B(self):
        node_name = rospy.get_name()
        timeout = 30
        # Prepare to play windows game B (Look that way, Acchimuite Hoi)
        topic_name_from_win, topic_name_from_ros = self.topic_name_pairs[1]
        to_win_pub = rospy.Publisher(topic_name_from_ros, String, queue_size=1)
        messenger = RosWinMessenger(to_win_pub, topic_name_from_win)
        # Start game sequence B
        rospy.loginfo("%s:Try to start game B", node_name)
        # Send game start signal to windows game Rock, Paper, Scissors, and wait user's hand type.
        message_from_win = messenger.wait_response(
            "Start your game!", None, timeout)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr("%s:Timeout. can't start windows game Rock, Paper, Scissors", node_name)
            return False
        rospy.sleep(3)
        # Select face_direction
        face_direction = random.choice(["up", "down", "left", "right"])
        rospy.loginfo("%s:Robot selects '%s'", node_name, face_direction)
        # Send robot's choice to windows game B, and wait game result
        message_from_win = messenger.wait_response(
            face_direction, ["win", "lose", "even"], timeout)
        if message_from_win:
            rospy.loginfo("%s:Receive from win:%s",
                          node_name, message_from_win)
        else:
            rospy.logerr(
                "%s:Timeout. can't get windows game Rock, Paper, Scissors result", node_name)
            return False
        rospy.sleep(1)
        return self.show_face(message_from_win)

    def main(self):
        node_name = rospy.get_name()
        ac = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        # Waiting action server for navigation
        if ac.wait_for_server(rospy.Duration(5)) == False:
            rospy.logerr(
                "%s:Can't connect to move_base action server", node_name)
            return
        rospy.loginfo("%s:The server %s comes up",
                      node_name, self.move_base_name)
        # Show normal face image
        self.pub_robot_face_type.publish("normal")
        rospy.sleep(0.5)
        result_A = self.play_game_rps()  # Play game Rock, Paper, Scissors
        navigation(ac, 1.15, 2.42, math.radians(90))  # Go to next point
        result_B = self.play_game_B()  # Play game B
        # Show game results
        rospy.loginfo("/* GAME RESULTS */")
        rospy.loginfo("/* GAME (Rock, Paper, Scissors):%s */",
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
            ("/from_windows_rps", "/from_ros_rps"),
            ("/from_windows_b", "/from_ros_b")
        ], "/robot_face_type")
    rospy.loginfo("%s:Started", rospy.get_name())

    node.main()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)
