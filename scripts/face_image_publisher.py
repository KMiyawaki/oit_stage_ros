#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import cv2
import rospkg
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from threading import Lock


class FaceImagePublisherNode(object):
    def __init__(self, topic_name_robot_face_type, topic_name_face_image, robot_face_images, initial_face_type="normal"):
        rospy.Subscriber(topic_name_robot_face_type, String,
                         self.face_type_callback, queue_size=1)
        self.face_image_pub = rospy.Publisher(
            topic_name_face_image, Image, queue_size=1)
        self.robot_face_images = robot_face_images
        self.current_face = initial_face_type
        self.lock = Lock()

    def spin(self):
        node_name = rospy.get_name()
        robot_face_type = "N/A"
        self.lock.acquire()
        robot_face_type = self.current_face
        self.lock.release()
        if not robot_face_type in self.robot_face_images:
            rospy.logerr("%s:Can't find face type '%s'", node_name, robot_face_type)
        elif self.robot_face_images[robot_face_type] == None:
            rospy.logerr("%s:face type '%s' is not loaded",
                         node_name, robot_face_type)
        else:
            self.face_image_pub.publish(self.robot_face_images[robot_face_type])

    def face_type_callback(self, recv):
        try:
            rospy.loginfo("%s:Recv message '%s'", rospy.get_name(), recv.data)
            self.lock.acquire()
            self.current_face = recv.data
            self.lock.release()
        except Exception as e:
            rospy.logerr("%s:%s", rospy.get_name(), str(e))


def load_face_images():
    node_name = rospy.get_name()
    face_image_dir = rospkg.RosPack().get_path('oit_stage_ros') + "/images/faces"
    robot_face_images = {"happy": None, "normal": None, "sad": None}
    bridge = CvBridge()
    for f in robot_face_images:
        try:
            filepath = face_image_dir + "/" + f + ".png"
            rospy.loginfo("%s:Loading %s", node_name, filepath)
            im = cv2.imread(filepath, cv2.IMREAD_COLOR)
            robot_face_images[f] = bridge.cv2_to_imgmsg(im, encoding="bgr8")
        except Exception as e:
            rospy.logerr(str(e))
    return robot_face_images


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    process_rate = rospy.get_param("~process_rate", 10.0)
    rate = rospy.Rate(process_rate)
    node = FaceImagePublisherNode(
        "/robot_face_type", "/robot_face_image", load_face_images())
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
