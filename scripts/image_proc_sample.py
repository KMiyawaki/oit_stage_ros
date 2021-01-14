#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageProcSampleNode(object):
    def __init__(self, topic_name_image):
        rospy.Subscriber(topic_name_image, Image,
                         self.image_callback, queue_size=1)
        self.cv_bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "~" + topic_name_image + "_mod", Image, queue_size=1)

    def spin(self):
        pass

    def image_callback(self, recv):
        try:
            rospy.loginfo("%s:Recv image", rospy.get_name())
            cv_image = self.cv_bridge.imgmsg_to_cv2(recv, "bgr8")
            cv_image_result = self.cv_process(cv_image)
            send = self.cv_bridge.cv2_to_imgmsg(cv_image_result, "mono8")
            self.image_pub.publish(send)
        except Exception as e:
            rospy.logerr("%s:%s", rospy.get_name(), str(e))

    def cv_process(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        return cv2.Canny(gray, 10, 10)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # 起動直後は rospy.Time.now() がゼロを返す．

    process_rate = rospy.get_param("~process_rate", 20.0)
    rate = rospy.Rate(process_rate)
    node = ImageProcSampleNode("/image")
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
