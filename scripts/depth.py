#!/usr/bin/python

import cv2
import sys
from cv_bridge import CvBridge
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped


class DepthEstimation(object):
    def __init__(self, debug=False):

        self.depth_image_topic = rospy.get_param(
            "~depth_img_topic", "/d435i/depth/image_rect_raw"
        )

        self.min_depth = rospy.get_param("~min_depth", 0.1)
        self.max_depth = rospy.get_param("~max_depth", 20.0)
        self.depth_scale_factor = rospy.get_param("~depth_scale_factor", 1000.0)
        self.roi_width = rospy.get_param("~roi_width", 50)

        self._bridge = CvBridge()

        self._pub_median_depth = rospy.Publisher(
            "/depth_estimation/point_cloud/pose", Vector3Stamped, queue_size=10
        )

        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.got_image)

    # calculate median depth in 100x100 roi around image center
    def got_image(self, depth_msg):
        depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        split_array = np.array([320, 320 + self.roi_width])
        split_array = [0 if x < 0 else x for x in split_array]
        bbox_content = np.hsplit(depth_image[240 : 240 + self.roi_width], split_array,)[1]

        mask = (
            (bbox_content >= (self.min_depth * self.depth_scale_factor))
            & (bbox_content <= (self.max_depth * self.depth_scale_factor))
            & (bbox_content != 9.0)
        )

        median = np.nanmedian(bbox_content[mask])

        msg = Vector3Stamped()
        msg.header = depth_msg.header
        msg.vector.z = median / self.depth_scale_factor
        self._pub_median_depth.publish(msg)


if __name__ == "__main__":
    rospy.init_node("median_depth_estimation")
    rospy.loginfo("Starting MedianDepthEstimation...")

    node = DepthEstimation()

    rospy.spin()
