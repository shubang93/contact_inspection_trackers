#!/usr/bin/python2
# --------------------------------------------------------
# DaSiamRPN
# Licensed under The MIT License
# Written by Qiang Wang (wangqiang2015 at ia.ac.cn)
# --------------------------------------------------------


import argparse, cv2, torch, json
import numpy as np
from os import makedirs
from os.path import realpath, dirname, join, isdir, exists

from SiamRPN_net import SiamRPNotb
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from SiamRPN_utils import rect_2_cxy_wh, cxy_wh_2_rect

import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Float32, Int8


class DaSiamRPN:

    def __init__(self, device):
        self.init_variables_hard()
        self.init_publisher()
        self.init_subscribers()

        self.model = SiamRPNotb()
        self.model.load_state_dict(torch.load(join(realpath(dirname(__file__)), 'SiamRPNOTB.model')))
        self.model.eval().to(device)

    def init_variables_hard(self):

            self.bbox_in_topic = rospy.get_param(
                "~bbox_in_topic", "/mbz2020/perception/roi/rect"
            )
            self.color_image_topic = rospy.get_param(
                "~color_img_topic", "/front_track_camera/fisheye1/camera_raw"
            )
            self.publish_result_img = rospy.get_param(
                "~publish_tracked", "False"
            )
            self.oob_threshold = rospy.get_param("~oob_threshold", 10)
            self.max_bbox_ratio = rospy.get_param("~max_bbox_ratio", 1.0) 
            self._inital_bbox = None
            self._last_bbox = None
            self._is_first_frame = True
            self._bridge = CvBridge()
            self.state = None
            self._current_status =1 
    def init_publisher(self):
            # regressed bbox
            self._pub_bbox = rospy.Publisher(
                "/perception/tracker/bboxOut", BoundingBox2D, queue_size=10
            )
            # Image wiht overlayed bbox
            self._pub_result_img = rospy.Publisher(
                "/perception/tracker/bboxImage", Image, queue_size=10
            )
            # Tracker status
            self._pub_status = rospy.Publisher(
                "/perception/tracker/status", Int8, queue_size=10
            )

    def init_subscribers(self):

        # Input Image frame 
        sub_color = message_filters.Subscriber(self.color_image_topic, Image)

        # Input color frame with callback
        sub_image = rospy.Subscriber(self.color_image_topic, Image, self.track_video)

        # Selected ROI from first frame 
        sub_bbox = rospy.Subscriber(
            self.bbox_in_topic, BoundingBox2D, self.got_bounding_box
        )

    def calculate_bbox_center(self, bbox):
        center = (bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2)
        center = tuple([int(x) for x in center])

        return center

    def got_bounding_box(self, boundingBox):
        self.init_variables_hard() 
        center = (boundingBox.center.x, boundingBox.center.y)
        width = boundingBox.size_x
        height = boundingBox.size_y

        if self._inital_bbox is None:
            rospy.loginfo("BBox Received")
            self._inital_bbox = (
                int(center[0] - width / 2),
                int(center[1] - height / 2),
                width,
                height,
            )

    def track_video(self, rgb_msg):
        color_image = self._bridge.imgmsg_to_cv2(rgb_msg)
            
        if self._is_first_frame and self._inital_bbox is not None:  # init
            print('init')
            target_pos, target_sz = rect_2_cxy_wh(self._inital_bbox)
            self.state = SiamRPN_init(color_image, target_pos, target_sz, self.model, device)  # init tracker
            location = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])
            self._last_bbox = self._inital_bbox
            self._is_first_frame = False
            
        elif not self._is_first_frame:  # tracking
            self.state = SiamRPN_track(self.state, color_image, device)  # track
            location = cxy_wh_2_rect(self.state['target_pos']+1, self.state['target_sz'])
            self._last_bbox= rect_2_cxy_wh(location)
            
        if self._last_bbox is not None:

            bbox_message = BoundingBox2D()
            bbox_message.size_x = self._last_bbox[1][0]
            bbox_message.size_y = self._last_bbox[1][1]
            bbox_message.center.theta = 0
            bbox_message.center.x = self._last_bbox[0][0] + self._last_bbox[1][0]/2
            bbox_message.center.y = self._last_bbox[0][1] + self._last_bbox[1][1]/2
            self._pub_bbox.publish(bbox_message)
            
            status_message = Int8()
            status_message.data = self._current_status
            self._pub_status.publish(status_message)

      
            location = [int(l) for l in location]  #
            cv2.rectangle(color_image, (location[0], location[1]),
                                (location[0] + location[2], location[1] + location[3]), (0, 255, 255), 3)
            cv2.putText(color_image, str("DaSiamRPN"), (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            imgmsg = self._bridge.cv2_to_imgmsg(color_image, encoding="mono8")
            self._pub_result_img.publish(imgmsg)


    def spin(self):
            rate = rospy.Rate(30)

            while not rospy.is_shutdown():
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node("goturn_tracker")
    rospy.loginfo("Starting DaSiamRPN tracker")

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    instance = DaSiamRPN(device)
    instance.spin()


