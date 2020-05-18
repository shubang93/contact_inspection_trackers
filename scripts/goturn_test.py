#!/usr/bin/python2
import os
import time
import argparse
import re
import torch
import numpy as np
import cv2
import sys
import rospy
import numpy as np
import message_filters
import argparse
from model import GoNet
from helper import NormalizeToTensor, Rescale, crop_sample, bgr2rgb
from boundingbox import BoundingBox
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Float32, Int8



class GOTURN:
    """Tester for OTB formatted sequences"""
    def __init__(self, model_path, device):
        # model_path = os.path.join(model_path, "/checkpoints/pytroch_goturn.pth.tar")
        if (not os.path.isfile(model_path)) or (not os.path.exists(model_path   )):
            fileName = input("Whhoops! No such file! Please enter the name of the file you'd like to use.")
        # Ros params
        self.init_variables_hard()
        self.init_publisher()
        self.init_subscribers()

        # GOTURN Model params
        self.device = device
        self.transform = NormalizeToTensor()
        self.scale = Rescale((224, 224))
        self.model_path = model_path
        self.model = GoNet()
        self.opts = None
        self.prev_img = None 
        self.curr_img = None
        checkpoint = torch.load(
            model_path, map_location=lambda storage, loc: storage)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.to(device)



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

            self._bridge = CvBridge()

            self._current_color_msg = None
            self._inital_bbox = None
            self._last_bbox = None
            self._is_first_frame = True
            self._has_scale_changed = False

            self._color_timestamp = -1
            self._original_distance = -1
            self._current_distance = -1
            self._previous_distance = -1

            self._scale = 1.0
            self._fallback_scale = 0.4
            self._current_status = 1


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
        sub_image = rospy.Subscriber(self.color_image_topic, Image, self.got_image)

        # Selected ROI from first frame 
        sub_bbox = rospy.Subscriber(
            self.bbox_in_topic, BoundingBox2D, self.got_bounding_box
        )

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
    def calculate_bbox_center(self, bbox):
        center = (bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2)
        center = tuple([int(x) for x in center])

        return center
        
    def got_image(self, rgb_msg):
        color_image = self._bridge.imgmsg_to_cv2(rgb_msg)

        if self._is_first_frame and self._inital_bbox is not None:
            rospy.loginfo("Initializing tracker")
            self._last_bbox = self._inital_bbox
            self._is_first_frame = False
            self.prev_img = color_image
        
        elif not self._is_first_frame:
            self.model.eval()
            self.curr_img = color_image
            sample = self.transform(self._get_sample())            
            
            self._last_bbox = self.get_rect(sample)
            self.prev_img = self.curr_img
        
        if self._last_bbox is not None:


            bbox_message = BoundingBox2D()
            bbox_message.size_x = self._last_bbox[2]
            bbox_message.size_y = self._last_bbox[3]
            bbox_message.center.theta = 0
            bbox_message.center.x = self._last_bbox[0] + self._last_bbox[2]/2
            bbox_message.center.y = self._last_bbox[1] + self._last_bbox[3]/2

            self._pub_bbox.publish(bbox_message)
            
            status_message = Int8()
            status_message.data = self._current_status
            self._pub_status.publish(status_message)

            if self.publish_result_img:
                self._last_bbox = tuple([int(i) for i in self._last_bbox])

                if self._current_status == 1:
                    cv2.rectangle(color_image, (self._last_bbox[0], self._last_bbox[1]), (self._last_bbox[0]+self._last_bbox[2], self._last_bbox[1]+self._last_bbox[3]), (0, 0, 255), 2)
                else:
                    cv2.rectangle(color_image,(self._last_bbox[0], self._last_bbox[1]), (self._last_bbox[0]+self._last_bbox[2], self._last_bbox[1]+self._last_bbox[3]), (255, 0, 0), 2)


                imgmsg = self._bridge.cv2_to_imgmsg(
                    color_image, encoding="mono8"
                )
  
                self._pub_result_img.publish(imgmsg)



    def _get_sample(self):
        """
        Returns cropped previous and current frame at the previous predicted
        location. Note that the images are scaled to (224,224,3).
        """
        prev = self.prev_img
        curr = self.curr_img
        prevbb = self._last_bbox
        prev_sample, opts_prev = crop_sample({'image': prev, 'bb': prevbb})
        curr_sample, opts_curr = crop_sample({'image': curr, 'bb': prevbb})
        prev_img = bgr2rgb(self.scale(prev_sample, opts_prev)['image'])
        curr_img = bgr2rgb(self.scale(curr_sample, opts_curr)['image'])
        sample = {'previmg': prev_img, 'currimg': curr_img}
        self.curr_img = curr
        self.opts = opts_curr
        return sample

    def get_rect(self, sample):
        """
        Regresses the bounding box coordinates in the original image dimensions
        for an input sample.
        """
        x1, x2 = sample['previmg'], sample['currimg']
        x1 = x1.unsqueeze(0).to(self.device)
        x2 = x2.unsqueeze(0).to(self.device)
        y = self.model(x1, x2)
        bb = y.data.cpu().numpy().transpose((1, 0))
        bb = bb[:, 0]
        bbox = BoundingBox(bb[0], bb[1], bb[2], bb[3])

        # inplace conversion
        bbox.unscale(self.opts['search_region'])
        bbox.uncenter(self.curr_img, self.opts['search_location'],
                      self.opts['edge_spacing_x'], self.opts['edge_spacing_y'])
        return bbox.get_bb_list()
    
    def spin(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("goturn_tracker")
    rospy.loginfo("Starting GOTURN tracker")

    cuda = torch.cuda.is_available()
    device = torch.device('cuda:0' if cuda else 'cpu')
    myargv = rospy.myargv(argv=sys.argv)

    instance = GOTURN(myargv[1], device)
    # rospy.loginfo("Running csrt tracker in debug mode: {}".format(debug))

    instance.spin()
