#!/usr/bin/python

import cv2
import sys
from cv_bridge import CvBridge
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Float32, Int8

"""
This ROS Node implements a object tracker that used a depth image
to resize it's generated bounding box.

It takes the following parameters:
    ~color_img_topic:
        The name of the topic this node should subscribe to for receiving the color img
    ~depth_img_topic:
        The name of the topic this node should subscribe to for receiving the depth img
    ~publish_tracked:
        Either True or False. If True, an Image where the BBox is lying over the RGB image is being published
    ~publish_median_depth:
        Either True or False. IF True, the median depth will be published.
    ~oob_threshold:
        A integer value that specifies the distance in pixels that the bounding box has to be away from any image
        border to be invalid
    ~min_depth:
        The minimum depth the drone "can see"
    ~max_depth:
        The maximum depth the drone "can see"

It subscribes to the following topics:
    /mbz2020/scaletracker/bboxIn (vision_msgs/BoundingBox2D):
        The inital bounding box of to object to track.
        Only the first message on this topic will be used.

    There are two more subscribers that depend on the topics ~color_img_topic and ~depth_img_topic

It publishes on the following topics:
    /mbz2020/scaletracker/bboxOut (vision_msgs/BoundingBox2D):
        The current (scaled) bounding box of the object that's being tracked.
    /mbz2020/scaletracker/bboxImage (sensor_msgs/Image)
        The current RGB Image with overlayed BBox (only if enabled)
    /mbz2020/scaletracker/medianDepth (Float32)
        The current median depth value.
    /mbz2020/scaletracker/status (Int8)
        The status of the current bounding box (1 = Valid, 0 = Invalid)

"""


class scaling_tracker(object):

    #
    # Initilization Functions
    #

    def __init__(self, debug=False):
        self.init_variables(debug)
        self.init_subscribers()
        self.init_publisher()

    def init_variables(self, debug):
        if not debug:
            self.bbox_in_topic = rospy.get_param(
                "~bbox_in_topic", "/mbz2020/perception/roi/rect"
            )
            self.color_image_topic = rospy.get_param(
                "~color_img_topic", "/camera/color/image_raw"
            )
            self.depth_image_topic = rospy.get_param(
                "~depth_img_topic", "/camera/depth/image_rect_raw"
            )
            self.publish_result_img = rospy.get_param(
                "~publish_tracked", "False"
            )
            self.publish_median_depth = rospy.get_param(
                "~publish_median_depth", "False"
            )

            self.oob_threshold = rospy.get_param("~oob_threshold", 10)
            self.min_depth = rospy.get_param("~min_depth", 0.1)
            self.max_depth = rospy.get_param("~max_depth", 20.0)
            self.depth_scale_factor = rospy.get_param("~depth_scale_factor", 1000.0)
            self.max_bbox_ratio = rospy.get_param("~max_bbox_ratio", 1.0)
        else:
            self.color_image_topic = "/front_depth_camera/color/image_raw"
            self.depth_image_topic = "/front_depth_camera/aligned_depth_to_color/image_raw"
            self.bbox_in_topic = "/mbz2020/perception/roi/rect"
            self.publish_result_img = True
            self.publish_median_depth = True
            self.oob_threshold = 10
            self.max_depth = 5.0
            self.min_depth = 0.5
            self.depth_scale_factor = 1.0
            self.max_bbox_ratio = 1.0

        self._bridge = CvBridge()

        self._color_timestamp = -1
        self._depth_timestamp = -1

        self._current_depth_msg = None
        self._current_color_msg = None

        self._inital_bbox = None
        self._current_bbox = None

        self._original_distance = -1
        self._current_distance = -1
        self._previous_distance = -1

        self._tracker = cv2.TrackerCSRT_create()

        self._is_first_frame = True

        self._has_scale_changed = False
        self._scale = 1.0
        self._fallback_scale = 0.4
        # self._max_ratio = 1.0

        self._last_bbox = None

        self._current_status = 1

    def init_subscribers(self):
        sub_color = message_filters.Subscriber(self.color_image_topic, Image)
        sub_depth = message_filters.Subscriber(self.depth_image_topic, Image)

        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], 10, 1
        )
        sync.registerCallback(self.got_image)

        sub_bbox = rospy.Subscriber(
            self.bbox_in_topic, BoundingBox2D, self.got_bounding_box
        )

    def init_publisher(self):
        self._pub_bbox = rospy.Publisher(
            "/mbz2020/scaletracker/bboxOut", BoundingBox2D, queue_size=10
        )

        self._pub_result_img = rospy.Publisher(
            "/mbz2020/scaletracker/bboxImage", Image, queue_size=10
        )

        self._pub_median_depth = rospy.Publisher(
            "/mbz2020/scaletracker/bboxMedianDepth", Float32, queue_size=10
        )

        self._pub_status = rospy.Publisher(
            "/mbz2020/scaletracker/status", Int8, queue_size=10
        )

    #
    # Helper Functions
    #

    def check_point_oob(self, point, image, threshold):
        if point[0] < threshold or point[0] > image.shape[1] - threshold:
            return True

        if point[1] < threshold or point[1] > image.shape[0] - threshold:
            return True

        return False

    def calculate_bbox_center(self, bbox):
        center = (bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2)
        center = tuple([int(x) for x in center])

        return center

    def scale_bbox(self, bbox, factor):
        width, height = bbox[2], bbox[3]

        width = width * factor
        height = height * factor

        center = self.calculate_bbox_center(bbox)

        new_bbox = (
            center[0] - width / 2,
            center[1] - height / 2,
            width,
            height,
        )

        return tuple([int(x) for x in new_bbox])

    def get_bbox_scale(self, inital_bbox, new_bbox):
        i_width, i_height = inital_bbox[2], inital_bbox[3]
        n_width, n_height = new_bbox[2], new_bbox[3]

        width_scale = n_width / i_width
        height_scale = n_height / i_height

        return (width_scale + height_scale) / 2

    def spin(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

    #
    # Callback functions for ROS Subscriptions
    #

    def got_bounding_box(self, boundingBox):
        self.init_variables(False)

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

    def got_image(self, rgb_msg, depth_msg):
        depth_image = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="32FC1"
        )

        color_image = self._bridge.imgmsg_to_cv2(rgb_msg)

        final_bbox = None

        if self._is_first_frame and self._inital_bbox is not None:
            rospy.loginfo("Initializing tracker")
            current_bbox = self._inital_bbox
            bbox_center = self.calculate_bbox_center(current_bbox)

            if depth_image[bbox_center[1]][bbox_center[0]] > 0:
                if self._original_distance == -1:
                    self._original_distance = depth_image[bbox_center[1]][
                        bbox_center[0]
                    ]

                current_distance = depth_image[bbox_center[1]][bbox_center[0]]

            self._tracker.init(color_image, current_bbox)
            self._is_first_frame = False

            final_bbox = current_bbox

        elif not self._is_first_frame:
            width = int(color_image.shape[1] * self._scale)
            height = int(color_image.shape[0] * self._scale)

            color_image = cv2.resize(color_image, (width, height))
            depth_image = cv2.resize(depth_image, (width, height))

            if self._has_scale_changed:
                self._has_scale_changed = False

                self._tracker = cv2.TrackerCSRT_create()

                if self._scale == self._fallback_scale:
                    bbox_scaled = tuple(
                        [self._scale * i for i in self._last_bbox]
                    )

                    self._inital_bbox = self.scale_bbox(
                        self._inital_bbox, self._scale)

                    self._tracker.init(color_image, bbox_scaled)
                    self.tracker_suggested_bbox = bbox_scaled
                else:
                    bbox_scaled = tuple(
                        [
                            self._scale / self._fallback_scale * i
                            for i in self._last_bbox
                        ]
                    )

                    self._inital_bbox = self.scale_bbox(
                        self._inital_bbox, self._scale / self._fallback_scale)

                    self._tracker.init(color_image, bbox_scaled)
                    self.tracker_suggested_bbox = bbox_scaled

                ok = True

            else:
                ok, self.tracker_suggested_bbox = self._tracker.update(
                    color_image
                )

            if ok:
                bbox_center = self.calculate_bbox_center(
                    self.tracker_suggested_bbox
                )

                split_array = np.array(
                    [
                        int(self.tracker_suggested_bbox[0]),
                        int(
                            self.tracker_suggested_bbox[0]
                            + self.tracker_suggested_bbox[2]
                        ),
                    ]
                )
                split_array = [0 if x < 0 else x for x in split_array]
                bbox_content = np.hsplit(
                    depth_image[
                        int(self.tracker_suggested_bbox[1]): int(
                            self.tracker_suggested_bbox[1]
                            + self.tracker_suggested_bbox[3]
                        )
                    ],
                    split_array,
                )[1]

                mask = (bbox_content >= (self.min_depth * self.depth_scale_factor)) & (
                        bbox_content <= (self.max_depth * self.depth_scale_factor)) & (
                        bbox_content != 9.0)

                median = np.nanmedian(bbox_content[mask])
                if self.publish_median_depth:
                    bbox_depth_msg = Float32()
                    bbox_depth_msg.data = median / self.depth_scale_factor
                    self._pub_median_depth.publish(bbox_depth_msg)

                if median > 0:
                    if self._original_distance == -1:
                        self._original_distance = median

                    current_distance = median

                    depth_scale = self._original_distance / current_distance
                    tracker_scale = self.get_bbox_scale(
                        self._inital_bbox, self.tracker_suggested_bbox
                    )

                    scaled_tracker_bbox = self.scale_bbox(
                        self.tracker_suggested_bbox, depth_scale / tracker_scale
                    )

                    scaled_tracker_bbox = tuple(
                        [int(x) for x in scaled_tracker_bbox]
                    )

                    final_bbox = scaled_tracker_bbox

                elif np.isnan(median):
                    self._current_status = 0

                self.tracker_suggested_bbox = [
                    int(x) for x in self.tracker_suggested_bbox
                ]
                self.tracker_suggested_bbox = (
                    self.tracker_suggested_bbox[0],
                    self.tracker_suggested_bbox[1],
                    self.tracker_suggested_bbox[2],
                    self.tracker_suggested_bbox[3],
                )

                if final_bbox is None:
                    final_bbox = self.tracker_suggested_bbox

        if final_bbox is not None:
            self._last_bbox = final_bbox

            width_ratio = float(final_bbox[2]) / float(color_image.shape[1])
            height_ratio = float(final_bbox[3]) / float(color_image.shape[0])

            if (
                width_ratio > self.max_bbox_ratio or height_ratio > self.max_bbox_ratio
            ) and self._scale != self._fallback_scale:
                rospy.loginfo("Scaling down...")

                self._scale = self._fallback_scale
                self._has_scale_changed = True
            elif (
                width_ratio < self.max_bbox_ratio and height_ratio < self.max_bbox_ratio
            ) and self._scale == self._fallback_scale:
                rospy.loginfo("Scaling back up...")

                self._scale = 1.0
                self._has_scale_changed = True

            center = self.calculate_bbox_center(final_bbox)

            if self.check_point_oob(center, color_image, oob_threshold):
                self._current_status = 0

            bbox_message = BoundingBox2D()

            bbox_message.size_x = final_bbox[2]
            bbox_message.size_y = final_bbox[3]

            bbox_message.center.theta = 0
            bbox_message.center.x = final_bbox[0] + final_bbox[2] / 2
            bbox_message.center.y = final_bbox[1] + final_bbox[3] / 2

            self._pub_bbox.publish(bbox_message)

            status_message = Int8()
            status_message.data = self._current_status
            self._pub_status.publish(status_message)

            if self.publish_result_img:
                final_bbox = tuple([int(i) for i in final_bbox])

                if self._current_status == 1:
                    cv2.rectangle(color_image, final_bbox, (0, 0, 255), 2)
                else:
                    cv2.rectangle(color_image, final_bbox, (255, 0, 0), 2)

                cv2.circle(color_image, center, 3, (255, 0, 0), 2)
                imgmsg = self._bridge.cv2_to_imgmsg(
                    color_image, encoding="rgb8"
                )

                self._pub_result_img.publish(imgmsg)


if __name__ == "__main__":
    rospy.init_node("tracker_scale")
    rospy.loginfo("Starting scale tracker...")

    # When started from roslaunch, argc is > 1
    if len(sys.argv) > 1:
        debug = False
    else:  # When started from VSCode, argc == 1
        debug = True

    st = scaling_tracker(debug=debug)
    rospy.loginfo("Running scale tracker in debug mode: {}".format(debug))

    st.spin()