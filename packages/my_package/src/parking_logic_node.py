#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
from my_package.src.utils import LineDetector
from my_package.src.perception import LanePerception
from my_package.src.controller import PurePursuitController
class ParkingLogicNode(DTROS):

    def __init__(self, node_name):
        super(ParkingLogicNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # --- State Machine ---
        self.state = "SEARCHING"
        rospy.loginfo("Current State: SEARCHING")

        # --- Analysis Machine ---
        self.detector = LineDetector()

        # --- MODIFICATION 2: Timeout ke liye variables ---
        self.align_start_time = None
        self.align_timeout = rospy.Duration(30.0) # 5 second ka timeout

        # --- MODIFICATION 1: Proportional Controller ke liye variables ---
        self.camera_width = 640 # Aapki camera image ki width
        self.image_center = self.camera_width / 2
        self.kp = 0.5 # Yeh ek tuning constant hai, aap ise badal kar dekh sakte hain

        # --- Topics ---
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # --- Publishers and Subscribers ---
        self._wheels_pub = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)
        self._camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        self._processed_image_pub = rospy.Publisher(
            f"/{self._vehicle_name}/parking_node/image/processed/compressed",
            CompressedImage,
            queue_size=1
        )

        
        # --- Image Processing ---
        self._bridge = CvBridge()

    def camera_callback(self, msg):
        proc_img = None
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # --- Perception ---
        proc_img, lookahead_pt = self.perception.detect(image)

        # --- Control ---
        vel_left, vel_right = self.controller.compute_control(self.image_center, lookahead_pt)
        self.send_wheel_commands(vel_left, vel_right)

        # --- Publish processed image ---
        proc_msg = self._bridge.cv2_to_compressed_imgmsg(proc_img)
        self._processed_image_pub.publish(proc_msg)

    def send_wheel_commands(self, vel_left, vel_right):
        msg = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
        self._wheels_pub.publish(msg)

if __name__ == '__main__':
    node = ParkingLogicNode(node_name='parking_logic_node')
    rospy.spin()