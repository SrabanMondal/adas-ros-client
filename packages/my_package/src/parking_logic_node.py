#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
import numpy as np
from cv_bridge import CvBridge

#from my_package.src.utils import LineDetector
from .utils import LineDetector
class ParkingLogicNode(DTROS):

    def __init__(self, node_name):
        super(ParkingLogicNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # --- State Machine Initial State ---
        self.state = "SEARCHING"
        rospy.loginfo("Current State: SEARCHING")

        self.detector = LineDetector()

        # --- Vehicle Name and Topics ---
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # --- Publishers and Subscribers ---
        self._wheels_pub = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)
        self._camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        
        # --- Image Processing ---
        self._bridge = CvBridge()

    def camera_callback(self, msg):
        # Image Conversion
        try:
            image = self._bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            rospy.logerr(e)
            return

        # --- State Machine Logic ---
        if self.state == "SEARCHING":
            white_line_pos = self.detector.detect_white_line(image)
            
            if white_line_pos is not None:
                cx, cy = white_line_pos
                rospy.loginfo(f"White line: ({cx}, {cy}). Aligning...")
                self.state = "ALIGNING"
                rospy.loginfo("Current State: ALIGNING")
            else:
                self.send_wheel_commands(0.2, 0.2)

        elif self.state == "ALIGNING":
            blue_line_pos = self.detector.detect_blue_line(image)
            
            if blue_line_pos is not None:
                self.state = "PARKING"
                rospy.loginfo("Current State: PARKING")
            else:
                self.send_wheel_commands(0.2, 0.2)

        elif self.state == "PARKING":
            rospy.loginfo("Parking maneuver shuru...")
            self.send_wheel_commands(0, 0); rospy.sleep(1)
            self.send_wheel_commands(0.3, -0.3); rospy.sleep(2)
            self.send_wheel_commands(0.3, 0.3); rospy.sleep(1.5)
            self.send_wheel_commands(0, 0)
            self.state = "PARKED"
            rospy.loginfo("Current State: PARKED")

        elif self.state == "PARKED":
            self.send_wheel_commands(0, 0)
            rospy.loginfo("Bot park ho chuka hai. Stopping camera feed.")
            self._camera_sub.unregister()

    def send_wheel_commands(self, vel_left, vel_right):
        msg = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
        self._wheels_pub.publish(msg)

if __name__ == '__main__':
    node = ParkingLogicNode(node_name='parking_logic_node')
    rospy.spin()