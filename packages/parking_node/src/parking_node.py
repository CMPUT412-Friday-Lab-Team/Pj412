#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32, String, Float32
from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from geometry_msgs.msg import Transform, Vector3, Quaternion

MAX_DIST = 0.85

HOST_NAME = os.environ["VEHICLE_NAME"]
class Parking(DTROS):

    def __init__(self, node_name):
        super(Parking, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        #Static Parameters
        self._radius = 0.0318
        self._robot_width = 0.1

        #Subscibers
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/apriltag_detector_node/detections', AprilTagDetectionArray,self.callback())
        self.sub_left_wheel = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback(),callback_args="left")
        self.sub_right_wheel = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback(),callback_args="right")

        #Publishers
        self.pub = rospy.Publisher(f"/{HOST_NAME}/car_cmd_switch_node/cmd", Twist2DStamped,queue_size=1)
        self.genreal_shutdown = rospy.Publihseer('/general', String, queue_size = 1)
        #Encoder variables
        self.left_tick = None
        self.right_tick = None

        self.delta_left = None
        self.delta_right = None

        #Move and Turning variables
        self.left_distance = 0
        self.right_distance = 0
        self.angle = 0

        #Apriltag variables
        self.rotation_matrix = None
        self.translation_matrix = None
        self.tag_id = None

    def callback(self,msg):
        for i in msg.detections:
            print(f"Z: {i.transform.translation.z}")
            if i.transform.translation.z < MAX_DIST:
                self.translation_matrix = i.transform.translation
                self.rotation_matrix = i.transform.rotation
                self.tag_id = i.tag_id

    def wheel_callback(self,msg,wheel):
        if wheel == "left":
            if self.left_tick == None:
                self.left_tick = abs(msg.data)
            orig_tick = self.left_tick

        elif wheel == "right":
            if self.right_tick == None:
                self.right_tick = abs(msg.data)
            orig_tick = self.right_tick

        delta = 2 * math.pi * self._radius*(abs(msg.data) - orig_tick)/ msg.resolution
        
        if wheel == "left":
            self.left_tick = abs(msg.data)
            self.delta_left = delta

        else:
            self.right_tick = abs(msg.data)
            self.delta_right = delta

    def reset_variables(self):
        self.angle = 0
        self.left_distance = 0
        self.right_distance = 0
        
    def turn(self,direction,angle=(math.pi/2),v1=0.3,v2=0.3):
        self.reset_variables()

        if direction == "left":
            v1 *= -1

        else:
            v2 *= -1
        
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v1
        msg.vel_right = v2

        while self.angle < angle:
            self.pub.publish(msg)
            self.angle += (self.delta_right + self.delta_left)/(2*self._robot_width)
            rospy.sleep(0.1)


    def move(self,distance,v1=0.4,v2=0.4):
        self.reset_variables()

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v1
        msg.vel_right = v2

        d_traveled = 0

        while d_traveled < distance:
            self.pub.publish(msg)
            self.left_distance += self.delta_left
            self.right_distance += self.delta_right
            d_traveled = (self.left_distance + self.right_distance)/2

    def take_position(self):
        park_slot = int(input("Which parking slot?: "))
        if park_slot == 1 or park_slot == 3:
            self.move(0.5)
            if park_slot == 1:
                self.turn("left",(math.pi/4))
            else:
                self.turn("right",(math.pi/4))
        else:
            self.move(0.25)
            if park_slot == 2:
                self.turn("left",math.pi/4)
            else:
                self.turn("right",math.pi/4)

    def shutdown(self):
        self.genreal_shutdown.publish("shutdown")
        rospy.signal_shutdown("Shutting Down ...")

    def main(self):
        self.take_position()

        """
        Main idea is:
                taking position -> calculate distance using translation matrix -> Do 180 turn -> Use Rotation Matrix to Allign -> Go Backwards
        
        
        """
        pass

if __name__ == '__main__':
    parking_node = Parking('parking_node')
    rospy.spin()


