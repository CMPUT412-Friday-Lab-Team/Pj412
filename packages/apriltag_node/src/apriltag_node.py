#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
import yaml
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from dt_apriltags import Detector
import rospkg

from std_msgs.msg import Int32, String
from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Transform, Vector3, Quaternion
import tf


HOST_NAME = os.environ["VEHICLE_NAME"]
IGNORE_DISTANCE_MAX = .85
IGNORE_DISTANCE_MIN = .0
DEBUG = True


def _matrix_to_quaternion(r):
    T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)


class MLNode(DTROS):

    def __init__(self, node_name):
        super(MLNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospack = rospkg.RosPack()
        self.seq = 0
        self.intrinsic = self.readYamlFile(rospack.get_path('apriltag_node') + '/src/camera_intrinsic.yaml')
        self.detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.timer = 0

        self.camera_sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.general_sub = rospy.Subscriber('/general', String, self.general_callback)

        self.tag_pub = rospy.Publisher(f'/{HOST_NAME}/detected_tagid', Int32, queue_size=10)
        self.tag_distance_pub = rospy.Publisher(f'/{HOST_NAME}/detected_tag_distance', String, queue_size=10)
        self.detections_pub = rospy.Publisher(f'/{HOST_NAME}/apriltag_detector_node/detections', AprilTagDetectionArray,
                                              queue_size=2)

    def general_callback(self, msg):
        if msg.data == 'shutdown':
            rospy.signal_shutdown('received shutdown message')

    def process_tags(self, detected_tags):
        for det in detected_tags:
            # ymin = int(np.min(det.corners[:, 1]).item())
            # x, y = det.center

            # ignore tags that are too close
            id = det.tag_id
            ihom_pose = det.pose_t
            distance = np.linalg.norm(ihom_pose)
            
            distance_msg_str = f'{id} {distance}'
            self.tag_distance_pub.publish(String(distance_msg_str))
            if IGNORE_DISTANCE_MAX < distance or IGNORE_DISTANCE_MIN > distance:
                continue

            # broadcast tag id
            # print(f'detected id: {id}')
            self.tag_pub.publish(Int32(id))
    
    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        self.timer += 1
        if self.timer % 8 == 0:
            compressed_image = np.frombuffer(msg.data, np.uint8)
            im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)

            camera_matrix = np.array(self.intrinsic["camera_matrix"]["data"]).reshape(3,3)
            camera_proj_mat = np.concatenate((camera_matrix, np.zeros((3, 1), dtype=np.float32)), axis=1)
            distort_coeff = np.array(self.intrinsic["distortion_coefficients"]["data"]).reshape(5,1)
            fx = camera_matrix[0][0].item()
            fy = camera_matrix[1][1].item()
            cx = camera_matrix[0][2].item()
            cy = camera_matrix[1][2].item()
            tag_size = 0.065  # in meters

            width = im.shape[1]
            height = im.shape[0]

            newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distort_coeff, (width,height), 1, (width,height))
            undistort_im = cv2.undistort(im, camera_matrix, distort_coeff, None, newmatrix)
            input_image = cv2.cvtColor(undistort_im, cv2.COLOR_BGR2GRAY)
            detected_tags = self.detector.detect(input_image, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tag_size)

            # pack detections into a message
            tags_msg = AprilTagDetectionArray()
            tags_msg.header.stamp = msg.header.stamp
            tags_msg.header.frame_id = msg.header.frame_id
            for tag in detected_tags:
                # turn rotation matrix into quaternion
                q = _matrix_to_quaternion(tag.pose_R)
                p = tag.pose_t.T[0]
                # create single tag detection object
                detection = AprilTagDetection(
                    transform=Transform(
                        translation=Vector3(x=p[0], y=p[1], z=p[2]),
                        rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                    ),
                    tag_id=tag.tag_id,
                    tag_family=str(tag.tag_family),
                    hamming=tag.hamming,
                    decision_margin=tag.decision_margin,
                    homography=tag.homography.flatten().astype(np.float32).tolist(),
                    center=tag.center.tolist(),
                    corners=tag.corners.flatten().tolist(),
                    pose_error=tag.pose_err,
                )
                # add detection to array
                tags_msg.detections.append(detection)
            # publish detections
            self.detections_pub.publish(tags_msg)

            self.process_tags(detected_tags)

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def onShutdown(self):
        super(MLNode, self).onShutdown()


if __name__ == '__main__':
    apriltag_node = MLNode('apriltag_node')
    rospy.spin()


