#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pytz import timezone
from datetime import datetime
import numpy as np
from sensor_msgs.msg import Image
from config import *
import cv2

import warnings
warnings.filterwarnings("ignore")
from config import *

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class PoseTracking:

    def __init__(self):
        super().__init__('pose_tracking')
        self.sub = self.create_subscription(
            Image,
            "/astra_ros/devices/default/color/image_color",
            self.callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'facial_features', 10)
        self.landmark_points = ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer', 'right_eye_inner', 'right_eye', 'right_eye_outer', 'left_ear', 'right_ear', 'mouth_left', 'mouth_right', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky', 'left_index', 'right_index', 'left_thumb', 'right_thumb', 'left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle', 'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index']
        base_options = python.BaseOptions(model_asset_path='/home/quori4/quori_files/quori_ros/src/quori_exercises/exercise_session/pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=False)
        self.pose_detector = vision.PoseLandmarker.create_from_options(options)
        self.flag = False


    def calc_angle(self, vec_0, vec_1, angle_type):
        if angle_type == 'xy':
            angle = np.arctan2(vec_1[1], vec_1[1]) - \
                np.arctan2(-vec_0[1], vec_0[0])
        elif angle_type == 'yz':
            angle = np.arctan2(vec_1[1], vec_1[2]) - \
                np.arctan2(-vec_0[1], -vec_0[2])
        elif angle_type == 'xz':
            angle = np.arctan2(vec_1[2], vec_1[0]) - \
                np.arctan2(-vec_0[2], -vec_0[0])
        
        angle = np.abs(angle*180.0/np.pi)
        if angle > 180:
            angle = 360-angle

        return 180 - angle


    def get_direction(self, landmarks):
        nose = np.array(landmarks[self.landmark_points.index('nose')])
        left_ear = np.array(landmarks[self.landmark_points.index('left_ear')])
        right_ear = np.array(landmarks[self.landmark_points.index('right_ear')])

        nose_left_ear_dist = np.linalg.norm(nose - left_ear)
        nose_right_ear_dist = np.linalg.norm(nose - right_ear)

        if nose_left_ear_dist < nose_right_ear_dist:
            return -90  # Facing right
        elif nose_right_ear_dist < nose_left_ear_dist:
            return 90  # Facing left
        elif nose_left_ear_dist == nose_right_ear_dist:
            return 0  # Facing camera
        else:
            return -180  # Facing away from camera


    def callback(self, data):
        if not self.flag:
            return

        image = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
        cv2.imwrite('test.jpg', image) 
        image = mp.Image.create_from_file('test.jpg')

        results = self.pose_detector.detect(image)
        if results.pose_landmarks:
            ct = datetime.now(tz)

            # Sort detected people by x-coordinate of nose (left to right)
            sorted_people = sorted(results.pose_landmarks, key=lambda person: person.landmark[self.landmark_points.index('nose')].x)

            # Publish number of detected people
            num_people_msg = Float64MultiArray()
            num_people_msg.data = [len(sorted_people)]
            self.publisher_.publish(num_people_msg)

            # For each detected person, calculate and publish direction
            for person in sorted_people:
                landmarks = []
                for i, landmark in enumerate(person):
                    landmarks.append([landmark.x, landmark.y, landmark.z])

                direction = self.get_direction(landmarks)
                direction_msg = Float64MultiArray()
                direction_msg.data = [direction]
                self.publisher_.publish(direction_msg)

                angle_msg = Float64MultiArray()
                data = []
                for joint_group, angle_description in ANGLE_INFO:
                    indices = [self.landmark_points.index(angle_description[i]) for i in range(3)]
                    points = [np.array(landmarks[i]) for i in indices]
                    vec_0 = points[0] - points[1]
                    vec_1 = points[2] - points[1]

                    for plane in ['xy', 'yz', 'xz']:
                        angle = self.calc_angle(vec_0, vec_1, plane)
                        data.append(angle)
                
                angle_msg.data = data
                angle_pub.publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)

    pose_tracking = PoseTracking()
    rclpy.spin(pose_tracking)

    pose_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()