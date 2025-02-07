#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
import cv2
from test_lane_detection import LaneTracking  # lane_tracking.py에서 가져옴
import numpy as np

class MainControl:
	def __init__(self):
		rospy.init_node('main_control_node')

		# ROS 이미지 구독
		rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)

		# Ackermann 메시지 퍼블리셔
		self.ack_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

		# OpenCV-ROS 변환 브릿지
		self.bridge = CvBridge()

		# 차선 추적 클래스 초기화
		self.lane_tracker = LaneTracking()

	# def adjust_gamma(image, gamma=1.0):
	# 	invGamma = 1.0 / gamma
	# 	table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(0, 256)]).astype("uint8")
	# 	return cv2.LUT(image, table)

	def drive(self, steering_angle):
		"""
		AckermannDriveStamped 메시지를 퍼블리시하여 차량을 제어
		"""
		pub_data = AckermannDriveStamped()
		pub_data.header.frame_id = "base_link"
		pub_data.header.stamp = rospy.Time.now()

		# 스티어링 각도와 속도 설정
		pub_data.drive.steering_angle = np.radians(steering_angle)  # 각도를 라디안으로 변환
		pub_data.drive.speed = 0.3  # 고정 속도

		# Ackermann 메시지 퍼블리시
		self.ack_pub.publish(pub_data)
		rospy.loginfo(f"Driving with Steering Angle: {steering_angle:.2f} degrees")

	def image_callback(self, msg):
		try:
			# ROS Image 메시지를 OpenCV 이미지로 변환
			frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except Exception as e:
			rospy.logerr(f"Failed to convert image: {e}")
			return

		# 조명 변화에 따른 HSV 범위 조정
		self.lane_tracker.adjust_hsv_based_on_brightness(frame)

		# 노란색 차선 감지
		mask = self.lane_tracker.detect_yellow_lane(frame)

		# 스티어링 각도 계산
		steering_angle = self.lane_tracker.compute_steering_angle(mask)

		# 차량 제어
		self.drive(steering_angle)

		# 결과 시각화
		result = cv2.bitwise_and(frame, frame, mask=mask)
		
		# gamma_corrected = self.adjust_gamma(result, gamma=0.5)
		
		cv2.putText(result, f"Steering Angle: {steering_angle:.2f} deg", (10, 50),
					cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		cv2.imshow("Yellow Lane Detection", result)
		cv2.waitKey(1)


if __name__ == '__main__':
    try:
        node = MainControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass