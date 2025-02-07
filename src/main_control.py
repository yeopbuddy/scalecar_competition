#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Int32, String, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from obstacle_detector.msg import Obstacles
from ar_track_alvar_msgs.msg import AlvarMarkers

# from detection_msgs.msg import BoundingBoxes

import time
import cv2
import numpy as np
import os

from cv_bridge import CvBridge

from test_lane_detection import LaneTracking
from warper import Warper

class Main_control:
	def __init__(self):
		rospy.init_node("main_control")  # node 이름 정하기
		self.webot_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size = 1)
		self.webot_steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size = 1) # node 역할 정하기
		self.ack_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

		# rospy.Subscriber("usb_cam/image_rect_color", Image, self.img_callback)
		rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback)
		rospy.Subscriber("usb_cam/image_rect_color", Image, self.red_zone_callback)
		rospy.Subscriber("usb_cam/image_rect_color", Image, self.tunnel_callback)
		rospy.Subscriber("usb_cam/image_rect_color", Image, self.crosswalk_callback)
		rospy.Subscriber("lidar_warning", String, self.drive_CB)

		# rospy.Subscriber('yolov5/detections', BoundingBoxes, self.yolo_callback)
		# rospy.Subscriber("lidar_warning", Detection, self.yolo_callback)
		# rospy.Subscriber("sign_id", Int32, self.child_sign_callback)

		rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.alvar_callback)
		# rospy.Subscriber("obstacles", Obstacles, self.obs_callback)
		rospy.Subscriber("obstacle_slide", Float32, self.obs_callback)
		# rospy.Subscriber("direction", String, self.turn_CB)
		rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)

		self.rate = rospy.Rate(20) # 주기 설정
		# self.speed = 0.0
		self.current_lane = "SECOND" # 초기 차선
		self.turn_left_flag = 0
		self.turn_right_flag = 0
		self.speed_turn = 0.2

		# obstacle flag
		self.warn_flag = False
		self.safe_flag = True

		# red_zone
		self.red_zone_flag = False
		self.red_zone_threshold = 2500 # 값 확인하고 바꾸기
		self.red_init = False

		self.crosswalk_init = False
		self.crosswalk_flag = False
		
		self.white_threshold = 17000
		
		# self.parking_init = False
		# self.parking_flag = False
		

		# tunnel
		self.tunnel_flag = False
		self.tunnel_threshold = 50 # 값 확인하고 바꾸기

		# turn left : +
		# turn right : -
		self.turn_right = False
		self.turn_left = False
		self.initialized = False

		# 차량 주행에 필요한 객체
		self.lane_tracker = LaneTracking()
		self.warper = Warper()
		self.bridge = CvBridge()
		self.obstacle_img = []
		self.slide_img = None
		self.slide_x_location = 0
		self.current_lane_window = ""

		self.emergency_flag = 0

		#self.child_cnt = 0
		#self.sign_data = 0
		#self.slow_down_flag = 0

   # Alvar 관련 플래그 및 카운터
		self.alvar_cnt_0 = 0
		self.alvar_cnt_4 = 0
		self.alvar_left = 0
		self.alvar_right = 0

		#rabacon or tunnel
		self.rabacon_mission = 0

		self.turn_left_t1 = 0.0
		self.turn_right_t1 = 0.0

	def tunnel_callback(self, _data):
		img = self.bridge.imgmsg_to_cv2(_data)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		average_brightness = np.mean(gray)

		if average_brightness < self.tunnel_threshold:
			self.tunnel_flag = True
			# rospy.loginfo("Dark")
		else:
			self.tunnel_flag = False
			# rospy.loginfo("Bright")

	# def obs_callback(self, _data):
	# 	self.rabacon_mission_flag = _data.data

	# 	if self.rabacon_mission_flag != 1000.0 and self.rabacon_mission_flag != 3000.0:
	# 		self.rabacon_mission = 1
	# 		self.emergency_flag = 0
	# 	elif self.rabacon_mission_flag == 3000.0:
	# 		self.emergency_flag = 1
	# 		self.rabacon_mission = 0
	# 	elif self.rabacon_mission_flag == 1000.0:
	# 		self.rabacon_mission = 0
	# 		self.emergency_flag = 0
	def obs_callback(self, _data):
		self.rabacon_mission_flag = _data.data

		if self.rabacon_mission_flag != 1000.0 and self.rabacon_mission_flag != 3000.0:
			self.rabacon_mission = 1
			# self.emergency_flag = 0
		# elif self.rabacon_mission_flag == 3000.0:
		# 	self.emergency_flag = 1
		# 	self.rabacon_mission = 0
		elif self.rabacon_mission_flag == 1000.0:
			rospy.loginfo("not rabacon // not rabacon // not rabacon")
			self.rabacon_mission = 0
			# self.emergency_flag = 0

	def crosswalk_callback(self, _data):
		cv2_img = self.bridge.imgmsg_to_cv2(_data, "bgr8")

		# ROI 설정: 중간 하단 부분
		height, width, _ = cv2_img.shape
		roi_polygon = np.array([[
			(int(width * 0.25), int(height * 0.7)),  # 좌측 하단
			(int(width * 0.75), int(height * 0.7)),  # 우측 하단
			(int(width * 0.75), int(height * 0.5)),  # 우측 중단
			(int(width * 0.25), int(height * 0.5))   # 좌측 중단
		]], np.int32)  # 중간 하단 영역 설정

		# ROI 마스크 생성
		roi_mask = np.zeros_like(cv2_img[:, :, 0])  # 단일 채널 (grayscale 크기와 동일)
		cv2.fillPoly(roi_mask, roi_polygon, 255)  # 다각형을 흰색(255)으로 채우기

		# ROI 적용: 관심 영역만 남기기
		roi_img = cv2.bitwise_and(cv2_img, cv2_img, mask=roi_mask)

		# 하얀색 마스크 생성
		white_mask = self.lane_tracker.detect_white(roi_img)

		# ROI 이미지에 하얀색 필터 적용
		# result = cv2.bitwise_and(roi_img, roi_img, mask=white_mask)

		# # 결과 이미지 표시
		# cv2.imshow("White Detection with ROI", result)
		# cv2.waitKey(1)

		# result = cv2.bitwise_and(cv2_img, cv2_img, white_mask)
		# # _stabilize_hsv = self.stabilize_hsv(result)

		# # _stabilize_hsv 이미지에서 뭐 count 해서 일단 정지하게 하는 코드 ㄱㄱ
		# cv2.imshow('wm', result)
		# cv2.waitKey(1)

		white_pixel_cnt = cv2.countNonZero(white_mask)
		rospy.loginfo(f"white count ===== {white_pixel_cnt}")

		if self.white_threshold < white_pixel_cnt:
			self.crosswalk_flag = True
		else:
			self.crosswalk_flag = False
			

	# # ----------------------------------------------------------------

	def red_zone_callback(self, _data):
		cv2_img = self.bridge.imgmsg_to_cv2(_data)
		hsv_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

		lower_red1 = np.array([100, 160, 120]) # low 값 확인 후 수정 필요
		upper_red1 = np.array([130, 200, 200]) # high 값 확인 후 수정 필요

		mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
		red_mask = mask1


		kernel = np.ones((7, 7), np.uint8) # 노이즈 제거
		red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

		red_pixel_cnt = cv2.countNonZero(red_mask)
		# rospy.loginfo(f"red count ===== {red_pixel_cnt}")

		# result = cv2.bitwise_and(hsv_img, hsv_img, mask=red_mask)

		if self.red_zone_threshold < red_pixel_cnt:
			self.red_zone_flag = True
			# rospy.loginfo("!!!!!!!!!!!!!!! red")
		else:
			self.red_zone_flag = False
			# rospy.loginfo("!!!!!!!!!!!!!!! sdfajsdfljalsdkjf")


	def lane_callback(self, _data):
		cv2_img = self.bridge.imgmsg_to_cv2(_data, "bgr8")

		yellow_mask = self.lane_tracker.detect_yellow_lane(cv2_img)

		# result = cv2.bitwise_and(cv2_img, cv2_img, mask=yellow_mask)

		# _stabilize_hsv = self.stabilize_hsv(result)

		# cv2.imshow("Yellow Lane Detection", result)
		# cv2.waitKey(1)
		
		# _stabilize_hsv = self.stabilize_hsv(result)


		# 스티어링 각도 계산
		steering_angle = self.lane_tracker.compute_steering_angle(yellow_mask)

		# 차량 제어
		self.drive(steering_angle) 

	def drive_CB(self, msg):
		# rospy.loginfo(msg.data)
		if msg.data == "warning":
			self.emergency_flag = 1
			# rospy.loginfo('sldfkjsdlkfjlsdkjf')

		else:  # safe
			self.emergency_flag = 0
			
	# go straight
	def drive(self, angle):

		# os.system('clear')
		pub_data = AckermannDriveStamped()
		pub_data.header.frame_id = "base_link"
		pub_data.header.stamp = rospy.Time.now()
		# pub_data.drive.steering_angle = 0
		pub_data.drive.speed = 0.3
		# pub_data.header.stamp = rospy.Time.now()
		# 280 수정해야 될 수도 아닐 수도


		# redzone 행동 정의하는 부분

		if self.red_zone_flag and self.rabacon_mission == 0: #레드존 상황
			rospy.loginfo('red redredredredredredredredred')
			pub_data.drive.speed = 0.2
			pub_data.drive.steering_angle = angle
			# rospy.sleep(0.5)
			self.ack_pub.publish(pub_data)

		elif self.rabacon_mission == 1:
			rospy.loginfo('rabarabarabarabarabarabaraba')
			self.rabacon_drive()

		elif self.emergency_flag == 1:
			rospy.loginfo('emergency emergency emergency emergency')
			pub_data.drive.steering_angle = 0
			pub_data.drive.speed = 0.0
			self.ack_pub.publish(pub_data)
			rospy.sleep(1)
			# return

		elif self.crosswalk_flag and self.rabacon_mission == 0:
			rospy.loginfo("white zzz for 6s")
			rospy.sleep(3)
		else:
			pub_data.drive.steering_angle = angle
			self.ack_pub.publish(pub_data)

		if self.alvar_left: # 여기서 동적장애물 처리도 time.sleep 넣어서 할 수 있을듯듯듯듯
			if self.turn_left_flag == 0:
				self.turn_left_t1 = rospy.get_time()
				self.turn_left_flag = 1
			t2 = rospy.get_time()
			while t2 - self.turn_left_t1 <= 1:
				self.change_line_left()
				t2 = rospy.get_time()
			while t2 - self.turn_left_t1 <= 2:
				self.change_line_right()
				t2 = rospy.get_time()

			self.alvar_left = False
			self.turn_left_flag = 0

		elif self.alvar_right: # 아래 while문 2개에 값 1, 2로 설정한 거 실제 맵에 맞게 조정 필요 (줄여야 할 듯)
			if self.turn_right_flag == 0:
				self.turn_right_t1 = rospy.get_time()
				self.turn_right_flag = 1
			t2 = rospy.get_time()
			while t2 - self.turn_right_t1 <= 1:
				self.change_line_right()
				t2 = rospy.get_time()
			while t2 - self.turn_right_t1 <= 2:
				self.change_line_left()
				t2 = rospy.get_time()

			self.alvar_right = False
			self.turn_right_flag = 0

    # Rabacon Mission
	def rabacon_drive(self) :
		# rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
		publishing_data = AckermannDriveStamped()
		publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
		publishing_data.header.frame_id = "base_link"
		publishing_data.drive.speed = 0.3
		publishing_data.drive.steering_angle = self.rabacon_mission_flag # -를 곱해야 할 sudo 있음
		self.ack_pub.publish(publishing_data)


	def change_line_left(self) :
		pub_data = AckermannDriveStamped()
		pub_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
		pub_data.header.frame_id = "base_link"
		pub_data.drive.speed = self.speed_turn
		pub_data.drive.steering_angle = 0.5
		self.ack_pub.publish(pub_data)

	def change_line_right(self) :
		pub_data = AckermannDriveStamped()
		pub_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
		pub_data.header.frame_id = "base_link"
		pub_data.drive.speed = self.speed_turn
		pub_data.drive.steering_angle = -0.5
		self.ack_pub.publish(pub_data)


	def alvar_callback(self, _data):
		# Alvar 마커 인식을 위한 콜백 함수
		for marker in _data.markers:
			distance = np.sqrt(marker.pose.pose.position.x**2 + marker.pose.pose.position.y**2 + marker.pose.pose.position.z**2)
			if marker.id == 0 and distance <= 0.3:  # 특정 거리 이하일 때
				rospy.loginfo("ALVAR IS 0 ~~~~~~~~~~~~~~~~ GO LEFT")
				self.alvar_cnt_0 += 1
				if self.alvar_cnt_0 >= 20:
					#self.sign_data = marker.id
					self.alvar_left = 1
					self.alvar_cnt_0 = 0
			elif marker.id == 4 and distance <= 0.3:  # 특정 거리 이하일 때
				self.alvar_cnt_4 += 1
				rospy.loginfo("ALVAR IS 4 ~~~~~~~~~~~~~~~~ GO RIGHT")
				if self.alvar_cnt_4 >= 20:
					#self.sign_data = marker.id
					self.alvar_right = 1
					self.alvar_cnt_4 = 0


	def timer_callback(self, _event):
		try :
			self.drive(0.0)
		except :
			pass

	def nothing(x):
		pass

def main():
	main_control = Main_control()
	rospy.Timer(rospy.Duration(1.0/30.0), main_control.timer_callback)
	rospy.spin()

if __name__ == "__main__":
	main()