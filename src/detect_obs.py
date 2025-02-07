#! /usr/bin/env python3

# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan
# from math import *
# import os

# class OBS_detector:
# 	def __init__(self):
# 		rospy.init_node("detect_obs")  # node 이름 정하기
# 		rospy.Subscriber("/scan", LaserScan, self.lazer_CB)
# 		self.obs_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
# 		self.rate = rospy.Rate(10) # 주기 설정
# 		self.safe_range = 0.60 # 60cm
# 		self.detect_degree = 90 # 값 줄이면 sensitivity도 조정
# 		self.detect_range = 0.60
# 		self.sensitivity = 10
# 		self.degree_flag = False
# 		self.degrees = []

# 	def lazer_CB(self, msg):
# 		# os.system("clear")
# 		e_stop_degree_list = []
# 		avoid_degree_list = []
# 		# right_index_t = []
# 		# left_index_t = []

# 		if not self.degree_flag:
# 			self.degrees = [(msg.angle_min + msg.angle_increment * index) * 180/pi for index, value in enumerate(msg.ranges)]
# 			self.degree_flag = True
# 		# 인지, 판단, 제어
# 		for index, value in enumerate(msg.ranges):
# 			if 150 < abs(self.degrees[index]) and 0 < value < self.safe_range:  # lidar를 이용한 거리, +- 30 degree
# 				e_stop_degree_list.append(self.degrees[index])  # stop
# 			elif self.detect_degree < abs(self.degrees[index]) and 0 < value < self.detect_range:  # 공간의 여유가 있을 때
# 				avoid_degree_list.append(self.degrees[index])  # avoid

# 		# 제어
# 		if len(e_stop_degree_list) > 10:
# 			self.obs_pub.publish("warning")
# 			# rospy.loginfo("WARNING")
# 		else:
# 			self.obs_pub.publish("safe")
# 			# rospy.loginfo("SAFE")



# # 다음과 같이 코드를 정의하는 이유는 OOP(Object-Oriented Programming) 특성을 살리기 위해서
# def main():
# 	obs_detector = OBS_detector()
# 	rospy.spin()

# if __name__ == "__main__":
# 	main()


import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles
import os

class OBS_detector:
	def __init__(self):
		rospy.init_node("detect_obs")  # node 이름 정하기
		rospy.Subscriber("/scan", LaserScan, self.lazer_CB)
		self.obs_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
		self.rate = rospy.Rate(10) # 주기 설정
		self.safe_range = 0.40 # 60cm
		# self.detect_degree = 90 # 값 줄이면 sensitivity도 조정
		# self.detect_range = 0.80
		self.sensitivity = 10
		self.degree_flag = False
		self.degrees = []

	def lazer_CB(self, msg):
		e_stop_degree_list = []
		# # avoid_degree_list = []

		if not self.degree_flag:
			self.degrees = [(msg.angle_min + msg.angle_increment * index) * 180/math.pi for index, value in enumerate(msg.ranges)]
			self.degree_flag = True
		# # 인지, 판단, 제어
		for index, value in enumerate(msg.ranges):
			if 150 < abs(self.degrees[index]) and 0 < value < self.safe_range:  # lidar를 이용한 거리, +- 30 degree
				e_stop_degree_list.append(self.degrees[index])  # stop
		# 	# elif self.detect_degree < abs(self.degrees[index]) and 0 < value < self.detect_range:  # 공간의 여유가 있을 때
		# 	# 	avoid_degree_list.append(self.degrees[index])  # avoid

		# os.system('clear')
		# rospy.loginfo(e_stop_degree_list)
		# rospy.loginfo(len(e_stop_degree_list))
		# # # 제어
		if len(e_stop_degree_list) > self.sensitivity:
			self.obs_pub.publish("warning")
			# rospy.loginfo("WARNING")
		else:
			self.obs_pub.publish("safe")
		# 	# rospy.loginfo("SAFE")



# 다음과 같이 코드를 정의하는 이유는 OOP(Object-Oriented Programming) 특성을 살리기 위해서
def main():
	obs_detector = OBS_detector()
	rospy.spin()

if __name__ == "__main__":
	main()
