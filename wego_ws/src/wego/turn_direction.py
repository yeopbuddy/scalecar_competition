# #! /usr/bin/env python3

# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped
# from math import *
# import os

# class Turn_mode:
# 	def __init__(self):
# 		rospy.init_node("turn_mode")
# 		rospy.Subscriber("/scan", LaserScan, self.turn_CB)
# 		self.turn_pub = rospy.Publisher("direction", String, queue_size=5)
# 		self.rate = rospy.Rate(10)
# 		self.degree_flag = False
# 		self.sensitivity = 20
# 		# self.safe_range = 0.60 # 60cm
# 		self.detect_range = 0.8
# 		self.detect_degree = 90
		
# 	def turn_CB(self, msg):
# 		# e_stop_degree_list = []
# 		avoid_degree_list = []
# 		right_idx_t = []
# 		left_idx_t = []
		

# 		if not self.degree_flag:
# 			self.degrees = [(msg.angle_min + msg.angle_increment * index) * 180/pi for index, value in enumerate(msg.ranges)]
# 			self.degree_flag = True
			
# 		# 인지, 판단, 제어
# 		for index, value in enumerate(msg.ranges):
# 			# if 150 < abs(self.degrees[index]) and 0 < value < self.safe_range:  # lidar를 이용한 거리, +- 30 degree
# 			# 	e_stop_degree_list.append(self.degrees[index])  # stop
# 			if self.detect_degree < abs(self.degrees[index]) and 0 < value < self.detect_range:  # 공간의 여유가 있을 때
# 				avoid_degree_list.append(self.degrees[index])  # avoid

# 		for degree in avoid_degree_list:
# 			if degree > 0:
# 				right_idx_t.append(degree)
# 			else:
# 				left_idx_t.append(degree)

# 		len_left = len(left_idx_t)
# 		len_right = len(right_idx_t)
# 		if len_right < self.sensitivity:
# 			len_right = 0
# 		if len_left < self.sensitivity:
# 			len_left = 0
			
# 		if len_left == 0 and len_right == 0:
# 			self.turn_pub.publish("Center")
# 		elif len_left < len_right: # go left
# 			self.turn_pub.publish("Left")
# 		else: # go right
# 			self.turn_pub.publish("Right")
			
# def main():
# 	turn_model = Turn_mode()
# 	rospy.spin()
		
# if __name__ == "__main__":
# 	main()