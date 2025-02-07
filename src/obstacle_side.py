#! /usr/bin/env python3
#-*- coding: utf-8 -*-

# import rospy
# import math
# from time import sleep
# from std_msgs.msg import String
# from obstacle_detector.msg import Obstacles
# from std_msgs.msg import Int32, Float32


# class ClusterLidar :

#     def __init__(self) :
#         # rospy.Subscriber("/raw_obstacles", Obstacles, self.rabacon)
#         self.obstacle_slide_pub = rospy.Publisher("obstacle_slide", Float32, queue_size = 5)
#         rospy.Subscriber("obstacles", Obstacles, self.obs_slide)
#         self.target_distance = 0.4
#         self.kp = -0.9

#     # select close rabacon
#     def obs_slide(self, _data)  :
#         left_obs = []
#         right_obs = []
#         front_obs = []
#         for i in _data.circles :
#             if -1.7 < i.center.x < 0  :
#                 if 0 < i.center.y < 1 :
#                     right_obs.append(i)
#                 elif -1 < i.center.y < 0 :
#                     left_obs.append(i)

#             if -0.5 < i.center.x < 0:
#                 if 0 < i.center.y < 0.2 or  -0.2 < i.center.y < 0:
#                     front_obs.append(i)

#         if len(front_obs):
#             self.obstacle_slide_pub.publish(3000.0)
#         elif len(left_obs) >= 1 and len(right_obs) >= 1:
#             left_close_rabacon = sorted(left_obs, key = lambda x : -x.center.x)[0]
#             right_close_rabacon = sorted(right_obs, key = lambda x : -x.center.x)[0]
#             # 거리 오차 계산
#             distance_error = left_close_rabacon.center.y + right_close_rabacon.center.y

#             # 조향각 계산 (P 제어)
#             steering_angle = self.kp * distance_error
#             self.obstacle_slide_pub.publish(steering_angle)

#         else :
#             self.obstacle_slide_pub.publish(1000.0)



# def run() :
#     rospy.init_node("Obs_slide")
#     cluster = ClusterLidar()
#     rospy.spin()

# if __name__=='__main__' :
#     run()


import os
import rospy
import math
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Float32

class ClusterLidar:
    def __init__(self):
        self.obstacle_slide_pub = rospy.Publisher("obstacle_slide", Float32, queue_size=5)
        rospy.Subscriber("obstacles", Obstacles, self.obs_slide)

        self.target_distance = 0.25  # 목표 거리
        self.kp = 0.5               # 비례 이득
        self.ki = 0.01              # 적분 이득
        self.kd = 0.1               # 미분 이득

        self.previous_error = 0.0   # 이전 거리 오차
        self.integral = 0.0         # 적분 값

    def obs_slide(self, _data):
        left_obs = []
        right_obs = []
        # front_obs = []

        # for i in _data.circles:
        #     if -1.5 < i.center.x < 0:
        #         if 0 < i.center.y <= 1.0:
        #             right_obs.append(i)
        #         elif -1.0 <= i.center.y < 0:
        #             left_obs.append(i)
        for i in _data.circles:
            if -1.5 < i.center.x < 0:
                if 0 < i.center.y <= 0.05:
                    right_obs.append(i)
                elif -0.05 <= i.center.y < 0:
                    left_obs.append(i)

            # if -0.7 < i.center.x < 0 and abs(i.center.y) < 0.1:
            #     front_obs.append(i)

        # rospy.loginfo(f"left_obs = {len(left_obs)}")
        # rospy.loginfo(f"right_obs = {len(right_obs)}")
        # rospy.loginfo(f"front_obs = {len(front_obs)}")

        # 장애물 판단 및 조향각 계산
        if len(left_obs) >= 1 and len(right_obs) >= 1:
            left_close_rabacon = sorted(left_obs, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_obs, key = lambda x : -x.center.x)[0]
            raba = (left_close_rabacon.center.y + right_close_rabacon.center.y)
            self.obstacle_slide_pub.publish(-raba * 2.0)
            # left_closest = sorted(left_obs, key=lambda x: math.hypot(x.center.x, x.center.y))[0]
            # right_closest = sorted(right_obs, key=lambda x: math.hypot(x.center.x, x.center.y))[0]

            # # 양쪽 장애물의 중심점 계산
            # left_distance = math.hypot(left_closest.center.x, left_closest.center.y)
            # right_distance = math.hypot(right_closest.center.x, right_closest.center.y)

            # # 거리 차이 계산
            # distance_error = (right_distance - left_distance)
            # self.integral += distance_error
            # derivative = distance_error - self.previous_error
            # self.previous_error = distance_error

            # steering_angle = self.kp * distance_error + self.ki * self.integral + self.kd * derivative
            # self.obstacle_slide_pub.publish(steering_angle)

        # elif len(front_obs) >= 1:
        #     # 앞에 장애물이 있는 경우 특정 값으로 회피 (3000.0)
        #     self.obstacle_slide_pub.publish(3000.0)
        else:
            # 장애물이 없는 경우 특정 값 유지 (1000.0)
            self.obstacle_slide_pub.publish(1000.0)

def run():
    rospy.init_node("Obs_slide")
    cluster = ClusterLidar()
    rospy.spin()

if __name__ == '__main__':
    run()
