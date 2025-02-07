#! /usr/bin/env python3
#-*- coding: utf-8 -*-

# lane_tracking.py
import cv2
import numpy as np

class LaneTracking:
    def __init__(self):
        # 기본 HSV 범위 (노란색)
        self.lower_yellow = np.array([16, 189, 121])
        self.upper_yellow = np.array([33, 255, 201])
        self.brightness_threshold = 100

    def adjust_hsv_based_on_brightness(self, frame):
        """
        이미지 밝기를 기준으로 HSV 임계값을 동적으로 조정
        """
        self.lower_yellow = np.array([16, 189, 121])
        self.upper_yellow = np.array([33, 255, 201])

    def stabilize_hsv(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        avg_brightness = np.mean(v_channel)

        # 밝기가 너무 높거나 낮은 경우 보정
        if avg_brightness > 150:
            v_channel = cv2.addWeighted(v_channel, 0.8, v_channel, 0, -50)  # 밝기를 낮춤
        elif avg_brightness < 100:
            v_channel = cv2.addWeighted(v_channel, 1.2, v_channel, 0, 50)   # 밝기를 높임

        hsv[:, :, 2] = v_channel
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def detect_yellow_lane(self, frame):
        """
        HSV를 사용하여 노란색 차선을 감지
        """

        _stabilize_hsv = self.stabilize_hsv(frame)

        # 이미지를 HSV 색 공간으로 변환
        hsv = cv2.cvtColor(_stabilize_hsv, cv2.COLOR_BGR2HSV)

        # 노란색 마스크 생성
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        # 모폴로지 연산으로 마스크 개선
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask
    
    def detect_white(self, frame):
        """
        HSV를 사용하여 노란색 차선을 감지
        """

        _stabilize_hsv = self.stabilize_hsv(frame)

        # 이미지를 HSV 색 공간으로 변환
        hsv = cv2.cvtColor(_stabilize_hsv, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 100])
        upper_white = np.array([150, 100, 255])

        # 노란색 마스크 생성
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 모폴로지 연산으로 마스크 개선
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def compute_steering_angle(self, mask):
        """
        차선의 중심을 기준으로 스티어링 각도 계산
        """
        height, width = mask.shape
        mid_x = width // 2

        # 마스크의 아래쪽 영역에서 차선 중심 찾기
        bottom_mask = mask[int(height * 0.8):, :]
        moments = cv2.moments(bottom_mask)

        if moments["m00"] > 0:
            lane_center_x = int(moments["m10"] / moments["m00"])
        else:
            lane_center_x = mid_x  # 차선이 보이지 않으면 중앙으로 설정

        # 중심점에서 차선까지의 x 오프셋
        x_offset = lane_center_x - mid_x

        # 스티어링 각도 계산
        y_offset = int(height * 0.8)  # 기준 y값
        angle_to_mid_rad = np.arctan2(x_offset, y_offset)
        steering_angle = np.degrees(angle_to_mid_rad)
        return steering_angle