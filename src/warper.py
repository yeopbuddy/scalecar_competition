#! /usr/bin/env python3

import cv2
import numpy as np

class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)
         
        # distort src(INPUT) to dst(OUTPUT) 
        # src = np.float32([ # 4개의 원본 좌표 점
        #     [w * 1.6, h * 1.3], # [1024, 624]
        #     [w * (-0.1), h * 1.3], # [-64.0, 624]
        #     [0, h * 0.62], # [0, 297.6]
        #     [w, h * 0.62], # [640, 297.6]
        # ])
        # dst = np.float32([ # 4개의 결과 좌표 점
        #     [w * 0.65, h * 0.98], # [416, 470.4]
        #     [w * 0.35, h * 0.98], # [224, 470.4]
        #     [w * (-0.3), 0], # [-192, 0]
        #     [w * 1.3, 0], # [832, 0]
        # ])

        src = np.float32([
            [w * 1.7, h * 1.1],  # x 좌표를 약간 넓히고, y 좌표를 줄여서 더 멀리 포함
            [w * (-0.2), h * 1.1],
            [w * 0.2, h * 0.5],   # 위쪽 영역 포함
            [w * 0.8, h * 0.5],   # 위쪽 영역 포함
        ])
        
        # 결과 이미지에서 원근 효과 제거된 정사각형으로 변환
        dst = np.float32([
            [w * 0.65, h * 0.98],  # 변환 후 아래쪽 좌표 유지
            [w * 0.35, h * 0.98],
            [w * 0.1, 0],          # 위쪽 좌표 좁혀서 정사각형으로 맞춤
            [w * 0.9, 0],
        ])

        # ----------------------------------------
        # image = np.zeros((h, w, 3), dtype=np.uint8)
        # p_int = src.astype(int)
        # for i in range(len(p_int)):
        #     pt1 = tuple(p_int[i])
        #     pt2 = tuple(p_int[(i+1) % 4])
        #     cv2.line(image, pt1, pt2, color=(0, 255, 0), thickness=2)
        # for i, point in enumerate(p_int):
        #     cv2.circle(image, tuple(point), radius=5, color=(0, 0, 255), thickness=-1)
        # cv2.imshow("rect", image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # ----------------------------------------
        

        self.M = cv2.getPerspectiveTransform(src, dst) # self.M : 투시변환 행렬(src to dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # self.Minv : 투시변환 행렬(dst to src)

    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]), # img w, h
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )