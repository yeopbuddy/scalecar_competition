#! /usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *
import rospy

TOTAL_CNT = 50

class LineTracking:
    def __init__(self):
        self.current_line = "DEFAULT"
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None
        self.left_cnt = 25
        self.right_cnt = 25

    def linetracking(self, img):

        fork_detected = False
        width_threshold = 0

        x_location = None
        # init out_img, height, width
        out_img = np.dstack((img, img, img)) * 255 # deleted
        # out_img = img # added
        height = img.shape[0]
        width = img.shape[1]

        # num of windows and init the height
        window_height = 7
        nwindows = 30

        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()

        # nonzero = np.where(img == 0)

        #print nonzero
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        #print nonzerox
        # init data need to sliding windows
        margin = 20
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []

        win_h1 = 400
        win_h2 = 480
        win_l_w_l = 140-40
        win_l_w_r = 240-40
        win_r_w_l = 310+40
        win_r_w_r = 440+40

        # first location and segmenation location finder
        # draw line
        # 130 -> 150 -> 180
        # pts_left = np.array([[width/2 - 70, height], [width/2 - 70, height - 60], [width/2 - 170, height - 80], [width/2 - 170, height]], np.int32)
        pts_left = np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
        # pts_right = np.array([[width/2 + 30, height], [width/2 + 30, height - 80], [width/2 + 120, height - 110], [width/2 + 120, height]], np.int32)
        pts_right = np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        #pts_center = np.array([[width/2 + 90, height], [width/2 + 90, height - 150], [width/2 - 60, height - 231], [width/2 - 60, height]], np.int32)
        #cv2.polylines(out_img, [pts_center], False, (0,0,255), 1)
        pts_catch = np.array([[0, 340], [width, 340]], np.int32)
        cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)


        # indicies before start line(the region of pts_left)
        # 337 -> 310
        good_left_inds = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
        good_right_inds = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]

        # left line exist, lefty current init
        line_exist_flag = None
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None


        if len(good_left_inds) > len(good_right_inds):
            self.current_line = "LEFT"
            line_flag = 1
            x_current = np.int32(np.mean(nonzerox[good_left_inds]))
            y_current = np.int32(np.mean(nonzeroy[good_left_inds]))
            max_y = y_current
        elif len(good_right_inds) > len(good_left_inds):
            # print("IM IM RIzght GODD!!!!!!!!!!!!!!!!!!!!")
            if (self.left_cnt + self.right_cnt <= TOTAL_CNT) :
                # print("LEFCOUNT: {}".format(self.left_cnt))
                # print("RIGHTCOUNT: {}".format(self.right_cnt))
                self.right_cnt += 1
                self.left_cnt -=1

            self.current_line = "RIGHT"
            line_flag = 2
            x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            y_current = np.int32(np.max(nonzeroy[good_right_inds]))
        else:
            self.current_line = "MID"
            line_flag = 3

        if line_flag != 3:
            # it's just for visualization of the valid inds in the region: ind dot
            for i in range(len(good_left_inds)):
                    img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
            # window sliding and draw
            for window in range(0, nwindows):
                if line_flag == 1:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low + int(width * 0.27), win_y_low), (win_x_high + int(width * 0.27), win_y_high), (255, 0, 0), 1)
                    # indicies of dots in nonzerox in one square
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    # check num of indicies in square and put next location to current
                    if len(good_left_inds) > len(good_right_inds):
                        x_current = np.int32(np.mean(nonzerox[good_left_inds]))

                    elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                        p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
                        x_current = np.int32(np.polyval(p_left, win_y_high))
                    # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current + int(width * 0.135)
                else: # change line from left to right above(if)
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    cv2.rectangle(out_img, (win_x_low - int(width * 0.27), win_y_low), (win_x_high - int(width * 0.27), win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    if len(good_right_inds) > len(good_left_inds):
                        x_current = np.int32(np.mean(nonzerox[good_right_inds]))
                    elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
                        x_current = np.int32(np.polyval(p_right, win_y_high))
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current - int(width * 0.135)

                left_lane_inds.extend(good_left_inds)

        # rospy.loginfo(f"폭은 {width}")

        if x_location == None:
            x_location = width // 2
            # rospy.loginfo("DARK")

        lane_width = np.mean(nonzerox[right_lane_inds]) - np.mean(nonzerox[left_lane_inds])

  

        if lane_width > width_threshold:
            fork_detected = True
        elif lane_width <= width_threshold:
            fork_detected = False



        return out_img, x_location, self.current_line