#! /usr/bin/env python3
#-*- coding: utf-8 -*-
#-*- coding: future_fstrings -*-
import rospy

from std_msgs.msg import Int32, String
# from sensor_msgs.msg import Image
from fiducial_msgs.msg import Fiducial, FiducialArray

class Sign_Check():
    def __init__(self):
        self.sign_id = 0
        rospy.init_node("sign_id")
        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.child_sign_callback)
        self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)

        self.pub_cnt = 0

        

    def child_sign_callback(self, _data):
        # rospy.loginfo(_data)
        if (len(_data.fiducials) > 0 ) :
            self.sign_id = _data.fiducials[0].fiducial_id # aruco detect launch 파일에서 camera -> usb_cam, dictionary : 규격에 맞게
            rospy.loginfo("################## ID : {}".format(self.sign_id)) #start : 1, end : 2 ((주차? ㅋㅋ))
            self.sign_id_pub.publish(self.sign_id) 
            self.pub_cnt = 0
        else :
            self.pub_cnt += 1
            if self.pub_cnt > 20:
                self.sign_id_pub.publish(0)
                self.pub_cnt = 0

def run():
    # rospy.init_node("sign_id")
    new_class = Sign_Check()
    rospy.spin()


if __name__ == '__main__':
    run()
