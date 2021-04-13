#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
from posedetection_msgs.msg import ObjectDetection
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_srvs.srv import SetBool, SetBoolResponse

import time
import sys


class Berometer():

    def __init__(self):
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber(
            rospy.resolve_name('~input'),
            Image, self.image_cb, queue_size=1, buff_size=2**26)

        self.sub_cam_info = rospy.Subscriber(
            rospy.resolve_name('~camera_info'),
            CameraInfo, self.camera_info_cb, queue_size=1)

        self.sub_barometer_rect = rospy.Subscriber(
            rospy.resolve_name('~rect'),
            ObjectDetection, self.detection_cb, queue_size=1)

        self.debug_img_pub = rospy.Publisher('debug_image', Image, queue_size=10)

        self.measure_srv = rospy.Service('barometer_measure', SetBool, self.measure_flag_cb)

        self.img_h = None
        self.img_w = None
        self.img_k = None

        self.bbox = None

        self.barometer_size = rospy.get_param('~barometer_size', 0.1)
        self.detect_score_thresh = rospy.get_param('~detect_score_thresh', 0.7)

        # image process for hough line detection
        self.canny_thresh1 = rospy.get_param('~canny_thresh1', 50)
        self.canny_thresh2 = rospy.get_param('~canny_thresh2', 100)

        self.hough_vot_thresh = rospy.get_param('~hough_vot_thresh', 50)
        # standard barometer (270 degree)
        self.start_value = rospy.get_param('~start_value', 0)
        self.end_value = rospy.get_param('~end_value', 10)

        self.start_angle = np.pi * 1.25
        self.end_angle = - np.pi * 0.25

        self.debug = rospy.get_param('~debug', False)
        self.measure_flag = self.debug

    def measure_flag_cb(self, msg):
        self.measure_flag = msg.data

        if self.measure_flag and self.bbox is not None:
            self.sub_barometer_rect = rospy.Subscriber(
                rospy.resolve_name('~rect'),
                ObjectDetection, self.detection_cb, queue_size=1)

            self.bbox = None

        return SetBoolResponse(True, "")

    def camera_info_cb(self, msg):
        self.img_h = msg.height
        self.img_w = msg.width

        self.img_k = np.array(msg.K).reshape(3,3)

        self.sub_cam_info.unregister()

    def detection_cb(self, msg):

        if self.img_k is None:
            return

        if msg.objects[0].reliability < self.detect_score_thresh:
            return

        object_lt = np.array([msg.objects[0].pose.position.x, msg.objects[0].pose.position.y, msg.objects[0].pose.position.z])
        object_rb = object_lt + np.array([self.barometer_size, self.barometer_size, 0])
        self.bbox = []
        self.bbox.append(object_lt[0] / object_lt[2] * self.img_k[0,0] + self.img_k[0,2]) # x1
        self.bbox.append(object_lt[1] / object_lt[2] * self.img_k[1,1] + self.img_k[1,2]) # y1

        self.bbox.append(object_rb[0] / object_rb[2] * self.img_k[0,0] + self.img_k[0,2]) # x2
        self.bbox.append(object_rb[1] / object_rb[2] * self.img_k[1,1] + self.img_k[1,2]) # y2

        self.sub_barometer_rect.unregister()

    def image_cb(self, msg):

        if not self.measure_flag:
            return

        if self.bbox is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        bbox = np.round(self.bbox).astype(np.int)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        mask = np.ones((self.img_h, self.img_w), dtype=bool)
        mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = False
        blurred[mask] = 0
        edged = cv2.Canny(blurred, self.canny_thresh1, self.canny_thresh2, 255)
        mask = np.ones((self.img_h, self.img_w), dtype=bool)
        bottom_margin = 0.25
        mask[bbox[1] + 1:bbox[3] - int(bottom_margin * (bbox[3] - bbox[1])), bbox[0] + 1:bbox[2] - 1] = False
        edged[mask] = 0

        cv2.rectangle(img, tuple(bbox[0:2]), tuple(bbox[2:4]), (0, 0,255), 2)

        lines = cv2.HoughLinesP(edged, 1, np.pi/180, threshold=self.hough_vot_thresh, minLineLength= (bbox[2] - bbox[0]) * 0.1, maxLineGap= (bbox[2] - bbox[0]) * 0.05)

        if lines is not None:

            x1, y1, x2, y2 = lines[0,0]

            cx = (self.bbox[0] + self.bbox[2])/2
            if np.abs(x1-cx) < np.abs(x2-cx):
                ang = np.arctan2(y1 - y2, x2 - x1)
            else:
                ang = np.arctan2(y2 - y1, x1 - x2)


            if ang < -np.pi / 2:
                ang += np.pi * 2

            # cheat mode for demo
            if ang > 0:
                return
            cy =  y2 + np.tan(ang) * (x2 - cx)


            value = (self.end_value - self.start_value) * (ang - self.start_angle) / (self.end_angle - self.start_angle) + self.start_value
            #print(ang, value, self.start_value, self.end_value)

            #cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.line(img,(int(cx),int(cy)),(x2,y2),(0,255,0),2)
            cv2.putText(img, 'Vacuum-gauge is {:.3f} MPa'.format(value), (0, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
            #print("estimated angle: {}; value: {}".format(ang, value))


        #img_msg = self.bridge.cv2_to_imgmsg(gray, "mono8")
        #img_msg = self.bridge.cv2_to_imgmsg(edged, "mono8")
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.debug_img_pub.publish(img_msg)



if __name__ == '__main__':
    rospy.init_node('barometer_measurement')
    barometer_measurement = Berometer()
    rospy.spin()


