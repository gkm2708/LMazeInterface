#!/usr/bin/env python
import sys
import rospy
import roslib
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from imageQueue import imageQueue

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from std_msgs.msg import Bool


iQ = imageQueue(4)

BCcoefficients = [1,0,0] 			# Gives blue channel all the weight
GScoefficients = [0.114, 0.587, 0.299]		# Gray Scale Coffecient Luminance Method

m = np.array(GScoefficients).reshape((1,3))


class lmaze_interface():
    def __init__(self):
        #init code

        rospy.init_node("lmaze_interface")

        self.image_pub = rospy.Publisher('/interface/scene', Image, queue_size=1)
        self.reset_pub = rospy.Publisher('/interface/reset', Bool , queue_size=1)
        self.action_pub = rospy.Publisher('/interface/action', Vector3, queue_size=1)

        self.image_sub = rospy.Subscriber('/lmaze/view', Image, self.movingAugment)
        self.reset_sub = rospy.Subscriber('/player/reset', Bool, self.reset)
        self.action_sub = rospy.Subscriber('/player/action', Vector3, self.step)

        self.bridge = CvBridge()
        self.i = 0
        self.counter = 0

	self.reset()


    def movingAugment(self,msg):
        # convert msg to cv image
	self.i = self.i+1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

	gray = cv2.transform(cv_image, m)
        iQ.enqueue(gray)

        (rows,cols) = gray.shape

        summaryFrame = iQ.emit()

	if self.i%100 == 0:
            #cv2.imshow("summary Image", summaryFrame)
            #cv2.waitKey()
            cv2.imwrite('/homes/gkumar/rl/dump/image'+str(self.i)+'.png', summaryFrame)

        #rospy.loginfo(summaryFrame.shape)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(summaryFrame, "64FC1"))
        except CvBridgeError as e:
            rospy.loginfo(e)


	# just for now to check the functions
        try:
            self.action_pub.publish(Vector3(-0.2, -0.2, 0))
        except CvBridgeError as e:
            rospy.loginfo(e)


        if self.counter == 200:
            try:
                self.reset_pub.publish(Bool(True))
		self.counter = 0
            except CvBridgeError as e:
                rospy.loginfo(e)
            rospy.loginfo("Reset")
        else:
            self.counter = self.counter + 1


    def reset(self):
        try:
            self.reset_pub.publish(Bool(True))
        except CvBridgeError as e:
            rospy.loginfo(e)
	rospy.loginfo("Reset")



    def step(self,msg):
        try:
            self.action_pub.publish(Vector3(msg.x, msg.y, msg.z))
        except CvBridgeError as e:
            rospy.loginfo(e)
	rospy.loginfo("Step")



    def done(self):
        self.sub.unregister()

	self.image_pub.unregister()
        self.action_pub.unregister()
        self.reset_pub.unregister()

        rospy.signal_shutdown("done")



if __name__ == "__main__":
            lmaze = lmaze_interface()
	    rospy.spin()
