#!/usr/bin/env python
import sys
import rospy
import roslib
import numpy as np


class imageQueue:
    def __init__(self, window):

        self.width = 84
        self.height = 84
        self.window = window
        self.items = np.zeros((self.window+1, self.width, self.height))

    def enqueue(self, item):

        for x in range(self.window,1):
            self.items[x] = self.items[x-1]

        self.items[0] = item

        #rospy.loginfo(item.shape)
        #rospy.loginfo(self.items.shape)

    def emit(self):

        firstImage = np.zeros((self.width,self.height))
        finalImage = np.zeros((self.width,self.height))
        factor     = 0

        for x in range(0,self.window):
            tempImage = self.items[x]*(self.window-x)/self.window
	    finalImage =np.add( finalImage , tempImage)
	    factor = factor + (self.window - x)

        return finalImage[:,:]*self.window/factor

