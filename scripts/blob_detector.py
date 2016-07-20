#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np;


class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
	self.pub_blob = rospy.Publisher("blob_detections",\
		BlobDetections.msg, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        im_cv2 = self.bridge.imgmsg_to_cv2(image_msg)     #Convert to cv2 format
	
	im_hsv = cv2.cvtColor(im_cv2, cv2.COLOR_BGR2HSV)      #Convert to HSV
	
	#Apply mask
	mask = cv2.inRange(im_hsv, np.array([24,0,0]), np.array([26, 255,255]))
	im_mask = cv2.bitwise_and(im_hsv, im_hsv, mask = mask)

	im_gray = cv2.cvtColor(im_mask,cv2.COLOR_BGR2GRAY)  #Make image gray

	#Find and draw contours
	contours,hierarchy = cv2.findContours(im_gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	#print "There are " + str(len(contours)) + "blobs"
	for i in range(0, len(contours)):
		size = cv2.contourArea(contours[i])
		if size > 2000:
        		print "Blob " + str(i) + " :size  " + str(size) + str(cv2.isContourConvex(contours[i]))
			#x,y,w,h = cv2.boundingRect(contours[i])
			#cv2.rectangle(im_cv2, (x,y), (x+w, y+h), (0,255,0),2)
			cv2.drawContours(im_cv2,contours, i,(255,255,0),3)
        try:
		self.pub_image.publish(\
			self.bridge.cv2_to_imgmsg(im_cv2, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

