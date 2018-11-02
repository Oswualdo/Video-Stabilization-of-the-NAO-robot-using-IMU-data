#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import zbar
import numpy as np
font = cv2.FONT_HERSHEY_SIMPLEX

from naoqi import ALProxy
import math
import time
robotIP="169.254.236.250"
PORT=9559
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
motion = ALProxy("ALMotion", robotIP, PORT)

def euclidean(x, y): 
    return math.sqrt( sum( [ (xi - yi)**2 for xi, yi in zip(x, y) ] ) ) 
class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("image_viewer/oswualdo/image",Image,self.callback)
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")	#grayscale image
			dimensiones = cv_image.shape #array that contains the height, width and channels of the image in this order.
			#Convert the OpenCV image to an image that the ZBAR library can understand
			imagen_zbar = zbar.Image(dimensiones[1], dimensiones[0], 'Y800', cv_image.tobytes())
			#Build a scanner-type object, which allows you to scan the image for QR codes
			escaner = zbar.ImageScanner()
			#Scan the image and save all the QR codes that are found
			escaner.scan(imagen_zbar)
			
			for codigo_qr in imagen_zbar:
				loc = codigo_qr.location #Save the coordinates of the corners
				#dat = codigo_qr.data[:-2] #Save the message of the QR code. The last two characters are line breaks to be deleted
				dat = codigo_qr.data #Save the message of the QR code. The last two characters are line breaks to be deleted
		
				#Convert the coordinates of the four corners to an array of numpy
				#So, we can pass it as a parameter to the function cv2.polylines to draw the outline of the QR code
				localizacion = np.array(loc, np.int32)
 
				#Draw the outline of the QR code in blue on the image
				cv2.polylines(cv_image, [localizacion], True, (255,0,0), 2)
 
				#Draw the four corners of the QR code
				cv2.circle(cv_image, loc[0], 3, (0,0,255), -1) #Red - upper left corner
				cv2.circle(cv_image, loc[1], 3, (0,255,255), -1) #Yellow - lower left corner
				cv2.circle(cv_image, loc[2], 3, (255,100,255), -1) #pink - lower right corner
				cv2.circle(cv_image, loc[3], 3, (0,255,0), -1) #Green - upper right corner
				
				A=loc[0]
				B=loc[3]
				C=loc[1]
				
				DA=euclidean(A, B) #you get the relative length of the image
                
				#Write the message of the QR code.
				#cv2.putText(cv_image,dat,(50,220), font, 0.5,(255,255,255),2)
				cv2.putText(cv_image,dat,(50,220), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.7,(255,255,255),2)
 
				#Search the center of the QR code rectangle
				dir_x = (loc[0][0]+loc[2][0])/2
				dir_y = (loc[0][1]+loc[2][1])/2
				#print (dir_x,dir_y)
				
				if dir_x > 230:
					print "derecha"
					motion.post.moveTo(0, 0, -0.1) # X,Y,theta
				if dir_x < 50:
					print "izquierda"
					motion.post.moveTo(0, 0, 0.1)  # X,Y,theta
				if DA < 70.0:
					print "Avanzar"
					motion.post.moveTo(0.1, 0, 0)  # X,Y,theta
 
			#Show the image
			cv2.imshow("Imagen Procesada", cv_image)
			cv2.waitKey(3)
		except CvBridgeError as e:
			print(e)

def main(args):
	motion.wakeUp()
	postureProxy.goToPosture("StandInit", 0.5)
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	postureProxy.goToPosture("Sit", 0.5)
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
