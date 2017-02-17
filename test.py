#!/usr/bin/env python
import roslib; roslib.load_manifest('ptam')
import cv2
import numpy as np
import time

import math
import rospy
import sys
import time,math
from sensor_msgs.msg import Image as Im, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import Tkinter 
import tkMessageBox
from Tkinter import *
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist,Vector3      # for sending commands to the drone
from std_msgs.msg import Empty           # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
publand = rospy.Publisher('/ardrone/land',Empty,queue_size=10)
rospy.init_node('takeoff', anonymous=True)
pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=10)
pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
qua=0
token=0
top = Tkinter.Tk()
published=0
published2=0
key=0
top.minsize(width=400, height=400)
top.maxsize(width=400, height=400)
x = Entry(top, bd =1,width="20")
y = Entry(top, bd =1,width="20")
z = Entry(top, bd =1,width="20")
r = Entry(top, bd =1,width="20")
p = Entry(top, bd =1,width="20")
ya = Entry(top, bd =1,width="20")
def panoram():
	global key
	key=1
	img1 = val
	rospy.sleep(1.41)
	img2 = val
	rospy.sleep(1.41)
	img3 = val
	rospy.sleep(1.41)
	img4 = val
	rospy.sleep(1.41)
	img5 = val
	out = np.zeros((img1.shape[0],5*img1.shape[1],3), np.uint8)
	for i in range(img1.shape[0]):
		for j in range(img1.shape[1]):
			out[i,j]=img1[i,j]
			out[i,img1.shape[1]+j]=img2[i,j]
			out[i,2*img1.shape[1]+j]=img3[i,j]
			out[i,3*img1.shape[1]+j]=img4[i,j]
			out[i,4*img1.shape[1]+j]=img5[i,j]
	cv2.imshow("IIG",out)


def land():
	while alt>1500 :
		pubCommand.publish(Twist(Vector3(0,0,-1),Vector3(0,0,0)))
		rospy.sleep(1);
	published =0
        published2=0
	publand.publish(Empty())
	rospy.sleep(0.5)
	print "land"
def takeoff():
	pubCommand.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	pubTakeoff.publish(Empty())
	rospy.sleep(0.5)
	return
	pubCommand.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	pubCommand.publish(Twist(Vector3(1,0,0),Vector3(0,0,0)))
	rospy.sleep(1)
	pubCommand.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	land()
	print "takeOFF"
def exe():
	x1=float(x.get())
	y1=float(y.get())
	z1=float(z.get())
	r1=float(r.get())
	p1=float(p.get())
	ya1=float(ya.get())
	pubCommand.publish(Twist(Vector3(x1,y1,z1),Vector3(r1,p1,ya1)))
def callback(navdata):
	an = navdata.rotZ
	global alt
	alt=  navdata.altd
	global pos_found
	global pos_f
	global in_an
	
def image_callback(ros_image):
  global val
  bridge = CvBridge()
  try:
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8") # converting the sensor_msgs/image type to IplImage 
  except CvBridgeError, e:
    print "NOT"

  frame = np.array(frame, dtype=np.uint8) 
  
  val = frame	


  cv2.waitKey(1)
'''def takeoff():




   
   pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
   rospy.Subscriber("/ardrone/bottom/image_raw", Image, image_callback) 

   rospy.Subscriber("/ardrone/navdata", Navdata, callback)
	
   rate = rospy.Rate(1) # 10hz
   
   rospy.sleep(1)
   pubTakeoff.publish(Empty())

   rospy.sleep(3)
   pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
   rospy.sleep(3)
   pubCommand.publish(Vector3(0,0,0),Vector3(0,0,-1))
   rospy.sleep(10)
   pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
   rospy.sleep(5)
   pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
   rospy.sleep(3)
   print "move"

   publand.publish(Empty())
'''
def hover_f():
	pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
def rotate():
	pubCommand.publish(Vector3(0,0,0),Vector3(0,0,-1))
	panoram()
def init():
	
	pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
	B = Tkinter.Button(top, text ="Takeoff", command = takeoff)
	B.pack()
	B.place(x=10,y=20)

	B1 = Tkinter.Button(top, text ="Land", command = land)
	B1.pack()
	B1.place(x=100,y=20)

	xl = Label(top, text="X : ")
	xl.pack()
	xl.place(x=10,y=60)

	x.pack()
	x.place(x=60,y=60)


	yl = Label(top, text="Y : ")
	yl.pack()
	yl.place(x=10,y=90)

	y.pack()
	y.place(x=60,y=90)



	zl = Label(top, text="Z : ")
	zl.pack()
	zl.place(x=10,y=120)

	z.pack()
	z.place(x=60,y=120)


	rl = Label(top, text="Roll : ")
	rl.pack()
	rl.place(x=10,y=150)

	r.pack()
	r.place(x=60,y=150)


	pl = Label(top, text="Pitch : ")
	pl.pack()
	pl.place(x=10,y=180)

	p.pack()
	p.place(x=60,y=180)

	yal = Label(top, text="Yaw : ")
	yal.pack()
	yal.place(x=10,y=210)

	ya.pack()
	ya.place(x=60,y=210)



	B3 = Tkinter.Button(top, text ="Execute", command = exe)
	B3.pack()
	B3.place(x=100,y=240)

	hover = Tkinter.Button(top, text ="Hover", command = hover_f)
	hover.pack()
	hover.place(x=150,y=20)

	pan = Tkinter.Button(top, text ="Panorama", command = rotate)
	pan.pack()
	pan.place(x=210,y=20)

	top.mainloop()
def image_bottom(ros_image):
	#print "adsfadf"
	global frame
	bridge = CvBridge()
	try:
		frame = bridge.imgmsg_to_cv2(ros_image, "bgr8") # converting the sensor_msgs/image type to IplImage 
	except CvBridgeError, e:
		print "NOT"
	global key
	if(key==1):
		return
	frame = np.array(frame, dtype=np.uint8) 
	#print frame[0,0]
	#cv2.imshow("Bottom",frame)
	#hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	bimage=frame
	centers = []
	lower_red=np.array([50,50,180])
	upper_red=np.array([160,160,255])
	mask=cv2.inRange(frame,lower_red,upper_red)
	global token
	global qua
	global published
	global published2
	contours , hier = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	for cnt in contours:
	    if 50<cv2.contourArea(cnt)<10000:
		cv2.drawContours(mask,[cnt],0,255,-1)
		moments=cv2.moments(cnt)
		centers.append(( int(moments['m01']/moments['m00']),int(moments['m10']/moments['m00'])))
	cv2.imshow("image",mask)
	global pos_found
	if(token>100 and published==0):
		dx=0
		dy=0
		if(len(centers)!=0):
			hover_f()
			x,y=centers[0]
			if x<180 and y<320:
				qua=1
				pubCommand.publish(Vector3(0,1,0),Vector3(0,0,0))
			elif x<=180 and y>=320:
				#2
				qua=2
				pubCommand.publish(Vector3(0,-1,0),Vector3(0,0,0))
			elif x>180 and y<320:
				#3
				qua=3
				pubCommand.publish(Vector3(0,1,0),Vector3(0,0,0))
			elif x>180 and y>320:
				qua=4
				pubCommand.publish(Vector3(0,-1,0),Vector3(0,0,0))
			published=1
	elif(published==1 and published2!=1 and len(centers)!=0):
                for i in range (315,325):
                    if i == centers[0][1]:
                        if qua==1:
                            pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
                        elif qua==2:
                            pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
                        elif qua==3:
                            pubCommand.publish(Vector3(-1,0,0),Vector3(0,0,0))
                        elif qua==4:
                            pubCommand.publish(Vector3(-1,0,0),Vector3(0,0,0))
                        published2=1
                        break
        elif(published2==1):
            k=0
	    for i in range(175,185):
		    for j in range(315,325):
			    if(mask[i,j]==255):
			            k=k+1
            if(k>5):
                        hover_f()
                        print "YES"
                        token=0
                        land()
        else :
		if( len(centers)!=0):
			#print token
			token=token+1
			#print centers
	
	if len(centers)!=0:
		print centers
	
if __name__ == '__main__': 
   try: 
       #takeoff()
	rospy.Subscriber("/ardrone/image_raw", Im, image_callback)
	rospy.Subscriber("/ardrone/bottom/image_raw", Im, image_bottom)
	rospy.Subscriber("/ardrone/navdata", Navdata, callback)
	#rospy.Subscriber('vslam/pose ', String, speechCb)
	init()
   except rospy.ROSInterruptException: 
       pass
