#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import rospy
import math

from geometry_msgs.msg import Twist
import roslib; roslib.load_manifest('pocketsphinx')
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata 
rospy.init_node('takeoff', anonymous=True)
pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=10)
pubLand = rospy.Publisher('/ardrone/land',Empty,queue_size=10)
pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
pre=0
def speechCb(msg):
	print msg
	
	if msg.data.find("takeoff") > -1:
	   pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
	   pubTakeoff.publish(Empty())
	   print msg
	elif msg.data.find("land") > -1:
	    while alt>2000 :
	    	pubCommand.publish(Twist(Vector3(0,0,-1),Vector3(0,0,0)))
	    	rospy.sleep(1)
	    pubLand.publish(Empty())
	    rospy.sleep(3)
	    print msg
	elif msg.data.find("stay") > -1:
	    pubCommand.publish(Vector3(0,0,0),Vector3(0,0,0))
	    print msg
	elif msg.data.find("go") > -1:
	    pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
	    print msg

	
	elif msg.data.find("down") > -1:
	    pubCommand.publish(Vector3(0,0,-1),Vector3(0,0,0))
	    print msg
	elif msg.data.find("right") > -1:
	    pubCommand.publish(Vector3(0,0,0),Vector3(0,0,-1))
	    rospy.sleep(2)
	    pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
	    print msg
	elif msg.data.find("left") > -1:
	    pubCommand.publish(Vector3(0,0,0),Vector3(0,0,1))
	    rospy.sleep(2)
	    pubCommand.publish(Vector3(1,0,0),Vector3(0,0,0))
	    print msg
	elif msg.data.find("rotate") > -1:
	    pubCommand.publish(Vector3(0,0,0),Vector3(0,0,1))
	    print msg
	elif msg.data.find("back") > -1:
	    pubCommand.publish(Vector3(-1,0,0),Vector3(0,0,0))
	    print msg
	elif msg.data.find("up") > -1:
	    pubCommand.publish(Vector3(0,0,1),Vector3(0,0,0))
	    print msg

def callback(navdata):
	global alt
	alt=navdata.altd
		
rospy.Subscriber('recognizer/output', String, speechCb)
rospy.Subscriber("/ardrone/navdata", Navdata, callback)

rospy.spin()


