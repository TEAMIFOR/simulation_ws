#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
def cam_callback(data):
	global xP,yP
	xP=data.x
	yP=data.y
def local_pos_callback(data):
	global h
	h=data.pose.position.z

def main():
	while not rospy.is_shutdown():
		img=Vector3()
		 
		alpha = 1.076767572
		 
		theta_v = (50*3.14)/180 
		 
		theta2 = (theta_v - alpha + math.acos((math.cos(theta_v))*math.cos(alpha)))/2 
		 
		 
		z = (2*h) / (1 / math.tan(theta2)) + (1 / math.tan(alpha)) 
		 
		x = h/math.tan(alpha+theta2) 
		 
		T = (z*(800 - yP))/800
		     
		t = T+x
		dis=Vector3()
		dis.x=t
		
		dist_pub.publish(dis)
		rospy.loginfo(t)
		rate.sleep()

	
if(__name__=="__main__"):
	xP=0
	yP=0
	x=0
	y=0
	h=0
	rospy.init_node("distance_cal")
	dist_pub=rospy.Publisher("/dist",Vector3,queue_size=10)
	pose_sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,local_pos_callback)
	cam_sub=rospy.Subscriber("/mid",Vector3,cam_callback)
	rate=rospy.Rate(3)
	main()