#!/usr/bin/env python 
#http://docs.ros.org/jade/api/mavros_msgs/html/srv/CommandBool.html
#http://docs.ros.org/jade/api/mavros_msgs/html/srv/SetMode.html
#source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_lpe && export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) && export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
import rospy
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import Vector3
from math import sin,cos,sqrt
import time
#adding a subscriber
def local_pos_callback(msg):
	global crnt_x,crnt_y,crnt_z
	crnt_x=msg.pose.position.x
	crnt_y=msg.pose.position.y
	crnt_z=msg.pose.position.z
def dis_call(data):
	global landing_y
	landing_y=data.x
def cam_callback(msg):
	global cam_x,cam_y
	cam_x=msg.x
	cam_y=msg.y
def distance_cal(x,y,z):
	diff_x=x-crnt_x
	diff_y=y-crnt_y
	diff_z=z-crnt_z
	error=sqrt(diff_x**2+diff_y**2+diff_z**2)
#	rospy.loginfo(error)
	return(error)
def semi_stage():
		while(15<(time.time()-start)<=20):
			typ.pose.position.x=crnt_x
			typ.pose.position.y=crnt_y
			typ.pose.position.z=0
			while (distance_cal(typ.pose.position.x,typ.pose.position.y,0)>thresh):
				pub.publish(typ)
				rate.sleep()	
def arm():
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		arming=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
		arming(True)
	except rospy.ServiceException, e:
		print "Service arm call failed: %s"%e
def disarm():
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		arming=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
		arming(False)
	except rospy.ServiceException, e:
		print "Service arm call failed: %s"%e	
def cam_callback(data):
	global cam_x
	global cam_y
	cam_x=data.x
	cam_y=data.y

def stage1():
	print("stage1")
	while((time.time()-start)<=15):
		typ.pose.position.x=2
		typ.pose.position.y=2
		typ.pose.position.z=2
		while (distance_cal(typ.pose.position.x,typ.pose.position.y,typ.pose.position.z)>thresh):
			pub.publish(typ)
			rate.sleep()
	stage2()
def stage3():
	while (time.time()-start<=35):
		typ.pose.position.z=2
		if(-100<cam_y<=0 and -100<cam_x<=400):
			typ.pose.position.x=crnt_x+0.3
			typ.pose.position.y=crnt_y+0.3
		elif(-100<cam_y<=0 and 400<cam_x<=900):
			typ.pose.position.x=crnt_x+0.3
			typ.pose.position.y=crnt_y-0.3
		elif(0<cam_y<=100 and -100<cam_x<=400):
			typ.pose.position.x=crnt_x+0.3
			typ.pose.position.y=crnt_y+0.7
		elif(0<cam_y<=100 and 400<cam_x<=900):
			typ.pose.position.x=crnt_x+0.7
			typ.pose.position.y=crnt_y-0.7
		elif(100<cam_y<=200 and -100<cam_x<=400):
			typ.pose.position.y=crnt_y+0.7
			typ.pose.position.x=crnt_x+0.7
		elif(100<cam_y<=200 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.7
			typ.pose.position.x=crnt_x+0.7
		elif(100<cam_y<=200 and -100<cam_x<=400):
			typ.pose.position.x=crnt_x+0.7
			typ.pose.position.y=crnt_y+0.7
		elif(100<cam_y<=200 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.7
			typ.pose.position.z=crnt_x+0.7
		elif(200<cam_y<=300 and -100<cam_x<=400):
			typ.pose.position.y=crnt_x+0.09
			typ.pose.position.x=crnt_y+0.3
		elif(200<cam_y<=300 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.5
			typ.pose.position.x=crnt_x+0.7
		elif(300<cam_y<=400 and -100<cam_x<=400):
			typ.pose.position.y=crnt_y+0.3
			typ.pose.position.x=crnt_x+0.3
		elif(300<cam_y<=400 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.3
			typ.pose.position.x=crnt_x+0.7
		elif(400<cam_y<=500 and -100<cam_x<=400):
			typ.pose.position.y=crnt_y+0.7
			typ.pose.position.x=crnt_x-0.7
		elif(400<cam_y<=500 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.5
			typ.pose.position.x=crnt_x-0.3
		elif(500<cam_y<=900 and -100<cam_x<400):
			typ.pose.position.y=crnt_y+0.3
			typ.pose.position.x=crnt_x-0.7
		elif(500<cam_y<=900 and 400<cam_x<=900):
			typ.pose.position.y=crnt_y-0.3
			typ.pose.position.x=crnt_x+0.7
		else:
			typ.pose.position.y=crnt_y
			typ.pose.position.x=crnt_x
		pub.publish(typ)
		rate2.sleep()
		print("stage3")
	stage4()


def stage2():
	count=0
	print("stage2")
	while((time.time()-start<=35) and (cam_x==cam_y==0)):
		angle=0.2 *count*0.1
		typ.pose.position.x=2+3*sin(angle)
		typ.pose.position.y=2+3*cos(angle)
		typ.pose.position.z=2
		pub.publish(typ)
		count+=1
		rate.sleep()
	stage3()
def stage4():
	print("in stage 4")
	while((time.time()-start)<=40):
		typ.pose.position.x=landing_y+crnt_x
		typ.pose.position.z=2
		while (crnt_z>0.1):
			pub.publish(typ)
			typ.pose.position.z=crnt_z-0.5
			rate.sleep()

	stage1()
#	disarm()
#	print("finished")
def Test():
	i=0
	while not rospy.is_shutdown():
		vell.twist.linear.x=0.1
		vell.twist.linear.y=0
		if(crnt_z<=1.5):
			vell.twist.linear.z=0.5
			velopub.publish(vell)
		elif(crnt_z<=1.7):
			vell.twist.linear.z=0.3
			velopub.publish(vell)	
		elif(crnt_z<=1.9):
			vell.twist.linear.z=0.1
			velopub.publish(vell)		
		else:
			break		
		#i+=0.00035
		rate.sleep()
	while not rospy.is_shutdown():
		vell.twist.linear.z=0
		vell.twist.linear.x=-0.5
		vell.twist.linear.y=0.7
		velopub.publish(vell)
def main():
	arm()
	for i in range(100):
		rospy.loginfo(i)
		pub.publish(typ)
		rate.sleep()
	off(0,"OFFBOARD")
#	stage1()
	Test()
	rospy.loginfo(crnt_x)

if ("__main__"==__name__):
	rospy.init_node("main_mission")
	gx=gy=0
	start=time.time()
	crnt_x=crnt_y=crnt_z=0
	thresh=0.08
	cam_x=0
	cam_y=0
	rate=rospy.Rate(30)
	landing_y=0
	dist_sub=rospy.Subscriber("/dist",Vector3,dis_call)
	cam_sub=rospy.Subscriber("/mid",Vector3,cam_callback)
	pose_sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,local_pos_callback)
	arming=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
	typ=PoseStamped()
	vell=TwistStamped()
	off=rospy.ServiceProxy("/mavros/set_mode",SetMode)
	pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
	rate2=rospy.Rate(3)
	velopub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
	
#	cam subscribers to be added
	main()
