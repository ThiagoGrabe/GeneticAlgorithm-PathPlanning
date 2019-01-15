#!/usr/bin/env python

import numpy as np
import math
import rospy
import tf
import os
import rospkg

from geometry_msgs.msg import Point, Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import *


import matplotlib.pyplot as plt

import path_07 as path

#declaracao de variaveis
laserMsg = None
odomMsg = None
goal = None
goals = []
goals_ = []
euler = None




def LaserCallback(msg):
	global laserMsg
	laserMsg = msg

def OdomCallback(msg):
	global odomMsg
	odomMsg = msg
	quaternion = (
		odomMsg.pose.pose.orientation.x,
		odomMsg.pose.pose.orientation.y,
		odomMsg.pose.pose.orientation.z,
		odomMsg.pose.pose.orientation.w
		)
	global euler
	euler = tf.transformations.euler_from_quaternion(quaternion)

def distance(msg,goal): #distancia euclidiana
	return math.sqrt(pow((goal.x - msg.pose.pose.position.x),2)+pow((goal.y - msg.pose.pose.position.y),2))

def atracao(v1,v2):

	return (1*(v1-v2))

def run():
	rospy.init_node('final_project', log_level=rospy.DEBUG, anonymous = True)
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size =10)
	rospy.Subscriber('/base_scan',LaserScan,LaserCallback)
	rospy.Subscriber('/odom', Odometry, OdomCallback)
	rate = rospy.Rate (5) # 1 hz
	cmd_vel = Twist()
	global goals
	global odomMsg
	
	global goal

	rospack = rospkg.RosPack()
	pwd = rospack.get_path("tp3")
	goals_ = path.getCurve(image_dir=os.path.join(pwd, "results"))
	
	goals = goals_[1]
	#print goals
	

	goal = Pose2D
	goal.x = goals[0]
	goal.y = goals[1]

	i = 0
	global euler
	fim = False
	minimo_local = False
	iterations = 10

	cwd = os.getcwd()
	rospy.loginfo("Init nodes ... ")
	rospy.loginfo("Current path:%s", cwd)

	while (not rospy.is_shutdown() and not fim): #enquanto nao rodo todos os goals e enquanto o mapa esta mudando
		#falha nos sensores
		# if (laserMsg == None) or (odomMsg == None) :
		# 	rate.sleep ()
			# continue
		##rospy.loginfo("Goal X:%s, Y:%s, Pos X:%s, Y:%s", goal.x, goal.y, odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y)
		##print 'Goal X',goal.x,'Goal Y', goal.y, 'Pos X', odomMsg.pose.pose.position.x, 'Pos Y', odomMsg.pose.pose.position.y

	        if distance(odomMsg,goal) < 0.0000000001 or minimo_local or iterations == 0: #chegando no goal, mudar goal de posicao
            		minimo_local = False
            		iterations = 5
            		if i is len(goals_) - 1:
                		fim = True
                		break
            		else:
            			goals = goals_[i+1]
				goal.x = goals[0]
				goal.y = goals[1]
				i += 1
				#print 'a'
		fax = atracao(goal.x,odomMsg.pose.pose.position.x)
		fay = atracao(goal.y,odomMsg.pose.pose.position.y)


		cmd_vel.linear.x = 1*fax
		cmd_vel.linear.y = 1*fay
		iterations -= 1
		pub.publish(cmd_vel)
		rate.sleep()

	#fim do loop
	cmd_vel.linear.x = 0.0
	cmd_vel.linear.y = 0.0
	pub.publish(cmd_vel)


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
