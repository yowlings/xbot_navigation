#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
from xbot_navigation.srv import GlobalPlan
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Path,OccupancyGrid

class test:
	"""docstring for test"""
	def __init__(self):
		rospy.init_node('test')
		self.current_pose = PoseStamped()
		self.cmap = OccupancyGrid()	
		rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,self.costmapCB)
		rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.current_poseCB)
		rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.testCB)
		self.test_pub = rospy.Publisher('/test/astar_path',Path,queue_size=10)
		rospy.spin()
		

	def testCB(self,msg):
		print 'new sequence---------------------------'
		print 'costmap informations:'
		print self.cmap.info
		print '---------------------------------------'
		print 'the start point'
		print self.current_pose,msg
		print '---------------------------------------'
		print 'the goal point'
		print msg
		rospy.wait_for_service('global_plan')
		try:
			global_plan = rospy.ServiceProxy('global_plan',GlobalPlan)
			resp1 = global_plan(self.cmap,self.current_pose,msg)
			self.test_pub.publish(resp1.path)
		except Exception, e:
			print "Service call failed: %s"%e

	def current_poseCB(self,msg):
		self.current_pose.pose = msg.pose.pose

	def costmapCB(self,msg):
		self.cmap = msg




if __name__ == '__main__':
	test()