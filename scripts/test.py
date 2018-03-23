#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
from nav_msgs.msg import Path,OccupancyGrid


def testCB(msg):
	width,height = msg.info.width,msg.info.height
	print width,height
	map_2d = np.reshape(msg.data,[width,height])
	print map_2d



if __name__ == '__main__':
	rospy.init_node('test')
	test_sub = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,testCB)
	rospy.spin()