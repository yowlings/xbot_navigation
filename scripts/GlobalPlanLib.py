#!/usr/bin/env python
#coding=utf-8
"""
ROS global plan lib, including:
1.Breadth-first search
2.Depth-first search
3.Dijkstra search
4.A* search
5.Jump Point Search(JPS)


"""
from Queue import PriorityQueue
import rospy
import numpy as np
from xbot_navigation.srv import GlobalPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,OccupancyGrid

def global_plan_server():
	rospy.init_node('global_plan_server')
	rospy.loginfo('global plan server node initialized...')
	server = rospy.Service('global_plan', GlobalPlan, handle_global_plan)
	rospy.loginfo('global plan server launched...')
	rospy.spin()

def handle_global_plan(req):
	# if req.algorithm = 'astar':
	# 	planning = Astar(req.costmap,req.start,req.goal)
	# 	return GlobalPlanResponse(planning.get_path())
	# elif req.algorithm = 'dijkstra':
	# 	planning = Dijkstra(req.costmap,req.start,req.goal)
	# 	return GlobalPlanResponse(planning.get_path())
	# elif req.algorithm = 'breadth-first':
	# 	planning = Breadth_first(req.costmap,req.start,req.goal)
	# 	return GlobalPlanResponse(planning.get_path())
	# elif req.algorithm = 'depth-first':
	# 	planning = Depth_first(req.costmap,req.start,req.goal)
	# 	return GlobalPlanResponse(planning.get_path())
	# else:
	jps_planner = JPS(req.costmap,req.start,req.goal)
	rospy.loginfo('you are using JPS algorithm...')
	return GlobalPlanResponse(jps_planner.get_path())





class JPS():
	"""docstring for GlobalPlan"""
	def __init__(self,costmap,start,goal):
		self.costmap = costmap
		self.start = start
		self.goal = goal
		self.open_list = PriorityQueue()

		





	def cost_map_init(self):
		self.costmap_resolution = self.costmap.info.resolution
		self.costmap_size = [self.costmap.info.width,self.costmap.height]
		slef.costmap_2d = np.reshape(self.costmap.data,size)
		self.start_in_2dmap = [int(self.start.pose.position.x/self.costmap_resolution),int(self.start.pose.position.y/self.costmap_resolution)]
		self.goal_in_2dmap = [int(self.goal.pose.position.x/self.costmap_resolution),int(self.goal.pose.position.y/self.costmap_resolution)]
		cur_cost = 0 + abs(self.start_in_2dmap[0]-self.goal_in_2dmap[0])+abs(self.start_in_2dmap[1]-self.goal_in_2dmap[1])
		self.open_list.put((cur_cost,self.start_in_2dmap))


	def make_plan(self):
		rospy.loginfo('start making plan...')


	def get_path(self):
		self.cost_map_init()
		path = self.make_plan()
		return path.inverse()


class Astar():
	"""docstring for GlobalPlan"""
	def __init__(self,costmap,start,goal):
		self.costmap = costmap
		self.start = start
		self.goal = goal
		self.open_list = PriorityQueue()

		





	def cost_map_init(self):
		self.costmap_resolution = self.costmap.info.resolution
		self.costmap_size = [self.costmap.info.width,self.costmap.height]
		slef.costmap_2d = np.reshape(self.costmap.data,size)
		self.start_in_2dmap = [int(self.start.pose.position.x/self.costmap_resolution),int(self.start.pose.position.y/self.costmap_resolution)]
		self.goal_in_2dmap = [int(self.goal.pose.position.x/self.costmap_resolution),int(self.goal.pose.position.y/self.costmap_resolution)]
		rcost = 0
		hcost = heuristic_cost(self.start_in_2dmap)
		self.open_list.put((rcost+hcost,self.start_in_2dmap))
		
		self.ischecked = np.zeros(self.costmap_size,dtype=bool)
		self.rcost_table = np.zeros(self.costmap_size)
		self.from_table = {}

		self.rcost_table[self.start_in_2dmap[0]][self.start_in_2dmap[1]] = rcost
		self.ischecked[self.start_in_2dmap[0]][self.start_in_2dmap[1]] = True
		


	def make_plan(self):
		rospy.loginfo('start making plan...')
		while not self.open_list.empty():
			cur = self.open_list.get()
			self.expand(cur)




	def expand(self,item):
		cur_coord = item[1]['coord']
		cur_rcost = item[1]['rcost']
		cur_cost = item[0]

		for diff in [[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0]]:
			neighbor = cur_coord + diff
			if self.isValid(neighbor):
				rcost = cur_rcost+dist(diff)
				hcost = heuristic_cost(neighbor)
				if not self.ischecked[neighbor[0]][neighbor[1]]:
					self.open_list.put((rcost+hcost,{'coord':neighbor,'from':cur_coord,'rcost':rcost,'hcost':hcost}))
				elif self.rcost_table[neighbor[0]][neighbor[1]]>rcost:
					self.open
	
	def dist(self,diff):
		return 1.414 if abs(diff[0])==abs(diff[1]) else 1

	def heuristic_cost(self,coord):
		return abs(coord[0]-self.goal_in_2dmap[0])+abs(coord[1]-self.goal_in_2dmap[1])

	def isValid(self):
		pass

	def get_path(self):
		self.cost_map_init()
		path = self.make_plan()
		return path.inverse()


if __name__ == '__main__':
	global_plan_server()

	