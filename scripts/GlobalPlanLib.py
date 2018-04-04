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

import rospy
import math
import numpy as np
from Queue import PriorityQueue
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
	jps_planner = Astar(req.costmap,req.start,req.goal)
	rospy.loginfo('you are using Astar algorithm...')
	return jps_planner.get_path()

def init_costmap(costmap):
	width = costmap.info.width
	height = costmap.info.height
	resolution = costmap.info.resolution
	costmap_2d = np.reshape(costmap.data, (width,height))
	return width,height,resolution,costmap_2d


class Node:
	"""docstring for Node"""
	def __init__(self, parent, x, y, cost):
		self.parent = parent
		self.x = x
		self.y = y
		self.cost = cost


class Astar:
	"""docstring for Astar"""
	def __init__(self,costmap,start,goal):
		self.costmap = costmap
		self.m_w,self.m_h,self.m_r,self.costmap_2d = init_costmap(costmap)

		self.s_x = int(start.pose.position.x/self.m_r)
		self.s_y = int(start.pose.position.y/self.m_r)
		print self.s_x,self.s_y

		self.g_x = int(goal.pose.position.x/self.m_r)
		self.g_y = int(goal.pose.position.y/self.m_r)
		print self.g_x,self.g_y
		self.open = []
		self.close = []
		self.path = Path()

	
	def is_target(self, i):
		return i.x == self.g_x and i.y == self.g_y

	def make_path(self,p):
		while p:
			pose = PoseStamped()
			# pose.header.stamp = rospy.get_time()
			pose.pose.position.x = p.x * self.m_r
			pose.pose.position.y = p.y * self.m_r
			self.path.poses.append(pose)
			p = p.parent

	def make_plan(self):
		rospy.loginfo('start making plan...')
		p = Node(None, self.s_x, self.s_y, 0)
		while True:
			self.expand_eight(p)
			if not self.open:
				return
			#获取F值最小的节点
			idx, p = self.get_best()
			#找到路径，生成路径，返回
			if self.is_target(p):
				self.make_path(p)
				return
			#把此节点压入关闭列表，并从开放列表里删除
			self.close.append(p)
			del self.open[idx]



	def expand_eight(self,p):
		for diff in [[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0]]:
			new_x,new_y = p.x+diff[0],p.y+diff[1]
			if not self.isValid(new_x,new_y):
				continue
			node = Node(p, new_x, new_y, p.cost+self.dist(diff,new_x,new_y))
			if self.node_in_close(node):
				continue
			i = self.node_in_open(node)
			if i!=-1:
				if self.open[i].cost > node.cost:
					self.open[i].parent = p
					self.open[i].cost = node.cost
				continue
			self.open.append(node)

	def get_best(self):
		best = None
		bv = 100000000 #如果你修改的地图很大，可能需要修改这个值
		bi = -1
		for idx, i in enumerate(self.open):
			value = self.get_dist(i)#获取F值
			if value < bv:#比以前的更好，即F值更小
				best = i
				bv = value
				bi = idx
		return bi, best

	def get_dist(self, i):
		# F = G + H
		# G 为已经走过的路径长度， H为估计还要走多远
		# 这个公式就是A*算法的精华了。
		return i.cost + math.sqrt(
			(self.g_x-i.x)*(self.g_x-i.x)
			+ (self.g_y-i.y)*(self.g_y-i.y))*1.2*20


	def node_in_close(self, node):
		for i in self.close:
			if node.x == i.x and node.y == i.y:
				return True
		return False
		
	def node_in_open(self, node):
		for i, n in enumerate(self.open):
			if node.x == n.x and node.y == n.y:
				return i
		return -1


	def dist(self,diff,x,y):
		return self.costmap_2d[y][x]+1 if abs(diff[0])==abs(diff[1]) else (self.costmap_2d[y][x]+1)*1.414

	def heuristic_cost(self,x,y):
		return abs(x-self.g_x)+abs(y-self.g_y)

	def isValid(self,x,y):
		if x >= self.m_w or y >= self.m_h or x < 0 or y < 0:
			return False
		else:
			return self.costmap_2d[y][x] <50

	def get_path(self):
		self.make_plan()
		self.path.poses = list(reversed(self.path.poses))
		self.path.header.frame_id = 'map'
		print len(self.open)
		return self.path



class JPS:
	"""docstring for JPS"""
	def __init__(self,costmap,start,goal):
		self.m_d = costmap.data
		self.m_w = costmap.info.width
		self.m_h = costmap.info.height
		self.m_r = costmap.info.resolution
        self.m_s = self.m_w*self.m_h


		self.s_x,self.s_y = self.world2map(start.pose.position.x,start.pose.position.y)
		self.node_s = self.xy2node(self.s_x,self.s_y)
		# print self.s_x,self.s_y

		self.g_x,self.g_y = self.world2map(goal.pose.position.x,goal.pose.position.y)
		self.node_g = self.xy2node(self.g_x,self.g_y)
		# print self.g_x,self.g_y

		self.open = PriorityQueue()
		self.open.put((0,self.node_s))

		self.close = []
		self.checked = []
		self.path = Path()

	
	def is_target(self, i):
		return i = self.node_g

	def map2world(self,x,y):
		return x*self.m_r,y*self.m_r

	def world2map(self,x,y):
		return int(x/self.m_r),int(y/self.m_r)

	def xy2node(self,x,y):
		return x+y*self.m_w

	def node2xy(self,node):
		x = node%self.m_w
		y = int(node/self.m_w)
		return x,y

	def is_obstacle(self,node):
		return self.m_d[node] >=90

	def is_bound(self,node):
		return (node<self.m_w or node>self.m_s-self.m_w or (node+1)%self.m_w == 0)

    def illegal(self,node):
        return (self.is_bound(node) and self.is_obstacle(node))

    def calc_hcost(self,node):
    	x,y=node2xy(node)
    	return abs(x-self.g_x)+abs(y-self.g_y)

	def expand_horizonal(self,node,dx,dy):
		node = node+dx+dy*self.m_w
		if self.illegal(node):
			return
		if self.illegal(node+self.m_w*dx+dy):
			self.open.put((self.calc_hcost(node),node))
			return
		if self.illegal(node-self.m_w*dx-dy):
			self.open.put((self.calc_hcost(node),node))
			return
		self.expand_horizonal(node,dx,dy)


	def expand_diagonal(self,node,dx,dy):
		node = node+dy*self.m_w+dx
		if self.illegal(node):
			return
		if self.illegal(node-dx):
			self.open.put((self.calc_hcost(node),node))
			return
		if self.illegal(node-dy*self.m_w):
			self.open.put((self.calc_hcost(node),node))
			return
		self.expand_diagonal(node,dx,dy)

	def make_path(self,p):
		while p:
			pose = PoseStamped()
			# pose.header.stamp = rospy.get_time()
			pose.pose.position.x = p.x * self.m_r
			pose.pose.position.y = p.y * self.m_r
			self.path.poses.append(pose)
			p = p.parent

	def make_plan(self):
		rospy.loginfo('start making plan...')
		while not self.open.empty():
			p = self.open.get()
			for diff in [[0,1],[1,0],[0,-1],[-1,0]]:
				expand_horizonal(p[1],diff[0],diff[1])
			for diff in [[1,1],[1,-1],[-1,-1],[-1,1]]:
				expand_diagonal(p[1],diff[0],diff[1])
			
			if not self.open:
				return
			#获取F值最小的节点
			idx, p = self.get_best()
			#找到路径，生成路径，返回
			if self.is_target(p):
				self.make_path(p)
				return
			#把此节点压入关闭列表，并从开放列表里删除
			self.close.append(p)
			del self.open[idx]



	def expand_eight(self,p):
		for diff in [[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0]]:
			new_x,new_y = p.x+diff[0],p.y+diff[1]
			if not self.isValid(new_x,new_y):
				continue
			node = Node(p, new_x, new_y, p.cost+self.dist(diff,new_x,new_y))
			if self.node_in_close(node):
				continue
			i = self.node_in_open(node)
			if i!=-1:
				if self.open[i].cost > node.cost:
					self.open[i].parent = p
					self.open[i].cost = node.cost
				continue
			self.open.append(node)

	def get_best(self):
		best = None
		bv = 100000000 #如果你修改的地图很大，可能需要修改这个值
		bi = -1
		for idx, i in enumerate(self.open):
			value = self.get_dist(i)#获取F值
			if value < bv:#比以前的更好，即F值更小
				best = i
				bv = value
				bi = idx
		return bi, best

	def get_dist(self, i):
		# F = G + H
		# G 为已经走过的路径长度， H为估计还要走多远
		# 这个公式就是A*算法的精华了。
		return i.cost + math.sqrt(
			(self.g_x-i.x)*(self.g_x-i.x)
			+ (self.g_y-i.y)*(self.g_y-i.y))*1.2*20


	def node_in_close(self, node):
		for i in self.close:
			if node.x == i.x and node.y == i.y:
				return True
		return False
		
	def node_in_open(self, node):
		for i, n in enumerate(self.open):
			if node.x == n.x and node.y == n.y:
				return i
		return -1


	def dist(self,diff,x,y):
		return self.costmap_2d[y][x]+1 if abs(diff[0])==abs(diff[1]) else (self.costmap_2d[y][x]+1)*1.414

	def heuristic_cost(self,x,y):
		return abs(x-self.g_x)+abs(y-self.g_y)

	def isValid(self,x,y):
		if x >= self.m_w or y >= self.m_h or x < 0 or y < 0:
			return False
		else:
			return self.costmap_2d[y][x] <50

	def get_path(self):
		self.make_plan()
		self.path.poses = list(reversed(self.path.poses))
		self.path.header.frame_id = 'map'
		print len(self.open)
		return self.path
		






if __name__ == '__main__':
	global_plan_server()

	