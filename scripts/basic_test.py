#!/usr/bin/env python
#coding=utf-8

from Queue import PriorityQueue
import numpy as np

queue = PriorityQueue()
from_table = {'coord':1,'from':2,'rcost':3,'hcost':4}
queue.put((0,from_table))
queue.put((1,3))

print queue.get()