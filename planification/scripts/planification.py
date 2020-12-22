#!/usr/bin/env python2

import sys
import random
import math
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import time
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from mapping.srv import *
#from planification import listePoints

class Env:
    mat = np.zeros([205,205])
    # A 2D environment with bounds [0, 200] x [0, 200]
    def __init__(self, walls=[]):
        self.walls = [([6, 6], [200, 6]),
                      ([200, 6], [200, 200]),
                      ([200, 200], [6, 200]),
                      ([6, 200], [6, 6])] + walls

    @staticmethod
    def intersect(a, b, c, d):
        # If line segments ab and cd have a true intersection, return the intersection point. Otherwise, return False
        # a, b, c and d are 2D points of the form [x, y]
        x1, x2, x3, x4 = a[0], b[0], c[0], d[0]
        y1, y2, y3, y4 = a[1], b[1], c[1], d[1]
        denom = (x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3)
        if denom == 0:
            return False
        else:
            t = ((y3 - y4) * (x1 - x3) + (x4 - x3) * (y1 - y3)) / denom
            if t <= 0 or t >= 1:
                return False
            else:
                t = ((y1 - y2) * (x1 - x3) + (x2 - x1) * (y1 - y3)) / denom
                if t <= 0 or t >= 1:
                    return False
                else:
                    return [x3 + t * (x4 - x3), y3 + t * (y4 - y3)]

    @staticmethod
    def dist(a, b):
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    @staticmethod
    def random_action():
        return [random.uniform(-1, 1), random.uniform(-1, 1)]

    @staticmethod
    def oriented_action(state,cible,d):
        norm = max(1,np.sqrt(np.square(cible[0]-state[0])+np.square(cible[1]-state[1])))
        return [1*(cible[0]-state[0])/norm,1*(cible[1]-state[1])/norm]


    def step(self, state, action, full=False):
        candidate = [state[0] + action[0], state[1] + action[1]]
        dist = np.infty
        for w in self.walls:  # Naive way to check for collisions
            s1 = discretisation_segment(state[0],state[1],candidate[0],candidate[1])
            s2 = discretisation_segment(w[0][0],w[0][1],w[1][0],w[1][1])
            pt = self.intersect_discret(s1)
            if pt:
                candidate = pt
                return False
        if full:
            return candidate
        else:
            newstate = [state[0] + (candidate[0] - state[0]), state[1] + (candidate[1] - state[1])]
            if Env.dist(state, newstate) < 0.1:  # Reject steps that are too small
                return False
            else:
                return newstate


    def intersect_discret(self,s1): #retourne l'intersection du segments discretises avec l'environement si elle existe, False sinon
      for i in range(len(s1)):
        if self.mat[s1[i][0],len(self.mat[0])-s1[i][1]]>100:
          return [s1[i]]
      return False


class Tree:
    def __init__(self, init_state, parent=None, root=True):
        self.parent = parent
        self.state = init_state
        self.successors = []
        self.root = root
        self.all_nodes = []
        self.distance_root = []


# discretise un segment et retourne la liste des pixels du segment
def discretisation_segment(x1,y1,x2,y2):
  norm = (y2-y1)**2 + (x2-x1)**2
  dx = (x2-x1)/math.sqrt(norm)
  dy = (y2-y1)/math.sqrt(norm)
  i = 0
  xi = math.floor(x1)
  yi = math.floor(y1)
  pixels_segment = [[xi,yi]]

  while not (xi == math.floor(x2)) and (not (yi == math.floor(y2))):
    i+=1
    xi = math.floor(x1+i*dx)
    yi = math.floor(y1+i*dy)
    if(not pixels_segment[len(pixels_segment)-1]==[xi,yi]):
      pixels_segment.append([xi,yi])

  return pixels_segment

# expansion d'un seul rrt avec une methode choisie
def rrt_expansion(t, env, action_type, cible):
    nearest_neighbor = t
    if(action_type == 'random'):
      sample = [random.uniform(0,200),random.uniform(0,200)]
      d = Env.dist(t.state, sample)
      for s in t.all_nodes:  # Naive way to get the nearest neighbor
        d_tmp = Env.dist(s.state, sample)
        if d_tmp < d:
            nearest_neighbor = s
            d = d_tmp
      action = env.oriented_action(nearest_neighbor.state,sample,d)
    else :
      sample = cible
      d = Env.dist(t.state, sample)
      for s in t.all_nodes:
          d_tmp = Env.dist(s.state, sample)
          if d_tmp < d:
              nearest_neighbor = s
              d = d_tmp
      action = env.oriented_action(nearest_neighbor.state,sample,d)
    new_state = env.step(nearest_neighbor.state, action)
    if new_state:
        new_node = Tree(new_state, nearest_neighbor, False)
        nearest_neighbor.successors.append(new_node)
        t.all_nodes.append(new_node)
        t.distance_root.append(Env.dist(new_state,t.state))

# cree deux rrt qui cherchent a se rejoindre avec une methode connect
def rrt_connect(t,t2,env):
  i=0
  joint = False
  rrt_expansion(t, env, 'random',t2.state)
  rrt_expansion(t2, env, 'random',t.state)
  while not joint:
      if(i%2 == 0):
        cible = t.all_nodes[len(t.all_nodes)-1]
        rrt_expansion(t, env, 'random',cible.state)
        rrt_expansion(t2, env, 'oriented',cible.state)
      else :
        cible = t2.all_nodes[len(t2.all_nodes)-1]
        rrt_expansion(t, env, 'oriented',cible.state)
        rrt_expansion(t2, env, 'random',cible.state)
      if(env.dist(t.all_nodes[len(t.all_nodes)-1].state,t2.all_nodes[len(t2.all_nodes)-2].state)==0):
        joint = True
        node_t = t.all_nodes[len(t.all_nodes)-1]
        node_t2 = t2.all_nodes[len(t2.all_nodes)-2]
      if(env.dist(t2.all_nodes[len(t2.all_nodes)-1].state,t.all_nodes[len(t.all_nodes)-2].state)==0):
        joint = True
        node_t = t.all_nodes[len(t.all_nodes)-2]
        node_t2 = t2.all_nodes[len(t2.all_nodes)-1]
      i+=1
  return node_t, node_t2

# Cree le chemin entre les 2 racines
def find_path(node_t,node_t2):

  path_t = [node_t]
  while(node_t.parent!=None):
    node_t.parent.successors = [node_t]
    node_t = node_t.parent
    path_t.append(node_t)
  node_t.all_nodes = path_t

  path_t2 = [node_t2]
  while(node_t2.parent!=None):
    node_t2.parent.successors = [node_t2]
    node_t2 = node_t2.parent
    path_t2.append(node_t2)

  path_t[0].successors=[path_t2[1]]

  for i in range(1,len(path_t2)-1):
    path_t2[i].successors = [path_t2[i+1]]

  path_t2[len(path_t2)-1].successors = []

  node_t.all_nodes.append(path_t2)
  return node_t

# simplifie le chemin en reliant les noeuds qui n'ont pas d'obstacle entre eux
def simplify_path(path_tree, env):
  node = path_tree
  path_tree.all_nodes = [node]
  while len(node.successors) > 0 :
    node2 = node.successors[0]
    while len(node2.successors) > 0 :
      node2 = node2.successors[0]
      s1 = discretisation_segment(node.state[0],node.state[1],node2.state[0],node2.state[1])

      if not env.intersect_discret(s1):
        node.successors[0] = node2
    node = node.successors[0]
    path_tree.all_nodes.append(node)

def positionCallback(msg):
    pos = msg.pose.pose

def objectiveCallback(objective):
    obj = objective

# main

rospy.init_node('plannification',anonymous=False)

rospy.Subscriber("odom",Odometry,positionCallback)

# service de recuperation de la matrice map
rospy.wait_for_service('binary_map')
try:
    binary_map = rospy.ServiceProxy('binary_map', BinaryMap)
    map = binary_map().map
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

rospy.Subscriber("objective",Point,objectiveCallback)


env = Env()
env.mat = map
t = Tree([pos.position.x,pos.position.y])
t2 = Tree([obj.x,obj.y])

node_t, node_t2 = rrt_connect(t,t2,env)

path_tree = find_path(node_t,node_t2);

simplify_path(path_tree,env)


while not rospy.is_shutdown():

    trajectoryPub = rospy.Publisher('checkpoints',Point,queue_size = 10)
