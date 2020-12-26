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
from planification.msg import ListePoints
from planification.srv import Checkpoints

class Env:

    def __init__(self,map):
        self.mat = map

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
        s1 = discretisation_segment(state[0],state[1],candidate[0],candidate[1])
        pt = self.intersect_discret(s1)
        if pt:
            candidate = pt
            return False
        if full:
            return candidate
        else:
            newstate = [state[0] + (candidate[0] - state[0]), state[1] + (candidate[1] - state[1])]
            if Env.dist(state, newstate) < 0.5 or Env.dist(state,newstate) > 10:  # Reject steps that are too small or too big
                return False
            else:
                return newstate


    def intersect_discret(self,s1): #retourne l'intersection du segments discretises avec l'environement si elle existe, False sinon
      for i in range(len(s1)):
        if not self.mat[int(s1[i][0]),int(s1[i][1])] == 0 :
          return [s1[i]]
      return False


class Tree:
    def __init__(self, init_state, parent=None, root=True):
        self.parent = parent
        self.state = init_state
        self.successors = []
        self.root = root
        self.all_nodes = [self]
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
      sample = [random.uniform(0,w),random.uniform(0,h)]
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
      rospy.loginfo("distance min : %f\t--> 0",d)
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
  while not (joint or i==1000):
      if(i%2 == 0):
        cible = t.all_nodes[len(t.all_nodes)-1]
        rrt_expansion(t2, env, 'oriented',cible.state)
        rrt_expansion(t, env, 'random',cible.state)
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
  if(i==1000):
    node_t = t.all_nodes[len(t.all_nodes)-1]
    node_t2 = t2.all_nodes[len(t2.all_nodes)-2]
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

  #path_t[0].successors=[path_t2[1]]
  path_t[0].successors=[path_t2[0]]

  for i in range(1,len(path_t2)-1):
    #path_t2[i].successors = [path_t2[i+1]]
    path_t2[i].successors = [path_t2[i]]

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
    pos = msg.pose.pose.position

def objectiveCallback(objective):
    obj = objective

def send_checkpoints(self):
    return checkpoints

def transcription_map_repere(pixel,map_origin,resolution):
    x = pixel[0]*resolution+map_origin.x
    y = pixel[1]*resolution+map_origin.y
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    rospy.loginfo("pixel : [%d,%d] -> [%f,%f]",pixel[0],pixel[1],point.x,point.y)
    return point

def transcription_repere_map(point,map_origin,resolution):
    x = int((point.x-map_origin.x)/resolution)
    y = int((point.y-map_origin.y)/resolution)
    pixel = [x,y]
    rospy.loginfo("point : [%f,%f] -> [%d,%d]",point.x,point.y,pixel[0],pixel[1])
    return pixel

# main
if __name__ == '__main__':
    rospy.init_node('planification',anonymous=False)
    pos = Point()
    rospy.Subscriber("odom",Odometry,positionCallback)
    rospy.Subscriber("objective",Point,objectiveCallback)
    obj = Point()
    obj.x = -10
    obj.y = -2
    obj.x = 10
    obj.y = 10

    while not rospy.is_shutdown():
        rospy.loginfo("on entre dans la boucle du rrt")
        # service de recuperation de la matrice map
        rospy.wait_for_service('binary_map')
        rospy.loginfo("binary map recuperee")
        try:
            binary_map = rospy.ServiceProxy('binary_map', BinaryMap)
            w = binary_map().map.info.width
            h = binary_map().map.info.height
            rospy.loginfo("w = %d h = %d",w,h)
            map = np.zeros([h,w])
            bm = binary_map().map.data
            map_origin = binary_map().map.info.origin.position
            map_resolution = binary_map().map.info.resolution
            rospy.loginfo("origine : [%f,%f]",map_origin.x,map_origin.y)
            rospy.loginfo("resolution : %f",map_resolution)
            for i in range (h):
                if i%100==0:
                    rospy.loginfo("transcription binary_map : %d/%d",i,h)
                map[i] = bm[i*h:i*h+w]
            print("binary map ok")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        env = Env(map)
        pos_pixel = transcription_repere_map(pos,map_origin,map_resolution)
        obj_pixel = transcription_repere_map(obj,map_origin,map_resolution)
        t = Tree(pos_pixel)
        t2 = Tree(obj_pixel)

        print("lancement du rrt connect")
        node_t, node_t2 = rrt_connect(t,t2,env)
        rospy.loginfo("rrt connect fait")

        path_tree = find_path(node_t,node_t2);
        rospy.loginfo("chemin trouve")

        simplify_path(path_tree,env)
        rospy.loginfo("chemin simplifie")

        checkpoints = ListePoints()
        node = path_tree
        while node.successors != []:
            pixel = [node.state[0],node.state[1]]
            point = transcription_map_repere(pixel,map_origin,map_resolution)
            rospy.loginfo("checkpoint ajoute : [ %f , %f ]",point.x,point.y)
            checkpoints.points.append(point)
            node = node.successors[0]
        pixel = [node.state[0],node.state[1]]
        point = transcription_map_repere(pixel,map_origin,map_resolution)
        rospy.loginfo("checkpoint ajoute : [ %f , %f ]",point.x,point.y)
        checkpoints.points.append(point)
        rospy.loginfo("liste des checkpoints creee")
        checkpointService = rospy.Service('checkpoints',Checkpoints,send_checkpoints)
        rospy.loginfo("liste des checkpoints envoyee !")
        rospy.spin()
