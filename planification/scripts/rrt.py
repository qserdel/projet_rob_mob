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
    """
    Modelise l'environnement du robot, contient la matrice d'occupation et les methodes concernant l'environnement
    """

    def __init__(self,map):
        self.mat = map


    @staticmethod
    def dist(a, b):
        """
        Renvoie la distance entre les points a et b
        """
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


    @staticmethod
    def random_action():
        """
        Renvoie une action orientee vers une cible aleatoire
        """
        cible = [random.uniform(0, w), random.uniform(0, h)]
        dist = Env.dist(state,cible)
        if dist == 0 :
            dist = 1
        norm = min(20,dist)
        return [norm/dist*(cible[0]-state[0]),norm/dist*(cible[1]-state[1])]


    @staticmethod
    def oriented_action(state,cible):
        """
        Renvoie une action orientee depuis state vers cible
        """
        dist = Env.dist(state,cible)
        if dist == 0 :
            dist = 1
        norm = min(40,dist)
        return [norm/dist*(cible[0]-state[0]),norm/dist*(cible[1]-state[1])]


    def step(self, state, action):
        """
        Renvoie le point d'arrivee lorsque l'on applique action depuis state
        """
        candidate = [state[0] + action[0], state[1] + action[1]]
        pt = self.intersect_discret(state[0],state[1],candidate[0],candidate[1])
        if pt:
            candidate = pt
        newstate = candidate
        if Env.dist(state, newstate) < 0.5 : #or Env.dist(state,newstate) > 100:  # Reject steps that are too small or too big
           return False
        else:
            return newstate

    def intersect_discret(self,x1,y1,x2,y2):
        """
        Discretise un segment et retourne son intersection avec l'environnement si elle existe, false sinon
        """
        norm = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        if(norm == 0):
            norm =1;
        dx = (x2-x1)/norm
        dy = (y2-y1)/norm
        i = 0
        xi = math.floor(x1)
        yi = math.floor(y1)
        if not self.mat[int(xi)][int(yi)] == 0 :
            return [xi,yi]

        while not (xi == math.floor(x2)) and (not (yi == math.floor(y2))):
            i+=1
            xi = math.floor(x1+i*dx)
            yi = math.floor(y1+i*dy)
            if not self.mat[int(xi)][int(yi)] == 0 :
                return [xi,yi]
        return False


class Tree:
    """
    Modelise un arbre de points de l'environnement
    """
    def __init__(self, init_state, parent=None):
        self.parent = parent
        self.state = init_state
        self.successors = []
        self.all_nodes = [self]


def rrt_expansion(t, env, action_type, cible):
    """
    Expansion d'un seul rrt avec une methode choisie
    """
    nearest_neighbor = t
    if(action_type == 'random'):
      sample = [random.uniform(0,w),random.uniform(0,h)]
      d = Env.dist(t.state, sample)
      for s in t.all_nodes:  # Naive way to get the nearest neighbor
        d_tmp = Env.dist(s.state, sample)
        if d_tmp < d:
            nearest_neighbor = s
            d = d_tmp
      action = env.oriented_action(nearest_neighbor.state,sample)
    else :
      sample = cible
      d = Env.dist(t.state, sample)
      for s in t.all_nodes:
          d_tmp = Env.dist(s.state, sample)
          if d_tmp < d:
              nearest_neighbor = s
              d = d_tmp
      action = env.oriented_action(nearest_neighbor.state,sample)
      #rospy.loginfo("distance min : %f\t--> 0",d)
    new_state = env.step(nearest_neighbor.state, action)
    if new_state:
        new_node = Tree(new_state, nearest_neighbor)
        nearest_neighbor.successors.append(new_node)
        t.all_nodes.append(new_node)
        # envoie du segment cree a l'affichage
        segment = ListePoints()
        segment.points.append(Point(nearest_neighbor.state[0],nearest_neighbor.state[1],0))
        segment.points.append(Point(new_state[0],new_state[1],0))
        segmentPub.publish(segment)


def rrt_connect(t,t2,env):
    """
    Cree deux rrt qui cherchent a se rejoindre avec une methode connect
    """
    i=0
    joint = False
    rrt_expansion(t, env, 'random',t2.state)
    rrt_expansion(t2, env, 'random',t.state)
    while not (joint):
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
    return node_t, node_t2


def find_path(node_t,node_t2):
    """
    Cree le chemin entre les 2 racines
    """
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


def simplify_path(path_tree, env):
    """
    Simplifie le chemin en reliant les noeuds qui n'ont pas d'obstacle entre eux
    """
    node = path_tree
    path_tree.all_nodes = [node]
    while len(node.successors) > 0 :
        node2 = node.successors[0]
        while len(node2.successors) > 0 :
            node2 = node2.successors[0]
            if not env.intersect_discret(node.state[0],node.state[1],node2.state[0],node2.state[1]):
                node.successors[0] = node2
        node = node.successors[0]
        path_tree.all_nodes.append(node)

def positionCallback(msg):
    """
    Callback du subscriber de l'odometrie
    """
    global pos
    pos = msg.pose.pose.position

def objectiveCallback(objective):
    """
    Callback du subscriber de l'objectif
    """
    obj = objective

def send_checkpoints(self):
    return checkpoints

def transcription_map_repere(pixel,map_origin,resolution):
    """
    Traduit une position de pixel en coordonnees repere
    """
    y = pixel[0]*resolution+map_origin.x
    x = pixel[1]*resolution+map_origin.y
    point = Point()
    point.x = x
    point.y = y
    #rospy.loginfo("pixel : [%d,%d] -> [%f,%f]",pixel[0],pixel[1],point.x,point.y)
    return point

def transcription_repere_map(point,map_origin,resolution):
    """
    Traduit des coordonnees repere en position de pixel
    """
    y = int((point.x-map_origin.x)/resolution)
    x = int((point.y-map_origin.y)/resolution)
    pixel = [x,y]
    #rospy.loginfo("point : [%f,%f] -> [%d,%d]",point.x,point.y,pixel[0],pixel[1])
    return pixel

# main
if __name__ == '__main__':
    rospy.init_node('planification',anonymous=False)
    global pos
    rospy.Subscriber("odom",Odometry,positionCallback)
    rospy.Subscriber("objective",Point,objectiveCallback)
    # objectif du rrt
    obj = Point()
    obj.x = -3.5
    obj.y = -0


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

        # creation de l'environnement
        env = Env(map)
        pos_pixel = transcription_repere_map(pos,map_origin,map_resolution)
        obj_pixel = transcription_repere_map(obj,map_origin,map_resolution)
        t = Tree(pos_pixel)
        t2 = Tree(obj_pixel)

        print("execution du rrt connect ...")
        # creation du publisher pour l'affichage des segments
        segmentPub = rospy.Publisher('segments_rrt',ListePoints,queue_size=10)
        node_t, node_t2 = rrt_connect(t,t2,env)
        rospy.loginfo("rrt connect termine !")

        path_tree = find_path(node_t,node_t2);
        rospy.loginfo("chemin trouve !")

        simplify_path(path_tree,env)
        rospy.loginfo("chemin simplifie !")

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
