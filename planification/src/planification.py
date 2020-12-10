import numpy as np
import sys
import random
import math
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import time
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Env:
    # A 2D environment with bounds [-1, 1] x [-1, 1]
    def __init__(self, walls=[]):
        self.walls = [([-1.0, -1.0], [1.0, -1.0]),
                      ([1.0, -1.0], [1.0, 1.0]),
                      ([1.0, 1.0], [-1.0, 1.0]),
                      ([-1.0, 1.0], [-1.0, -1.0])] + walls

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
        return [random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1)]

    @staticmethod
    def oriented_action(state,cible,d):
        norm = max(0.1,np.sqrt(np.square(cible[0]-state[0])+np.square(cible[1]-state[1])))
        return [0.1*(cible[0]-state[0])/norm,0.1*(cible[1]-state[1])/norm]


    def step(self, state, action, full=False):
        candidate = [state[0] + action[0], state[1] + action[1]]
        dist = np.infty
        for w in self.walls:  # Naive way to check for collisions
            pt = Env.intersect(state, candidate, w[0], w[1])
            if pt:
                candidate = pt
                return False
        if full:
            return candidate
        else:
            newstate = [state[0] + (candidate[0] - state[0]), state[1] + (candidate[1] - state[1])]
            if Env.dist(state, newstate) < 0.01:  # Reject steps that are too small
                return False
            else:
                return newstate

    def step_wall(self, state, action, full=False):
          candidate = [state[0] + action[0], state[1] + action[1]]
          dist = np.infty
          for w in self.walls:  # Naive way to check for collisions
              pt = Env.intersect(state, candidate, w[0], w[1])
              if pt:
                  candidate = pt
          if full:
              return candidate
          else:
              newstate = [state[0] + (candidate[0] - state[0]), state[1] + (candidate[1] - state[1])]
              if Env.dist(state, newstate) < 0.01:  # Reject steps that are too small
                  return False
              else:
                  return newstate


    def plotwalls(self, fig, ax):
        lines = []
        rgbs = []
        for w in self.walls:
            lines.append(w)
            rgbs.append((0, 0, 0, 1))
        ax.add_collection(mc.LineCollection(lines, colors=rgbs, linewidths=2))
        plt.xlim(-1, 1)
        plt.ylim(-1, 1)
        plt.axis('equal')


class Tree:
    def __init__(self, init_state, parent=None, root=True):
        self.parent = parent
        self.state = init_state
        self.successors = []
        self.root = root
        self.all_nodes = []
        self.distance_root = []

    def __all_edges(self):
        if not self.successors:
            return [], []
        else:
            lines = []
            rgbs = []
            for s in self.successors:
                lines.append((self.state, s.state))
                rgbs.append((1, 0, 0, 1))
                ladd, rgbadd = s.__all_edges()
                lines += ladd
                rgbs += rgbadd
            return lines, rgbs

    def plot(self, fig, ax):
        lines, rgbs = self.__all_edges()
        ax.add_collection(mc.LineCollection(lines, colors=rgbs, linewidths=1))


def random_walls(env, n):
    for i in range(n):
        start = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        progress = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        end = env.step_wall(start, progress, True)
        env.walls.append((start, end))

# expansion d'un seul rrt avec une methode choisie
def rrt_expansion(t, env, action_type, cible):
    nearest_neighbor = t
    if(action_type == 'random'):
      sample = [random.uniform(-1.0,1.0),random.uniform(-1.0,1.0)]
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

#cree deux rrt qui cherchent a se rejoindre avec une methode connect
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

#simplifie le chemin en reliant les noeuds qui n'ont pas d'obstacle entre eux
def simplify_path(path_tree, env):
  node = path_tree
  path_tree.all_nodes = [node]
  while len(node.successors) > 0 :
    node2 = node.successors[0]
    while len(node2.successors) > 0 :
      node2 = node2.successors[0]
      for wall in env.walls :
        collision = env.intersect(node.state,node2.state,wall[0],wall[1])
        if collision :
          break
      if not collision:
        node.successors[0] = node2
    node = node.successors[0]
    path_tree.all_nodes.append(node)

def positionCallback(position):
    pos = position

def mapCallback(occupancyMap):
    map = occupancyMap

def objectiveCallback(objective):
    obj = objective

# main

rospy.init_node('plannification',anonymous=False)

rospy.Subscriber("odom",Odometry,positionCallback)
rospy.subscriber("binary_map",OccupancyGrid,mapCallback)
rospy.subscriber("objective",Point,objectiveCallback)
trajectoryPub = rospy.Publisher('trajectoire',)

# position du robot


nbmurs = 10
env = Env()
random_walls(env, nbmurs)
random.seed(None)

t = Tree([-0.8, -0.8])
t2 = Tree([0.8, 0.8])

node_t, node_t2 = rrt_connect(t,t2,env)

fig, ax = plt.subplots()
t.plot(fig, ax)
t2.plot(fig, ax)
env.plotwalls(fig, ax)
plt.show()

path_tree = find_path(node_t,node_t2);

fig, ax = plt.subplots()
path_tree.plot(fig, ax)
env.plotwalls(fig, ax)
plt.show()

simplify_path(path_tree,env)

fig, ax = plt.subplots()
path_tree.plot(fig, ax)
env.plotwalls(fig, ax)
plt.show()
