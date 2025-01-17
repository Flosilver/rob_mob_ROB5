import sys
import random
import math
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import time
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
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
    def random_action(state):
        """
        Renvoie une action orientee vers une cible aleatoire
        """
        cible = [int(random.uniform(0, w-1)), int(random.uniform(0, h-1))]
        dist = Env.dist(state,cible)
        if dist == 0 :
            dist = 1
        norm = min(100,dist)
        return [int(norm/dist*(cible[0]-state[0])),int(norm/dist*(cible[1]-state[1]))]


    @staticmethod
    def oriented_action(state,cible):
        """
        Renvoie une action orientee depuis state vers cible
        """
        dist = Env.dist(state,cible)
        if dist == 0 :
            dist = 1
        norm = min(100,dist)
        return [int(norm/dist*(cible[0]-state[0])),int(norm/dist*(cible[1]-state[1]))]


    def step(self, state, action):
        """
        Renvoie le point d'arrivee lorsque l'on applique action depuis state
        """
        candidate = [state[0] + action[0], state[1] + action[1]]
        pt = self.intersect_discret(state[0],state[1],candidate[0],candidate[1])
        if pt:
            candidate = pt
        newstate = candidate
        if Env.dist(state, newstate) < 2 :
            return False
        return newstate

    def intersect_discret(self,x1,y1,x2,y2):
        """
        Discretise un segment et retourne son intersection avec l'environnement si elle existe, false sinon
        """
        norm = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        if(norm == 0):
            return False
        dx = (x2-x1)/norm
        dy = (y2-y1)/norm
        i = 0
        xi = x1
        yi = y1
        if not self.mat[int(xi)][int(yi)] == 0 :
            return [xi,yi]
        imax = int(max(abs(x2-x1),abs(y2-y1)))
        for i in range(1,imax):
            xi = x1+i*dx
            yi = y1+i*dy
            if not self.mat[int(math.floor(xi))][int(math.floor(yi))] == 0 :
                return [int(xi-dx),int(yi-dy)]
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


def rrt_expansion(t, env, action_type, cible,segmentPub):
    """
    Expansion d'un seul rrt avec une methode choisie
    """
    nodes_libres=[]
    nearest_neighbor = t
    if(action_type == 'random'):
        sample = [int(random.uniform(0, len(env.mat[0])-1)), int(random.uniform(0, len(env.mat)-1))]
        while not (env.mat[sample[0]][sample[1]]==0):
            sample = [int(random.uniform(0, len(env.mat[0])-1)), int(random.uniform(0, len(env.mat)-1))]
        d = Env.dist(t.state, sample)
        for s in t.all_nodes:
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
            if not env.intersect_discret(s.state[0],s.state[1],sample[0],sample[1]) :
                nodes_libres.append(s)
            if d_tmp < d :
                nearest_neighbor = s
                d = d_tmp
        if not nodes_libres==[]:
            d_tmp = Env.dist(nodes_libres[0].state,sample)
        for s in nodes_libres:
            if Env.dist(s.state,sample) <= d_tmp :
                d_tmp = Env.dist(s.state,sample)
                nearest_neighbor = s
                d = d_tmp
        action = env.oriented_action(nearest_neighbor.state,sample)
    new_state = env.step(nearest_neighbor.state,action)
    if new_state:
        new_node = Tree(new_state, nearest_neighbor)
        nearest_neighbor.successors.append(new_node)
        t.all_nodes.append(new_node)
        # envoie du segment cree a l'affichage
        segment = ListePoints()
        segment.points.append(Point(nearest_neighbor.state[0],nearest_neighbor.state[1],0))
        segment.points.append(Point(new_state[0],new_state[1],0))
        segmentPub.publish(segment)
        #rospy.sleep(0.1)


def rrt_connect(t,t2,env,segmentPub):
    """
    Cree deux rrt qui cherchent a se rejoindre avec une methode connect
    """
    i=0
    joint = False
    rrt_expansion(t, env, 'random',t2.state,segmentPub)
    rrt_expansion(t2, env, 'random',t.state,segmentPub)
    while not (joint):
        if(i%2 == 0):
            rrt_expansion(t, env, 'random',None,segmentPub)
            cible = t.all_nodes[len(t.all_nodes)-1]
            rrt_expansion(t2, env, 'oriented',cible.state,segmentPub)
        else :
            rrt_expansion(t2, env, 'random',None,segmentPub)
            cible = t2.all_nodes[len(t2.all_nodes)-1]
            rrt_expansion(t, env, 'oriented',cible.state,segmentPub) 
        if(env.dist(t.all_nodes[len(t.all_nodes)-1].state,t2.all_nodes[len(t2.all_nodes)-2].state)<=2):
            joint = True
            node_t = t.all_nodes[len(t.all_nodes)-1]
            node_t2 = t2.all_nodes[len(t2.all_nodes)-2]
        if(env.dist(t2.all_nodes[len(t2.all_nodes)-1].state,t.all_nodes[len(t.all_nodes)-2].state)<=2):
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

class Pos_caller():
    def __init__(self):
        self.pos = None
    def positionCallback(self,msg):
        """
        Callback du subscriber de l'odometrie
        """
        #self.pos = Point(msg.pose.pose.position.y,msg.pose.pose.position.x,0)
        self.pos = msg.pose.pose.position

class Obj_caller():
    def __init__(self):
        self.obj = None

    def objectiveCallback(self,msg):
        """
        Callback du subscriber de l'objectif
        """
        #self.obj = Point(msg.pose.position.y,msg.pose.position.x,0)
        self.obj = msg.pose.position
        print(self.obj)


def transcription_map_repere(pixel,map_origin,resolution):
    """
    Traduit une position de pixel en coordonnees repere
    """
    x = (pixel[0]+0.5)*resolution+map_origin.x
    y = (pixel[1]+0.5)*resolution+map_origin.y
    point = Point(x,y,0)
    return point

def transcription_repere_map(point,map_origin,resolution):
    """
    Traduit des coordonnees repere en position de pixel
    """
    x = int((point.x-map_origin.x)/resolution)
    y = int((point.y-map_origin.y)/resolution)
    pixel = [x,y]
    return pixel