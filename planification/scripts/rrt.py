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
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from mapping.srv import *
from planification.msg import ListePoints 
from planification.srv import Checkpoints
import util_rrt

def send_checkpoints(self):
    return checkpoints

# main alternatif pour tester la commande en environnement libre
if __name__ == '__main_test_commande__':

    checkpoints = ListePoints()

    checkpoints.points.append(Point(1,1,0))
    checkpoints.points.append(Point(1,2,0))
    checkpoints.points.append(Point(2,2,0))
    checkpoints.points.append(Point(2,3,0))
    checkpoints.points.append(Point(-1,-1,0))
    checkpoints.points.append(Point(-10,-10,0))
    checkpoints.points.append(Point(0,0,0))

    checkpointService = rospy.Service('checkpoints',Checkpoints,send_checkpoints)
    rospy.loginfo("liste des checkpoints envoyee !")   
    rospy.spin()

# main
if __name__ == '__main__':
    rospy.init_node('planification',anonymous=False)
    # topic de la position du robot
    pos_caller = util_rrt.Pos_caller()
    rospy.Subscriber("odom",Odometry,pos_caller.positionCallback)
    # topic de l'objectif du rrt
    obj_caller = util_rrt.Obj_caller()
    rospy.Subscriber("move_base_simple/goal",PoseStamped,obj_caller.objectiveCallback)
    obj_caller.obj = Point(-10,-4,0)
    #obj_caller.obj = Point(0,0,0)

    while not rospy.is_shutdown():
        rospy.loginfo("on entre dans la boucle du rrt")
        rospy.loginfo("en attente de l'objectif du rrt...")
        while(obj_caller.obj==None):
            rospy.spin()
        rospy.loginfo("objectif recupere")
        # service de recuperation de la matrice map
        rospy.wait_for_service('binary_map')
        rospy.loginfo("binary map recuperee")
        try:
            binary_map = rospy.ServiceProxy('binary_map', BinaryMap)
            w = binary_map().map.info.width
            h = binary_map().map.info.height
            rospy.loginfo("w = %d h = %d",w,h)
            map = np.zeros([w,h])
            bm = binary_map().map.data
            map_origin = binary_map().map.info.origin.position
            map_resolution = binary_map().map.info.resolution
            rospy.loginfo("origine : [%f,%f]",map_origin.x,map_origin.y)
            rospy.loginfo("resolution : %f",map_resolution)
            for i in range (w):
                map[i] = bm[i*w:i*w+h]
            rospy.loginfo("binary map ok")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # creation de l'environnement
        env = util_rrt.Env(map)
        pos_pixel = util_rrt.transcription_repere_map(pos_caller.pos,map_origin,map_resolution)
        print(pos_pixel)
        obj_pixel = util_rrt.transcription_repere_map(obj_caller.obj,map_origin,map_resolution)
        print(obj_pixel)
        t = util_rrt.Tree([pos_pixel[1],pos_pixel[0]])
        t2 = util_rrt.Tree([obj_pixel[1],obj_pixel[0]])

        rospy.loginfo("execution du rrt connect ...")
        # creation du publisher pour l'affichage des segments
        segmentPub = rospy.Publisher('segments_rrt',ListePoints,queue_size=100,latch=True)

        node_t, node_t2 = util_rrt.rrt_connect(t,t2,env,segmentPub)
        rospy.loginfo("rrt connect termine !")

        path_tree = util_rrt.find_path(node_t,node_t2)
        rospy.loginfo("chemin trouve !")

        util_rrt.simplify_path(path_tree,env)
        rospy.loginfo("chemin simplifie !")

        checkpoints = ListePoints()

        node = path_tree
        while node.successors != []:
            pixel = [node.state[1],node.state[0]]
            point = util_rrt.transcription_map_repere(pixel,map_origin,map_resolution)
            rospy.loginfo("checkpoint ajoute : [ %f , %f ]",point.x,point.y)
            checkpoints.points.append(point)
            node = node.successors[0]
        pixel = [node.state[1],node.state[0]]
        point = util_rrt.transcription_map_repere(pixel,map_origin,map_resolution)
        rospy.loginfo("checkpoint ajoute : [ %f , %f ]",point.x,point.y)
        checkpoints.points.append(point)
        rospy.loginfo("liste des checkpoints creee")
        checkpointService = rospy.Service('checkpoints',Checkpoints,send_checkpoints)
        rospy.loginfo("liste des checkpoints envoyee !")
        rospy.spin()
