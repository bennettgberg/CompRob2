# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 16:49:08 2018

@author: Mike Nitzsche
"""
import numpy.random as rand
import numpy as np
import shapely.geometry as geo
import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import rospy
import math
from pqp_server.srv import *

def key(config):
    return str(config[0])+'_'+str(config[1]) + '_' + str(config[2]) + '_' + str(config[3]) + '_' + str(config[4]) + '_' + str(config[5]) + '_' + str(config[6])

#returns some weighted distance function for the piano
#can use euclidean distance and quat distance as helper functions
#this does not interface with Gazebo    
def distance(p1, p2, wt=1):
    ed = euclideanDist((p1[0], p1[1], p1[1]), (p2[0], p2[1], p2[2])) 
    qd = quatDist((p1[3], p1[4], p1[5], p1[6]), (p2[3], p2[4], p2[5], p2[6]))
    return ed + wt*quatDist

#takes in two xyz tuples, returns the euclidean distance between them
def euclideanDist(p1,p2):
    dist2 = 0
    for i in range(7):
        dist2 += (p1[i]-p2[i])**2
    return math.sqrt(dist2)

#returns an approximate distance metric between two unit quaternions
#need: rotational speed omega to scale this into a time   
    #should this actually 1/omega so 1/sec->sec
#double check algorithm in lecture slide 5
def quatDist(Q1,Q2):
    gamma=0
    omega=1
    #dot product
    for i in range(0,4):
        gamma+=Q1[i]*Q2[i]
    rho=omega*(1-np.abs(gamma))
    return rho  

#takes a configuration, outputs a boolean True if it collides, False if it is valid 
#interfaces with gazebo
def collides(T, R):

    R_flat = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    for i in R:
        for j in R[i]: 
            R_flat[3*i+j] = R[i][j]

    if len(T) != 3 or len(R_flat) != 9:
        print "Incorrect list size for pqp request"
        return True
    rospy.wait_for_service('pqp_server')
    try:
        pqp_server = rospy.ServiceProxy('pqp_server', pqpRequest)
        result = pqp_server(T, R_flat)
        return result
    except rospy.ServiceException, e:
        print "Service Call Failed: %s"%e

    return True


#takes two configurations (x1,y1,z1,w1,i1,j1,k1) and (x2,y2,z2,w2,i2,j2,k2), returns True if the path is valid, False if there is no straight line path between them
#interfaces with gazebo
def validPath(config1, config2):
    return False
    
def getPath(state1,state2):
    return 0

#returns randomized x,y,z, and a unit quaternion
def sample6D(xmax,ymax,zmax):
    x=rand.rand()*xmax
    y=rand.rand()*ymax
    z=rand.rand()*zmax
    s=rand.rand()
    sigma1=np.sqrt(1-s)
    sigma2=np.sqrt(s)
    theta1=2*np.pi*rand.rand()
    theta2=2*np.pi*rand.rand()
    a=np.cos(theta2)*sigma2
    b=np.sin(theta1)*sigma1
    c=np.cos(theta1)*sigma1
    d=np.sin(theta2)*sigma2    
    return (x,y,z,a,b,c,d)

    
#input: two quaternions Q1 and Q2 as 4 element tuples
#output: interpolation quaternion from Q1 to Q2  
def quatSLERP(Q1,Q2):
   #lam is magnitude
    lam=0
    f=.5
    #eps is the maximum angular distance to assume parallel
    #Mike made this number up, might need to check later
    eps=10**-6
    #dot product
    for i in range(0,4):
        lam+=Q1[i]*Q2[i]
    #if negative, flips them
    if lam <0:
        Q2=(-Q2[0],-Q2[1],-Q2[2],-Q2[3])
        lam=-lam
    if np.abs(1-lam)<eps:
        r=1-f
        s=f
    else:
        alpha=np.arccos(lam)
        gamma=1/np.sin(alpha)
        r=np.sin((1-f)*alpha)*gamma
        s=np.sin(f*alpha)*gamma
    w=r*Q1[0]+s*Q2[0]
    x=r*Q1[1]+s*Q2[1]
    y=r*Q1[2]+s*Q2[2]
    z=r*Q1[3]+s*Q2[3]
    Q=[w,x,y,z]
    Qmag=0
    for i in range(0,4):
        Qmag+=Q[i]**2
    Qmag=np.sqrt(Qmag)
    for i in range(0,4):
        Q[i]=Q[i]/Qmag
    return (Q[0],Q[1],Q[2],Q[3])
    
#converts a set of quaternions into a rotation matrix
#returns a 3x3 list
def quatToMatrix(w,x,y,z):
    R=[[0,0,0],[0,0,0],[0,0,0]]
    
    R[0][0]=1-2*y**2-2*z**2
    R[0][1]=2*x*y-2*z*w
    R[0][2]=2*x*z+2*y*w
    
    R[1][0]=2*x*y+2*z*w
    R[1][1]=1-2*x**2-2*z**2
    R[1][2]=2*y*z-2*x*w
    
    R[2][0]=2*x*z-2*y*w
    R[2][1]=2*y*z+2*x*w
    R[2][2]=1-2*x**2-2*y**2
    return R

def twoDdist(pt1,pt2):
    return np.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)    
    
#returns a 2D point in the 2D square defined from (0,0) to (xmax,ymax)    
def sample2D(xmax,ymax):
    return (xmax*rand.rand(),ymax*rand.rand())

