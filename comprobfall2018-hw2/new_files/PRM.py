# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 16:49:08 2018

@author: Mike Nitzsche
"""
import numpy.random as rand
import numpy as np

#returns a probabilistic random map 
def PRM():
    x=0
    

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

#returns an approximate distance metric between two unit quaternions
#need: rotational speed omega to scale this into a time   
    #should this actually 1/omega so 1/sec->sec
def quatDistance(Q1,Q2):
    gamma=0
    omega=1
    #dot product
    for i in range(0,4):
        gamma+=Q1[i]*Q2[i]
    rho=omega*(1-np.abs(gamma))
    return rho
    
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