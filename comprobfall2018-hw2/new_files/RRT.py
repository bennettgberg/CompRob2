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
import math

#returns a RRT roadmap with separation dq between nodes
def RRTacker(dq):
    #a list of nodes
    nodes=[]
    #if you assign it to a variable, it cuts the runtime in half
    #addNode = node.append
    #the adjacency list: indicates which nodes in the first list are connected to the others
    adjacency=[]
    nodeGroups=[]
    return 0

#returns coordinates of new point in RRT
#ogpt: (x,y) of point in the RRT. randpt: (x,y) of the random point (expand in this direction). dq: distance between points.
def getNew(ogpt, randpt, dq):
    m = (randpt[1] - ogpt[1] ) / (randpt[0] - ogpt[0])
    alpha = math.atan(m)
    dx = dq * math.cos(alpha)
    dy = dq * math.sin(alpha)
    return (ogpt[0]+dx, ogpt[1]+dy)

#creates a 2D RRT for testing the algorithm: will be deleted from final code
#returns an array of nodes, a 2D array of indices the node at the row's index is connected to, and a distances array corresponding to these connections
#start: (x, y) for start state. goal: (x, y) for goal state. N: number of nodes to have in the RRT. dq: distance between each node in the RRT.
#returns: (list of nodes in the RRT, accompanying adjacency list of lists of indices)
def RRT2D(start, goal, N, dq): 
    #a list of 2D tuples representing x and y values of node  
    twoDnodes=[]
    addNode = twoDnodes.append
    twoDadjacency=[]
    twoDdistances=[]
    vertices=[(1,1),(1,2),(2,2),(2,1)]
    poly1=geo.Polygon(vertices)
    addNode((start.x, start.y))
    i = 0
    while i < N:          
  #      print "i=" + str(i)
        x=-1
        y=-1
        twoDadjacency.append([])
        twoDdistances.append([])
        #gets a valid X and Y
        (x,y)=sample2D(5,5) #doesn't matter if there's a collision because we're just going dq in the direction of this point.
        plt.plot(x,y,'r.')
        addNode((x,y))
        j=0
        #Get closest existing node 
        newpt = None
        minD = float('inf')
        closej = 0
        for j in range(i+1):
            (newx, newy) = getNew(twoDnodes[j], (x,y), dq)
            lineArgs=[twoDnodes[j], (newx, newy)]
            tempLine=geo.LineString(lineArgs) 
            #if the line in question doesn't intesect the obstacle
            if not (poly1.contains(tempLine) or tempLine.crosses(poly1)): 
                dist = twoDdist(twoDnodes[j],(x,y))        
                if dist < minD:
                    minD=dist
                    closej = j
                    newpt = (newx, newy)
  #                  print "newpt: " + str(newpt)
        if newpt == None:
            continue
        twoDnodes.append(newpt)
        twoDadjacency[i].append(closej)
        twoDadjacency[closej].append(i)
        i += 1
    return twoDnodes, twoDadjacency


def RRT2Dshow(twoDnodes,twoDadjacency):
    polyXs=[1,1,2,2]
    polyYs=[1,2,2,1]
    plt.fill(polyXs,polyYs)
    for i in range(0,len(twoDadjacency)-1):
        for j in range(i,len(twoDadjacency[i])):
            if twoDadjacency[i][j]>i:
                pt1=twoDnodes[i]
                pt2=twoDnodes[twoDadjacency[i][j]]
                ys=[pt1[1],pt2[1]]
                xs=[pt1[0],pt2[0]]
                plt.plot(xs,ys,'red',linestyle='dashed')    
    return(0)

def twoDdist(pt1,pt2):
    return np.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)    
    
#returns a 2D point in the 2D square defined from (0,0) to (xmax,ymax)    
def sample2D(xmax,ymax):
    return (xmax*rand.rand(),ymax*rand.rand())


#takes in two xyz tuples, returns the euclidean distance between them
def euclideanDistPiano(p1,p2):
    dist=np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
    return dist
    
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
