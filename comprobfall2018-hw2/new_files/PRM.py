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

#returns a probabilistic random map with N nodes
def PRMPiano(N):
    #a list of nodes
    nodes=[]
    #if you assign it to a variable, it cuts the runtime in half
    #addNode = node.append
    #the adjacency list: indicates which nodes in the first list are connected to the others
    adjacency=[]
    nodeGroups=[]
    return 0

#creates a 2D prm for testing the algorithm: will be deleted from final code
#returns an array of nodes, a 2D array of indices the node at the row's index is connected to, and a distances array corresponding to these connections
def PRM2D(N): 
    #a list of 2D tuples representing x and y values of node  
    twoDnodes=[]
    addNode = twoDnodes.append
    twoDadjacency=[]
    twoDdistances=[]
    vertices=[(1,1),(1,2),(2,2),(2,1)]
    poly1=geo.Polygon(vertices)
    for i in range(0,N):
        x=-1
        y=-1
        twoDadjacency.append([])
        twoDdistances.append([])
        #gets a valid X and Y
        while(x<0 or poly1.contains(geo.Point(x,y))):
            (x,y)=sample2D(5,5)
        addNode((x,y))
        k=3
        #case: fewer than k nodes, don't check k or it will crash
        if(k>i):
            #if there is only one node, just add another
            if (i is 0) or (i is 1):
                continue
            else:
                k=i
        kDists=[]
        kIndices=[]
        farD=0
        j=0
        #adds ten ordered elements to the list, ordered shortest to furthest
        while len(kDists)<k or j<i:
            lineArgs=[twoDnodes[i],twoDnodes[j]]
            tempLine=geo.LineString(lineArgs) 
            #if the line in question doesn't intesect the obstacle
            if not (poly1.contains(poly1) or tempLine.crosses(poly1)): 
                dist=twoDdist(twoDnodes[i],twoDnodes[j])
                if dist>farD:
                    farD=dist
                kDists.append(dist)
                kIndices.append(j)
            j=j+1
        
        #if k is i: appends what we have, moves on to next sample so we can get more interesting cases
        if k is i:
            for j in range(0,len(kIndices)):
                twoDadjacency[i].append(kIndices[j])
                twoDdistances[i].append(kDists(j))
                twoDadjacency[kIndices[j]].append(i)
                twoDdistances[kIndices[j]].append(kDists(j))
            continue
                
        #otherwise: sorts from lowest distance to highest distance     
        [list(x) for x in zip(*sorted(zip(kDists, kIndices), key=lambda pair: pair[0]))]    
        #gets the k closest neighbors (can be adapted to k nearest neighbors)
        for j in range(k,i):
            dist=twoDdist(twoDnodes[i],twoDnodes[j])
            if dist < farD:
                lineArgs=[twoDnodes[i],twoDnodes[j]]
                tempLine=geo.LineString(lineArgs) 
                if not (poly1.contains(poly1) or tempLine.crosses(poly1)): 
                    kIndices.append(k)
                    kDists.append(dist)
                    #I was extremely lazy and just inserted and sorted the list each time
                    #this gets a lot faster if someone inserts the nodes  into the correct spots and then just pops the last node
                    [list(x) for x in zip(*sorted(zip(kDists, kIndices), key=lambda pair: pair[0]))]  
                    kDists.pop()
                    kIndices.pop()
                    farD=kDists[k-1]
        #adds the edges we found to the lists            
        for j in range(0,len(kIndices)):
            twoDadjacency[i].append(kIndices[j])
            twoDdist[i].append(kDists(j))
            twoDadjacency[kIndices[j]].append(i)
            twoDdist[kIndices[j]].append(kDists(j))
    return twoDnodes, twoDadjacency, twoDdistances


def PRM2Dshow(twoDnodes,twoDadjacency,twoDdistances):
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