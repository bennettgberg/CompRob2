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

