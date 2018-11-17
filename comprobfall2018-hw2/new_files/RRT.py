# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 16:49:08 2018

@author: Mike Nitzsche
"""
import numpy.random as rand
import planning
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
def getNewPt(ogpt, randpt, dq):
    m = (randpt[1] - ogpt[1] ) / (randpt[0] - ogpt[0])
    alpha = math.atan(m)
    dx = dq * math.cos(alpha)
    dy = dq * math.sin(alpha)
    return (ogpt[0]+dx, ogpt[1]+dy)

def sampleRRTPt(xmax,ymax,shift,polys):
    #gets a valid X and Y in the boundary of the actual world, eliminates points that are obviously inside obstacles
    while(True):
        pointCheck=True
        (x,y)=planning.sample2D(xmax,ymax)
        (x,y)=(x+shift[0],y+shift[1])
        tmpPt=geo.Point((x,y))
        for polyIndex in range(0,len(polys)):
            if polys[polyIndex].contains(tmpPt):
                pointCheck=False
        if pointCheck is True:
            return((x,y))

#takes in a start and goal node, number of expansions, maximum space between 
#state nodes take the form (x,y,theta)    
#start is an x, y, theta
#goal is a goal (x,y) polygon with clockwise vertices
#N is the maximum number of samples: terminates before that if a goal state is found
#greedy is True or False
def RRTROS(start, goal, N, greedy):
    #gazebo limits
    #MAXV=17.8816 #m/s: maximum tangential velocity
    #MAXSTEER=244.8696 #rad/s: maximum steering angle velocity
    #MAXANGLE=0.785398163 #rad: maximum steering angle
    #maxTimeStep=dq/MAXV
    #car x, y, theta
    carConfigs=[]
    #stores the tangential velocity, angular velocity, and time used to get here from the parent
    #Size N, but first row is null
    carControls=[]
    addControls=carControls.append
    addConfig=carConfigs.append
    #parents of each node: will be size N (but first row is null)
    carParents=[]
    addParent=carParents.append
    #children of each node: will be size N-=1
    carChildren=[]
    goalRegionPoly=geo.Polygon(goal)
    #creates the actual obstacles form the website
    polys=[]
    vertices=[(1.2,6.5),(1.5,6.5),(1.5,-1.5),(1.2,-1.5)]
    polys.append(geo.Polygon(vertices))
    vertices=[(6,2.9),(6.3,2.9),(6.3,-4.2),(6,-4.2)]
    polys.append(geo.Polygon(vertices))
    vertices=[(-4.2,1),(-4.2,-7.5),(-4.5,-7.5),(-4.5,1)]
    polys.append(geo.Polygon(vertices))
    #adds initial conditions
    addConfig((start[0],start[1],start[2]))
    addControls(None)    
    i = 0 
    goalFound=False
    goalIndex=None
    while i < N:          
        x=-1
        y=-1
        carChildren.append([])
        (x,y)=sampleRRTPt(19,14,(-9,-7.5),polys)
        #addConfig((x,y))
        j=0
        #Get closest existing node 
        newx=None
        minD = float('inf')
        closej = 0
        #finds nearest existing that doesn't obviously intersect with an obstacle
        #if there are none, resamples the point and tries again
        while True:
            for j in range(i+1):
                if greedy:
                    #we decided this was a shit algorithm but I'm leaving it here in case we want to try that later
                    #defines a 90 degree arc towards the bottom left corner where the car is not allowed to go
                    #relativeX=x-carConfigs[j][0]
                    #relativeY=y-carConfigs[j][1]
                    #if relativeY<0 and relativeX<0:
                    #    continue
            
                    #samples two new points, compares to old sample and picks the one closest to the goal
                    for q in range(0,1):
                        (x1,y1)=sampleRRTPt(19,14,(-9,-7.5),polys)
                        if twoDdistance((x1,y1),(10,8.5))<twoDdistance((x,y),(10,8.5)):
                            (x,y)=(x1,y1)
                dist = planning.twoDdistance((carConfigs[j][0],carConfigs[j][1]),(x,y))   
                #sees if it's close enough to even be worth checking
                if dist >= minD:
                    continue
                lineArgs=[(carConfigs[j][0],carConfigs[j][1]), (x,y)]
                tempLine=geo.LineString(lineArgs) 
                #if the line in question doesn't intesect the obstacle, it's good to go
                polyCheck=True
                for polyIndex in range(0,len(polys)):
                    if polys[polyIndex].contains(tempLine) or tempLine.crosses(polys[polyIndex]):
                        polyCheck=False
                        break  
                #assigns a new minimum distance, saves the index of the closest point    
                if polyCheck: 
                        #too close to existing nodes: we bias it to move away
                        if dist<.5:
                            newx=None
                            newy=None
                            minD=float('inf')
                            break
                        #else: adds it to the tree
                        minD=dist
                        closej=j 
                        (newx,newy) = (x, y) 
            if newx == None:
                (x,y)=sampleRRTPt(19,14,(-9,-7.5),polys)
            else:
                break
        #adds a parent link from the closest node to the new node
        addParent(closej)
        #adds this node as a child for row closej
        carChildren[closej].append()      
        #interfaces a few test controls with gazebo, returns the best one    
        (newConfig,newControls)=RRTSampleControls(carConfigs[j],(newx,newy))
        addConfig(newConfig)
        addControls(newControls)
        if not goalFound:
            goalPt=geo.Point(newConfig[0],newConfig[1])
            if goalRegionPoly.contains(goalPt):
                goalIndex=j
                goalFound=True
                break
        i += 1
    return carConfigs, carControls, carChildren,carParents, goalIndex
 

#from the initial state, samples X controls and returns the set of controls that gets the closest, as well as the final location
#odd is 1 or 0, prevents it from veering
#startConfig is a tuple (x,y,theta) for the car
#goalLoc is (x,y)
def RRTSampleControls(startConfig,goalLoc):
    acc=12
    jerk=8
    else:
        LR=1
    minDist=float('inf')
    for derp in range(0,5):
        #time to propagate this control
        timeStep=rand.rand()*.32+.1
        #steering angle
        steeringAngle=rand.rand()*.78
        #tangent velocity
        velocity=rand.rand()*15
        """STEVEN: RUN THIS CONTROL, store the final state in newX,newY,newTheta"""
        newX=0
        newY=0
        newTheta=0
        newDist=planning.twoDdist((startConfig[0],startConfig[1]),(newX,newY))
        if newDist<minDist:
            minDist=newDist
            newConfig=(newX,newY,newTheta)
            newControls=(velocity,steeringAngle,timeStep)
    return newConfig,newControls

#creates a 2D RRT for testing the algorithm: will be deleted from final code
#returns an array of nodes, a 2D array of indices the node at the row's index is connected to, and a distances array corresponding to these connections
#start: (x, y) for start state. goal: (x, y) for goal state. N: number of nodes to have in the RRT. dq: distance between each node in the RRT.
#returns: (list of nodes in the RRT, accompanying adjacency list of lists of indices)
#World bounds: (10, 6.5), (10, -7.5), (-9, -7.5), (-9, 6.5)
#(6,2.9) (6.3,2.9) (6.3,-4.2) (6,-4.2)
#(1.2,6.5) (1.5,6.5)  (1.5,-1.5) (1.2,-1.5)
#(-4.2,1) (-4.2,-7.5) (-4.5,-7.5) (-4.5,1 )     
def RRT2D(start, goal, N, dq): 
    #a list of 2D tuples representing x and y values of node  
    twoDnodes=[]
    addNode = twoDnodes.append
    twoDadjacency=[]
    twoDdistances=[]
    polys=[]
    #creates the actual obstacles form the website
    vertices=[(1.2,6.5),(1.5,6.5),(1.5,-1.5),(1.2,-1.5)]
    polys.append(geo.Polygon(vertices))
    vertices=[(6,2.9),(6.3,2.9),(6.3,-4.2),(6,-4.2)]
    polys.append(geo.Polygon(vertices))
    vertices=[(-4.2,1),(-4.2,-7.5),(-4.5,-7.5),(-4.5,1 )]
    polys.append(geo.Polygon(vertices))
    addNode((start[0], start[1]))
    i = 0
    while i < N:          
        x=-1
        y=-1
        twoDadjacency.append([])
        twoDdistances.append([])
        #gets a valid X and Y in the boundary of the actual world
        (x,y)=planning.sample2D(19,14)#doesn't matter if there's a collision because we're just going dq in the direction of this point.
        (x,y)=(x-9,y-7.5)
        plt.plot(x,y,'r.')
        addNode((x,y))
        j=0
        #Get closest existing node 
        newpt = None
        minD = float('inf')
        closej = 0
        for j in range(i+1):
            (newx, newy) = planning.getNew(twoDnodes[j], (x,y), dq)
            lineArgs=[twoDnodes[j], (newx, newy)]
            tempLine=geo.LineString(lineArgs) 
            #if the line in question doesn't intesect the obstacle
            polyCheck=True
            for polyIndex in range(0,len(polys)):
                if polys[polyIndex].contains(tempLine) or tempLine.crosses(polys[polyIndex]):
                    polyCheck=False
                    break
            if polyCheck: 
                dist = twoDdistances(twoDnodes[j],(x,y))        
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
    polyXs=[1.2,1.5,1.5,1.2]
    polyYs=[6.5,6.5,-1.5,-1.5]
    plt.fill(polyXs,polyYs)
    polyXs=[6,6.3,6.3,6]
    polyYs=[2.9,2.9,-4.2,-4.2]
    plt.fill(polyXs,polyYs)   
    polyXs=[-4.2,-4.2,-4.5,-4.5]
    polyYs=[1,-7.5,-7.5,1]
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

