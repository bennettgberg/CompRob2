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
import matplotlib.axes.Axes.arrow as arrow
import math

import rospy
import publisher
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg
import tf.transformations


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
                        if planning.twoDdistance((x1,y1),(10,8.5))<plannng.twoDdistance((x,y),(10,6.5)):
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
    minDist=float('inf')

    global model_state_x
    global model_state_y
    global model_state_theta
    rospy.init_node("ackermann_model_state_subscriber")
    for derp in range(0,5):
        #time to propagate this control
        timeStep=rand.rand()*.32+.1
        #steering angle
        steeringAngle=rand.rand()*.78
        #tangent velocity
        velocity=rand.rand()*15

        #Publishing controls to publisher
        publisher.ackermann_publisher(velocity, steeringAngle, acc, jerk, timeStep)
        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, ackermann_model_state)
        
        newDist=planning.twoDdist((startConfig[0],startConfig[1]),(model_state_x, model_state_y))
        if newDist<minDist:
            minDist=newDist
            newConfig=(newX,newY,newTheta)
            newControls=(velocity,steeringAngle,timeStep)
    return newConfig,newControls

def ackermann_model_state(msg)

    model_state_x = msg.pose.position.x
    model_state_y = msg.pose.position.y

    model_state_quaternion = msg.pose.orientation
    (model_state_roll,model_state_pitch,model_state_theta) = tf.transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])


def RRT2DshowSolution(carSolutionConfigs):
    polyXs=[1.2,1.5,1.5,1.2]
    polyYs=[6.5,6.5,-1.5,-1.5]
    plt.fill(polyXs,polyYs)
    polyXs=[6,6.3,6.3,6]
    polyYs=[2.9,2.9,-4.2,-4.2]
    plt.fill(polyXs,polyYs)   
    polyXs=[-4.2,-4.2,-4.5,-4.5]
    polyYs=[1,-7.5,-7.5,1]
    plt.fill(polyXs,polyYs)    
    for i in range(0,len(carSolutionConfigs)-1):
                x=carSolutionConfigs[i][0]
                x1=carSolutionConfigs[i+1][0]
                y=carSolutionConfigs[i][1]
                y1=carSolutionConfigs[i+1][1]
                theta=carSolutionConfigs[i][2]
                #plots the configuration at each point along the path as an arrow
                dy=np.sin(theta)*.05
                dx=np.cos(theta)*.05
                arrow(x, y, dx, dy)
                #plots the path taken. Very obviously wrong because we don't curve it, but I don't feel like splining this
                xs=[x,x1]
                ys=[y,y1]
                plt.plot(xs,ys,'blue',linestyle='dashed')    
    plt.show()
    return(0)

def main():
    start=(-7.5,-7,.5*np.pi())
    goal=[(10,6.5),(10,4.5),(8,4.5),(8,6.5)]
    (carConfigs, carControls, carChildren,carParents, goalIndex)=RRTROS(start, goal, 250, False)
    index=goalIndex
    solution=[]
    #traces up the tree and appends every node as it is found
    while(index is not 0):
        solution.insert(0,carConfigs[index])
        index=carParents[index]
    RRT2DshowSolution(solution)
    return 0

if __name__ == "__main__":
    main()
