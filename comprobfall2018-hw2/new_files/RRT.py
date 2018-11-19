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
# import matplotlib.axes.Axes as axes
import math

import rospy
import publisher
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
    #Outer loop: builds N nodes
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
        newx=None
        while True:
            if greedy:          
                #samples two new points, compares to old sample and picks the one closest to the goal
                for q in range(0,1):
                    (x1,y1)=sampleRRTPt(19,14,(-9,-7.5),polys)
                    if planning.twoDdist((x1,y1),(10,8.5))<planning.twoDdist((x,y),(10,6.5)):
                        (x,y)=(x1,y1)
                print("greedy algorithm sampled x="+ str(x) +" y="+ str(y))
            else:
                #only checks this point
                (x,y)=sampleRRTPt(19,14,(-9,-7.5),polys)
                print("testing with point x=" + str(x) +" y="+ str(y))            
            for j in range(0,i+1):
                #gets distane  between current node and th sampled node
                dist = planning.twoDdist((carConfigs[j][0],carConfigs[j][1]),(x,y))   
                #sees if it's close enough to even be worth checking
                if dist >= minD:
                    continue

                #start nominal collision check    
                lineArgs=[(carConfigs[j][0],carConfigs[j][1]), (x,y)]
                tempLine=geo.LineString(lineArgs) 
                #if the line in question doesn't intesect the obstacle, it's good to go
                polyCheck=True
                for polyIndex in range(0,len(polys)):
                    if polys[polyIndex].contains(tempLine) or tempLine.crosses(polys[polyIndex]):
                        polyCheck=False
                        break  
                #end collision check: if wecomment this out, be sure to leave polycheck true

                #assigns a new minimum distance, saves the index of the closest point    
                if polyCheck: 
                        #too close to existing nodes: we bias it to move away
                        if dist<.25:
                            newx=None
                            newy=None
                            minD=float('inf')
                            break
                        #else: adds it to the tree
                        minD=dist
                        closej=j 
                        (newx,newy) = (x, y)
            #if no valid straight line paths were found, kills it             
            if newx == None:
                (x,y)=sampleRRTPt(19,14,(-9,-7.5),polys)
            else:
                break
            print("Confirmed Valid Point x="+ str(x) +" y="+ str(y))
        #adds a parent link from the closest node to the new node
        addParent(closej)
        #adds this node as a child for row closej
        carChildren[closej].append(i)
        #interfaces a few test controls with gazebo, returns the best one 
        print("sampling controls from Gazebo")   
        (newConfig,newControls)=RRTSampleControls(carConfigs[j],(newx,newy), greedy)
        print("found valid controls: {}".format(newControls))
        addConfig(newConfig)
        addControls(newControls)
        if not goalFound:
            print("checking if a goal node was found")
            goalPt=geo.Point(newConfig[0],newConfig[1])
            if goalRegionPoly.contains(goalPt):
                goalIndex=j
                goalFound=True
                print("found point in goal region")
                break
        i += 1
    return carConfigs, carControls, carChildren,carParents, goalIndex
 
def ackermann_model_state(msg):

    model_state_x = msg.pose[2].position.x
    model_state_y = msg.pose[2].position.y
    model_state_quaternion = [msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w]
    # print(model_state_x, model_state_y, model_state_quaternion)

#from the initial state, samples X controls and returns the set of controls that gets the closest, as well as the final location
#odd is 1 or 0, prevents it from veering
#startConfig is a tuple (x,y,theta) for the car
#goalLoc is (x,y)
def RRTSampleControls(startConfig,goalLoc, greedy):

    #teleports robot to state closest to point sampled
    q = quaternion_from_euler(0,0,startConfig[2], axes='sxyz')
    q = geometry_msgs.msg.Quaternion(*q)
    p = geometry_msgs.msg.Point(startConfig[0],startConfig[1],0)
    pose = geometry_msgs.msg.Pose(p, q)
    publisher.model_state_publisher(pose, model_name="ackermann_vehicle")
    rospy.sleep(0.5)

    acc=12
    jerk=8
    minDist=float('inf')
    
    #tries 2 controls
    hitWall=False
    frontOrBack=1
    derp=0
    if not greedy: derp = 1
    while derp <2:
        #time to propagate this control
        timeStep=rand.rand()+.25
        #steering angle
        steeringAngle=rand.rand()*2*0.78-0.78
    #    if derp is 0:
    #        steeringAngle=steeringAngle*(-1)
        #tangent velocity
        velocity=(rand.rand()+1)*frontOrBack
        steeringAngleVelocity=rand.rand()*245 #245-np.abs(steeringAngle*100)


        #Publishing controls to publisher and wait for fall
        publisher.ackermann_publisher(velocity, steeringAngle, steeringAngleVelocity, acc, jerk, timeStep)
        rospy.sleep(0.25)

        client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # sub = rospy.Subscriber("/gazebo/model_states", ModelStates, ackermann_model_state)

        msg = client("ackermann_vehicle","")
        model_state_x = msg.pose.position.x
        model_state_y = msg.pose.position.y
        model_state_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (model_state_roll,model_state_pitch,model_state_theta) = euler_from_quaternion(model_state_quaternion)

        print(model_state_x, model_state_y, model_state_roll, model_state_pitch, model_state_theta)

        if np.abs(model_state_roll)>0.4 or np.abs(model_state_pitch)>0.4:
            # if hitWall==False:
            #     hitWall=True
            # elif hitWall:
            #     frontOrBack=-3
            frontOrBack=-3
            derp=derp-1
            publisher.model_state_publisher(pose, model_name="ackermann_vehicle")
            rospy.sleep(1)

        else:
            frontOrBack=1
            newDist=planning.twoDdist((model_state_x, model_state_y), (goalLoc[0],goalLoc[1]))
            if not (greedy and newDist>minDist):
                minDist=newDist
                newConfig=(model_state_x,model_state_y,model_state_theta)
                newControls=(velocity,steeringAngle,timeStep)
        derp=derp+1
    return newConfig,newControls


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
                # axes.arrow(x, y, dx, dy)
                #plots the path taken. Very obviously wrong because we don't curve it, but I don't feel like splining this
                xs=[x,x1]
                ys=[y,y1]
                plt.plot(xs,ys,'blue',linestyle='dashed')    
    plt.show()
    return(0)

def main():
    # start=(-7.5,-7,0.5*np.pi)
    # start=(4.0,0.0,0.5*np.pi)
    start=(8.0,0.0,0.5*np.pi)
    goal=[(10,6.5),(10,4.5),(8,4.5),(8,6.5)]

    (carConfigs, carControls, carChildren,carParents, goalIndex)=RRTROS(start, goal, 250, True)
    index=goalIndex
    solution=[]
    #traces up the tree and appends every node as it is found
    print("Solution found, tracing back through the tree")
    while(index is not 0):
        solution.insert(0,carConfigs[index])
        index=carParents[index]
    #visualizes solution    
    RRT2DshowSolution(solution)

    global model_state_x
    global model_state_y
    global model_state_quaternion

    return 0

if __name__ == "__main__":
    main()
