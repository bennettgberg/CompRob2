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

#creates a probibilistic roadmap for the piano
#returns an array of nodes, a 2D array of indices the node at the row's index is connected to, and a distances array corresponding to these connections
#this is actually traversed using the Astar algorithm
def PRMPiano(N):
    #a list of nodes
    pianoNodes=[]
    #making this a function instead of calling cuts the runtime in half
    addNode = pianoNodes.append
    #if you assign it to a variable, it cuts the runtime in half
    #addNode = node.append
    #the adjacency list: indicates which nodes in the first list are connected to the others
    pianoAdjacency=[]
    pianoDistances=[]
    for i in range(0,N):
        #a temporary variable for the piano position: x,y,z, quaternions
        pianoPos=(0,0,0,0,0,0,0)
        pianoAdjacency.append([])
        pianoDistances.append([])
        #gets a valid position for the piano
        #NOTE: I do not know what the coordinate system is: negative numbers might be fair game
        #in which case "x<0" is not a valid criterion
        while(pianoPos[0]<0 or pianoCollides()):
            pianoPos=sample6D()
        addNode(pianoPos)
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
        while len(kDists)<k and j<i:
            #if the line in question doesn't intesect the obstacle
            if (validPianoPath(pianoNodes[i],pianoNodes[j])): 
                dist=pianoDistance(pianoNodes[i],pianoNodes[j])
                if dist>farD:
                    farD=dist
                kDists.append(dist)
                kIndices.append(j)
            j=j+1
        
        #if k is i: appends what we have, moves on to next sample so we can get more interesting cases
        if k is i:
            for j in range(0,len(kIndices)):
                pianoAdjacency[i].append(kIndices[j])
                pianoDistances[i].append(kDists[j])
                pianoAdjacency[kIndices[j]].append(i)
                pianoDistances[kIndices[j]].append(kDists[j])
            continue
                
        #otherwise: sorts from lowest distance to highest distance, checks the rest    
        [list(x) for x in zip(*sorted(zip(kDists, kIndices), key=lambda pair: pair[0]))]    
        #gets the k closest neighbors (can be adapted to k nearest neighbors)
        for j in range(k,i):
            dist=twoDdist(pianoNodes[i],pianoNodes[j])
            if dist < farD: 
                if (validPianoPath(pianoNodes[i],pianoNodes[j])): 
                    #finds where in the list of k closest neighbors to insert it
                    for p in range(0,len(kIndices)):
                        if dist<=kDists[p]:
                            #inserts it at the first place it's closer than an existing node
                            #because these are ordered close to far
                            kDists.insert(p,dist)
                            kIndices.insert(p,j)
                            #pops the old ones now that it's inserted    
                            kDists.pop()
                            kIndices.pop()
                            #updates the new farthest distance
                            farD=kDists[k-1]                            
                            break

        #adds the edges we found to the lists            
        for j in range(0,len(kIndices)):
            ind2=kIndices[j]
            pianoAdjacency[i].append(ind2)
            pianoDistances[i].append(kDists[j])
            pianoAdjacency[ind2].append(i)
            pianoDistances[ind2].append(kDists[j])
    return pianoNodes, pianoAdjacency, pianoDistances    


#input: list of nodes, adjacency matrix, and distance matrix
#returns: augmented nodes, adjacency matrix, and distance matrix, index of start and goal states
#start and goal states will be second to last and last nodes respectively
#these new items are NOT the same objects as the old ones: the previous lists are reusable
def addStartandGoalPiano(pianoNodes, pianoAdjacency, pianoDistances, startConfig, endConfig):
    newPianoAdjacency = map(list, twoDadjacency)
    newPianoNodes=map(list,twoDNodes)
    newPianoDistances=map(list, twoDdistances)
    return 0

#returns some weighted distance function for the piano
#can use euclidean distance and quat distance as helper functions
#this does not interface with Gazebo    
def pianoDistance(config1, config2):
    return 0

#takes in two xyz tuples, returns the euclidean distance between them
def euclideanDistPiano(p1,p2):
    dist=np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
    return dist

#returns an approximate distance metric between two unit quaternions
#need: rotational speed omega to scale this into a time   
    #should this actually 1/omega so 1/sec->sec
#double check algorithm in lecture slide 5
def quatDistance(Q1,Q2):
    gamma=0
    omega=1
    #dot product
    for i in range(0,4):
        gamma+=Q1[i]*Q2[i]
    rho=omega*(1-np.abs(gamma))
    return rho  

#takes a piano configuration, outputs a boolean True if it collides, False if it is valid 
#interfaces with gazebo
def pianoCollides():
    return True


#takes two piano configurations, returns True if the path is valid, False if there is no straight line path between them
#interfaces with gazebo
def validPianoPath():
    return False
    
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
  #  plt.plot(poly1)
    for i in range(0,N):
        x=-1
        y=-1
        twoDadjacency.append([])
        twoDdistances.append([])
        #gets a valid X and Y
        while(x<0 or poly1.contains(geo.Point(x,y))):
            (x,y)=sample2D(5,5)
        plt.plot(x,y,'r.')
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
        while len(kDists)<k and j<i:
            lineArgs=[twoDnodes[i],twoDnodes[j]]
            tempLine=geo.LineString(lineArgs) 
            #if the line in question doesn't intesect the obstacle
            if not (poly1.contains(tempLine) or tempLine.crosses(poly1)): 
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
                twoDdistances[i].append(kDists[j])
                twoDadjacency[kIndices[j]].append(i)
                twoDdistances[kIndices[j]].append(kDists[j])
            continue
                
        #otherwise: sorts from lowest distance to highest distance     
        [list(X) for X in zip(*sorted(zip(kDists, kIndices), key=lambda pair: pair[0]))]    
        #gets the k closest neighbors (can be adapted to k nearest neighbors)
        for j in range(k,i):
            dist=twoDdist(twoDnodes[i],twoDnodes[j])
            if dist < farD:
                lineArgs=[twoDnodes[i],twoDnodes[j]]
                tempLine=geo.LineString(lineArgs) 
                if not (poly1.contains(poly1) or tempLine.crosses(poly1)): 
                    #finds where in the list of k closest neighbors to insert it
                    for p in range(0,len(kIndices)):
                        if dist<=kDists[p]:
                            #inserts it at the first place it's closer than an existing node
                            #because these are ordered close to far
                            kDists.insert(p,dist)
                            kIndices.insert(p,j)
                            #pops the old ones now that it's inserted    
                            kDists.pop()
                            kIndices.pop()
                            #updates the new farthest distance
                            farD=kDists[k-1]                            
                            break                    
        #adds the edges we found to the lists            
        for j in range(0,len(kIndices)):
            ind2=kIndices[j]
            twoDadjacency[i].append(ind2)
            twoDdistances[i].append(kDists[j])
            twoDadjacency[ind2].append(i)
            twoDdistances[ind2].append(kDists[j])
    return twoDnodes, twoDadjacency, twoDdistances


#input: list of nodes, adjacency matrix, and distance matrix
#returns: augmented nodes, adjacency matrix, and distance matrix, index of start and goal states
#start and goal states will be second to last and last nodes respectively
#these new items are NOT the same objects as the old ones: the previous lists are reusable
def startAndGoal2DPRM(twoDNodes, twoDadjacency, twoDdistances, startConfig,endConfig):
    #copies all the existing objects
    new2Dadjacency = map(list, twoDadjacency)
    new2DNodes=map(list,twoDNodes)
    new2Ddistances=map(list, twoDdistances)
    startOrGoal=0
    vertices=[(1,1),(1,2),(2,2),(2,1)]
    poly1=geo.Polygon(vertices)    
    while startOrGoal<2:
        new2Dadjacency.append([])
        new2Ddistances.append([])        
        if startOrGoal is 0:
            new2DNodes.append(startConfig)
            currIndex=len(new2DNodes)-1
        if startOrGoal is 1: 
            new2DNodes.append(endConfig)
            currIndex=len(new2DNodes)-1
        #note: make sure this is consistent with the k used above
        k=3        
        #initializing temporary variables for k closest neighbors
        kDists=[]
        kIndices=[]
        farD=0
        j=0 
        #adds k ordered elements to the list, ordered shortest to furthest
        while len(kDists)<k and j<currIndex:
            lineArgs=[new2DNodes[currIndex],new2DNodes[j]]
            tempLine=geo.LineString(lineArgs) 
            #if the line in question doesn't intesect the obstacle
            if not (poly1.contains(tempLine) or tempLine.crosses(poly1)): 
                dist=twoDdist(new2DNodes[currIndex],new2DNodes[j])
                if dist>farD:
                    farD=dist
                kDists.append(dist)
                kIndices.append(j)
            j=j+1        
        #sorts from lowest distance to highest distance     
        [list(x) for x in zip(*sorted(zip(kDists, kIndices), key=lambda pair: pair[0]))]         
        #gets the k closest neighbors (can be adapted to k nearest neighbors)
        for j in range(k,currIndex):
            dist=twoDdist(new2DNodes[currIndex],new2DNodes[j])
            if dist < farD:
                lineArgs=[new2DNodes[currIndex],new2DNodes[j]]
                tempLine=geo.LineString(lineArgs) 
                if not (poly1.contains(poly1) or tempLine.crosses(poly1)): 
                    #finds where in the list of k closest neighbors to insert it
                    for p in range(0,len(kIndices)):
                        if dist<=kDists[p]:
                            #inserts it at the first place it's closer than an existing node
                            #because these are ordered close to far
                            kDists.insert(p,dist)
                            kIndices.insert(p,j)
                            #pops the old ones now that it's inserted    
                            kDists.pop()
                            kIndices.pop()
                            #updates the new farthest distance
                            farD=kDists[k-1]                            
                            break                    
        #adds the edges we found to the lists            
        for j in range(0,len(kIndices)):
            ind2=kIndices[j]
            new2Dadjacency[currIndex].append(ind2)
            new2Ddistances[currIndex].append(kDists[j])
            new2Dadjacency[ind2].append(currIndex)
            new2Ddistances[ind2].append(kDists[j])
        #tells our function to add the goal instead  of the start next iteration through    
        startOrGoal+=1
    #saves start and goal indices to return to client    
    startIndex=len(new2DNodes)-2
    goalIndex=len(new2DNodes)-1
    #returns augmented copies of the original objects as well as start and goal indexes
    return new2DNodes, new2Dadjacency, new2Ddistances, startIndex,goalIndex 
    
def PRM2Dshow(twoDnodes,twoDadjacency,twoDdistances):
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


