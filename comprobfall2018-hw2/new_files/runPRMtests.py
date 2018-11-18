import astar_fringe as fring
import astar_node as anode
import astar_closed as closed
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import sys
import PRM
import planning
import geometry_msgs.msg
import time
from itertools import repeat

#return heuristic of start while searching for goal (given by Equation 1 in assignment instructions). Or straight-line distance if running FDA*.
def heur(config, goalConfig):
    return planning.distance(config, goalConfig)

#function to run the A* search algorithm from vertex start to vertex goal.
def runAStar(Start, goal, nodes, adjacency, distances):
    #start with an empty fringe.
    fringe = fring.Fringe()
    Closed = closed.Closed()
    start = Start 
    start.parent = None #start
    # g is 0 for the start
    start.f = start.h + 0
    fringe.insert(start) 

    while not fringe.empty():
        s = fringe.fringePop()
        if s.config == goal:
            return s, s.g
        Closed.insert(s)
# Now go to neighbors of s, check if visited or in fringe.
        updated = False
        #find index of s
        s_ind = -1
        for ind in range(len(nodes)):
            if nodes[ind] == s.config:
                s_ind = ind
                break
        if s_ind == -1: sys.exit("Error: s_ind not found! s={}".format(str(s)))
        for ni in range(len(adjacency[s_ind])): #get k nearest neighbors of s: FIX THIS!!!!!(Which index for adjacency???)!
            config = nodes[adjacency[s_ind][ni]] #neighbor's configuration
            #if not planning.validPath(s.config, config, 10):
            #    continue
            if not Closed.check(*config):
                if not fringe.check(*config): #theta):
                    n = anode.Node(config, s, heur(config, goal))
                    fringe.insert(n)
              #update n so its parent is s.
                if fringe.update(s, distances[s_ind][ni], *config):  #What is correct first index for distances?????????????????!!!!!!!!!!
                    updated = True
        if updated:
            fringe.updateFringeHeap()
#if haven't found the goal by the time the fringe is all empty, there is no valid path to the goal.
    return None, 0

def main():
    #locate start vertex of piano bot and goal vertex.
    """Read the input arguments"""
    prmstar = True
    minNodes=50
    maxNodes=150
    nodeIncrement=25
    nnodes = minNodes
    pianoNodes=None
    pianoAdjacency=None
    pianoDistances=None
    prmstar=False
    buildingCollisionChecks=[]
    runningCollisionChecks=[]
    buildCollideFile=open("kNearestBuildingCollisions.txt" "w")
    
    
    #generates 50 start and end points
    startList=[]
    goalList=[]
    firstSolved=list(repeat(-1, 50))
    i=0
    while i<50:
            (startx, starty) = planning.sample2D(10, 10)
            (goalx, goaly) = planning.sample2D(10, 10)
            #piano start and goal must be on the ground, have no rotation
            start = (startx, starty, 0.3, 1, 0, 0, 0)
            goal = (goalx, goaly, 0.3, 1, 0, 0, 0)
            startCollides = planning.collides(start[0:3], planning.quatToMatrix(start[3],start[4],start[5],start[6]))
            goalCollides = planning.collides(goal[0:3], planning.quatToMatrix(goal[3],goal[4],goal[5],goal[6]))
            plt.plot(startx,starty,'r.',markersize=5)
            plt.plot(goalx,goaly,'b.',markersize=5)
            if goalCollides or startCollides:
                continue
            else:
                startList.append(start)
                goalList.append(goal)
                i+=1
            
    while nnodes<=maxNodes:
        if not prmstar:
            filenameString="PRM_N="+str(nnodes)+"_data.txt"
            prm_file = open(filenameString, "w")
        else:
            filenameString="PRMstar_N="+str(nnodes)+"_data.txt"
            prm_file = open(filenameString, "w")
        prm_quals = []
        print "Starting to build PRM"
        pianoNodes, pianoAdjacency, pianoDistances,collisions = PRM.PRMPiano(nnodes,pianoNodes, pianoAdjacency, pianoDistances,prmstar)
        buildCollideFile.write(str(nnodes)+'\t'+str(collisions)+'\n')
        buildingCollisionChecks.append(collisions)
        i = 0
        runningCollisionChecks.append(0)
        #run 50 trials
        while i < 50:
            print "STARTING ITERATION "+ str(i) 
            newPianoNodes, newPianoAdjacency, newPianoDistances, startIndex, goalIndex,collisions = PRM.addStartandGoalPiano(pianoNodes, pianoAdjacency, pianoDistances, startList[i], goalList[i],prmstar)
            Start = anode.Node(newPianoNodes[startIndex], None, 0)
            Goal = (newPianoNodes[goalIndex])
            apath, final_dist = runAStar(Start, Goal, newPianoNodes, newPianoAdjacency, newPianoDistances)
            #if path wasn't found, repeat this trial
            if not apath:
                i=i+1
                continue
            #runningCollisionChecks[len(runningCollisionChecks)-1]=runningCollisionChecks[len(runningCollisionChecks)-1]+collisions            
            if firstSolved[i] is -1:
                firstSolved[i]=nnodes
            prm_quals.append(final_dist)
            prm_file.write(str(runningCollisionChecks[len(runningCollisionChecks)-1]) + '\t' + str(final_dist) + '\n ')
            i += 1
        nnodes=nnodes+nodeIncrement    
        for k in range(0,50):
            plt.plot(startList[k][0],startList[k][1],'r.',markersize=5)
            plt.plot(goalList[k][0],goalList[k],[1],'b.',markersize=5)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.set_xlim([0,10])
    plt.set_ylim([0,10])
    plt.title("Randomized Start (red) and Goal (blue) Configurations")
    plt.show()
    prm_file.close()
    #histogram of when these pairs were first solved
    plt.hist(firstSolved,bins=[-1,25,50,75,100,125,151])
    plt.title("Map Size for first solutions of start and goal pairs")
    plt.xlabel("number of nodes: -1 indicates no solution")
    plt.ylabel("number of pairs first solved then")
    plt.show()
    print("Final Build Times for each interation:"+str(buildingCollisionChecks)) 
main()
