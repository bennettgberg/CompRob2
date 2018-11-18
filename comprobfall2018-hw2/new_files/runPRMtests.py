import astar_fringe as fring
import astar_node as anode
import astar_closed as closed
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import sys
import PRM
import RRT
import planning
import geometry_msgs.msg
import time

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
            if not planning.validPath(s.config, config, 10):
                continue
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
    prmstar = False
    minNodes=25
    maxNodes=100
    nodeIncrement=25
    nnodes = minNodes
    pianoNodes=None
    pianoAdjacency=None
    pianoDistances=None
    prmstar=False
    buildTimes=[]
    while nnodes<=maxNodes:
        if not prmstar:
            filenameString="PRM_N="+str(nnodes)+"_data.txt"
            prm_file = open(filenameString, "w")
        else:
            filenameString="PRMstar_N="+str(nnodes)+"_data.txt"
            prm_file = open(filenameString, "w")
        prm_times = []
        prm_quals = []
        print "Starting to build PRM"
        init_time = time.time()
        pianoNodes, pianoAdjacency, pianoDistances = PRM.PRMPiano(nnodes,pianoNodes, pianoAdjacency, pianoDistances,prmstar)
        buildTimes.append(time.time() - init_time) #time to build the PRM (without start and goal nodes)
        i = 0
        #run 50 trials
        while i < 50:
            print "STARTING ITERATION "+ str(i) 
            (startx, starty) = planning.sample2D(10, 10)
            (goalx, goaly) = planning.sample2D(10, 10)
            #piano start and goal must be on the ground, have no rotation
            start = (startx, starty, 0.3, 1, 0, 0, 0)
            goal = (goalx, goaly, 0.3, 1, 0, 0, 0)
            start_time = time.time()
            newPianoNodes, newPianoAdjacency, newPianoDistances, startIndex, goalIndex = PRM.addStartandGoalPiano(pianoNodes, pianoAdjacency, pianoDistances, start, goal,prmstar)
            Start = anode.Node(newPianoNodes[startIndex], None, 0)
            Goal = (newPianoNodes[goalIndex])
            apath, final_dist = runAStar(Start, Goal, newPianoNodes, newPianoAdjacency, newPianoDistances)
            final_time = time.time() - start_time  #time to add start+goal, run A* to find path
            #if path wasn't found, repeat this trial
            if not apath:
                continue
            prm_times.append(final_time)
            prm_quals.append(final_dist)
            prm_file.write(str(final_time) + '\t' + str(final_dist) + '\n ')
            i += 1
        nnodes=nnodes+nodeIncrement    
        prm_file.close()
        #plot the final data: time vs. path quality
       # plt.plot(prm_times, prm_quals, '.b')
        #plt.show()
    print("Final Build Times for each interation:"+str(buildTimes)) 
main()
