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
            return s
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
                if fringe.update(s, config, distances[s_ind][ni]):  #What is correct first index for distances?????????????????!!!!!!!!!!
                    updated = True
        if updated:
            fringe.updateFringeHeap()
#if haven't found the goal by the time the fringe is all empty, there is no valid path to the goal.
    return None

def main():
    #locate start vertex of piano bot and goal vertex.
    """Read the input arguments"""
    ros = False
    rrt = False
    nnodes = 50
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "-n" or sys.argv[i] == "-num" or sys.argv[i] == "-number":
            num = int(sys.argv[i+1])
            i=i+1
        elif sys.argv[i] == "-ros" or sys.argv[i] == "-r":
            ros = True
        elif sys.argv[i] == "-RRT" or sys.argv[i] == "-rrt":
            rrt = True
        elif sys.argv[i] == "-nnodes":
            nnodes = int(sys.argv[i+1])
    #runAStar will return goal node if there's a path to goal from start

    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.xticks(np.arange(0, 5, 1))
    plt.yticks(np.arange(0, 5, 1))
    if not rrt:
        twoDnodes, twoDadjacency, twoDdistances = PRM.PRMPiano(nnodes)
        twoDnodes, twoDadjacency, twoDdistances, startIndex, goalIndex = PRM.addStartandGoalPiano(twoDnodes, twoDadjacency, twoDdistances, (5.0, 9.0, 0, 1, 0, 0, 0), (4.0, 4.0, 0, 1, 0, 0, 0))
        Start = anode.Node(twoDnodes[startIndex], None, 0)
        Goal = (twoDnodes[goalIndex])
    else:
        #FIX RRT
        twoDnodes, twoDadjacency = RRT.RRT2D(Start, (5,5), nnodes, 0.2)
        twoDdistances = [[0.2 for x in range(nnodes)] for y in range(nnodes)]
    apath = runAStar(Start, Goal, twoDnodes, twoDadjacency, twoDdistances)
    if rrt:
        RRT.RRT2Dshow(twoDnodes, twoDadjacency)
     #   plt.show()
    #NOW MOVE ALONG THE PATH THAT WAS FOUND!
    poses = []
    if not apath == None:
        #Now just follow the parent of goal in reverse to find the correct path.
        current = apath                                                  
        path = []
        tbx = []
        tby = []
        while current != None: 
            path.append(current)
            current = current.parent
        current = path.pop()
        while path != []:
            if not ros: print("Position pos: x={}, y={}".format(current.config[0], current.config[1]))
            tbx.append(current.config[0])
            tby.append(current.config[1])
            current = path.pop()
            if ros:
                import ackermann_publisher
                # add points to list to send to turtlebot_control_client
                q = geometry_msgs.msg.Quaternion(*current.config[3:7])
                p = geometry_msgs.msg.Point(*current.config[0:3])
                pose = geometry_msgs.msg.Pose(p, q)
                poses.append(pose)
        if not ros: print("Position: x={}, y={}".format(current.config[0], current.config[1]))
        tbx.append(current.config[0])
        tby.append(current.config[1])
        plt.plot(tbx, tby)
    else:
        print("Error! No path found!")
    plt.show()
   # PRM.PRM2Dshow(twoDnodes, twoDadjacency, twoDdistances)
    plt.show()
    if ros:
        for pose in poses:
            print "Sending (%s,%s)"%(p.x, p.y)
            if ackermann_publisher.model_state_publisher(pose).data !=True:
                print("Error! Collsion!")
            else: print "Sent turtlebot to (%s, %s) with no collision."%(p.x, p.y)
main()
