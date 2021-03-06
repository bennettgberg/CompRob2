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
import publisher

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
    #        if not planning.validPath(s.config, config, 10):
    #            continue #path has already been confirmed valid, no need to check again!
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
    return None

def main():
    #locate start vertex of piano bot and goal vertex.
    """Read the input arguments"""
    prmstar = False
    k = 3
    nnodes = 50
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "-n" or sys.argv[i] == "-N" or sys.argv[i] == "-nnodes":
            nnodes = int(sys.argv[i+1])
            i=i+1
        elif sys.argv[i] == "-s" or sys.argv[i] == "-star":
            prmstar = True
	elif sys.argv[i] == "-k":
	    k = int(sys.argv[i+1])
    #runAStar will return goal node if there's a path to goal from start

    plt.xlim(-2, 10)
    plt.ylim(0, 10)
    plt.xticks(np.arange(-2, 10, 1))
    plt.yticks(np.arange(0, 10, 1))
    twoDnodes, twoDadjacency, twoDdistances, ncc = PRM.PRMPiano(nnodes, None, None, None, prmstar, k)
    print "Roadmap created!"
    twoDnodes, twoDadjacency, twoDdistances, startIndex, goalIndex, ncc2 = PRM.addStartandGoalPiano(twoDnodes, twoDadjacency, twoDdistances, (5.0, 9.0, 0.4, 1.0, 0.0, 0.0, 0.0), (1.5, 1.5, 0.4, 1.0, 0.0, 0.0, 0.0), prmstar)
    # twoDnodes, twoDadjacency, twoDdistances, startIndex, goalIndex, ncc2 = PRM.addStartandGoalPiano(twoDnodes, twoDadjacency, twoDdistances, (5.0, 9.0, 0.4, 1.0, 0.0, 0.0, 0.0), (4.0, 4.0, 0.4, 1.0, 0.0, 0.0, 0.0), prmstar)
    print "Start and goal added!"
    Start = anode.Node(twoDnodes[startIndex], None, 0)
    Goal = (twoDnodes[goalIndex])
    apath = runAStar(Start, Goal, twoDnodes, twoDadjacency, twoDdistances)
    print "A* finished!"
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
        #current = path.pop()
        #while path != []:
        for i in range(len(path)):
            states = planning.getPath(path[i].config, path[i+1].config, 10)
            for j in range(len(states)):
                print("Position pos: x={}, y={}".format(states[j][0], states[j][1]))
                tbx.append(states[j][0])
                tby.append(states[j][1])
              #  current = path.pop()

                # add points to list to send to turtlebot_control_client
                q = geometry_msgs.msg.Quaternion(states[j][4], states[j][5], states[j][6], states[j][3])
                p = geometry_msgs.msg.Point(*states[j][0:3])
                pose = geometry_msgs.msg.Pose(p, q)
                poses.append(pose)

                print("Position: x={}, y={}".format(states[j][0], states[j][1]))
                tbx.append(states[j][0])
                tby.append(states[j][1])
                plt.plot(tbx, tby)
    else:
        print("Error! No path found!")
    plt.show()
   # PRM.PRM2Dshow(twoDnodes, twoDadjacency, twoDdistances)
    # plt.show()
    print("poses = {}".format(poses))
    # twist = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), geometry_msgs.msg.Vector3(0,0,0))
    for pose in poses:
        print "Sending (%s, %s, %s)"%(pose.position.x, pose.position.y, pose.position.z)
        publisher.model_state_publisher(pose, model_name="piano2")
        print "Sent piano to (%s, %s, %s)."%(pose.position.x, pose.position.y, pose.position.z)
main()
