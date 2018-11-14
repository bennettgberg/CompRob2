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

#return heuristic of start while searching for goal (given by Equation 1 in assignment instructions). Or straight-line distance if running FDA*.
def heur(config, goalConfig):
    return planning.distance(config, goalConfig)

#function to run the A* search algorithm from vertex start to vertex goal.
def runAStar(Start, goal, grid, nodes, adjacency, distances):
    #start with an empty fringe.
    fringe = fring.Fringe()
    Closed = closed.Closed()
  #make sure we're starting at a real grid vertex.
    start = Start 
    start.parent = None #start
    # g is 0 for the start
    start.f = start.h + 0
    fringe.insert(start) 

    while not fringe.empty():
        s = fringe.fringePop()
        if s.x == goal[0] and s.y == goal[1] and s.theta == goal[2]:
            return s
        Closed.insert(s)
# Now go to neighbors of s, check if visited or in fringe.
        updated = False
        #find index of s
        s_ind = -1
        for ind in range(len(nodes)):
            if nodes[ind][0] == s.x and nodes[ind][1] == s.y and s.z == nodes[ind][2]:
                s_ind = ind
                break
        if s_ind == -1: sys.exit("Error: s_ind not found! s={}".format(str(s)))
        for ni in range(len(adjacency[s_ind])): #get k nearest neighbors of s: FIX THIS!!!!!(Which index for adjacency???)!
            config = nodes[adjacency[s_ind][ni]].config #neighbor's configuration
            if not planning.validPath(s.config, config)
                continue
            if not Closed.check(config):
                if not fringe.check(config): #theta):
                    n = anode.Node(config, s, heur(config, goal)
                    fringe.insert(n)
              #update n so its parent is s.
                if fringe.update(s, config, distances[s_ind][ni]):  #What is correct first index for distances?????????????????!!!!!!!!!!
                    updated = True
        if updated:
            fringe.updateFringeHeap()
#if haven't found the goal by the time the fringe is all empty, there is no valid path to the goal.
    return None

def main():
    #create grid, define barriers, etc. 
    #locate start vertex of piano bot and goal vertex.
    """Read the input arguments"""
    mapn = 1
    num = 0
    ros = False
    rrt = False
    nnodes = 50
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "-n" or sys.argv[i] == "-num" or sys.argv[i] == "-number":
            num = int(sys.argv[i+1])
            i=i+1
        elif sys.argv[i] == "-map" or sys.argv[i] == "-m":
            mapn = sys.argv[i+1]
            i = i+1
        elif sys.argv[i] == "-ros" or sys.argv[i] == "-r":
            ros = True
        elif sys.argv[i] == "-RRT" or sys.argv[i] == "-rrt":
            rrt = True
        elif sys.argv[i] == "-nnodes":
            nnodes = int(sys.argv[i+1])
    #runAStar will return goal node if there's a path to goal from start
    if not ros:
        filename = "map_" + str(mapn) + ".txt"
    else: filename = "/home/steven/catkin_ws/src/turtlebot_maps/map_" + str(mapn) + ".txt"
    grid = astar_grid.astarGrid(filename)
    plt.xlim(0, 5)
    plt.ylim(0, 5)
    plt.xticks(np.arange(0, 5, 1))
    plt.yticks(np.arange(0, 5, 1))
    plt.grid(b=True)
    if not rrt:
        twoDnodes, twoDadjacency, twoDdistances = PRM.PianoPRM(nnodes)
        twoDnodes, twoDadjacency, twoDdistances, startIndex, goalIndex = PRM.startAndGoal2DPRM(twoDnodes, twoDadjacency, twoDdistances, (1.1, 2.2, 0, 0, 0, 0), (3.3, 4.4, 0, 0, 0, 0))
        Start = anode.Node(twoDnodes[startIndex].config, 0, None, 0)
        Goal = (twoDnodes[goalIndex].config)
    else:
        #FIX RRT
        twoDnodes, twoDadjacency = RRT.RRT2D(Start, (5,5), nnodes, 0.2)
        twoDdistances = [[0.2 for x in range(nnodes)] for y in range(nnodes)]
    apath = runAStar(Start, Goal, grid, twoDnodes, twoDadjacency, twoDdistances)
    if rrt:
        RRT.RRT2Dshow(twoDnodes, twoDadjacency)
     #   plt.show()
    #NOW MOVE ALONG THE PATH THAT WAS FOUND!
    points = []
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
                import turtlebot_control_client
                 # add points to list to send to turtlebot_control_client
                p = Point(current.config[0], current.config[1], 0)
                points.append(p)
        if not ros: print("Position: x={}, y={}".format(current.config[0], current.config[1]))
        tbx.append(current.config[0])
        tby.append(current.config[1])
        plt.plot(tbx, tby)
       # plt.plot(grid.Start(num).x, grid.Start(num).y, marker='.', color='green', markersize=8)
       # plt.plot(grid.Goal(num)[0], grid.Goal(num)[1], marker='.', color='green', markersize=8)
    else:
        print("Error! No path found!")
    plt.show()
   # PRM.PRM2Dshow(twoDnodes, twoDadjacency, twoDdistances)
    plt.show()
    if ros:
        for p in points:
            print "Sending (%s,%s)"%(p.x, p.y)
            if turtlebot_control_client.TurtlebotControlClient(p).data !=True:
                print("Error! Collsion!")
            else: print "Sent turtlebot to (%s, %s) with no collision."%(p.x, p.y)
main()
