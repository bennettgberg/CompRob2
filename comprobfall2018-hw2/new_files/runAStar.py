import astar_fringe as fring
import astar_node as anode
import astar_closed as closed
import astar_grid
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import sys
import PRM
import RRT

#return heuristic of start while searching for goal (given by Equation 1 in assignment instructions). Or straight-line distance if running FDA*.
#CHANGE THIS TO USE CORRECT DISTANCE FUNCTION LATER
def heur(startx, starty, starttheta, goalx, goaly, goaltheta):
    return math.sqrt((startx-goalx)**2 + (starty-goaly)**2 + (starttheta-goaltheta)**2)

#function to run the A* search algorithm from vertex start to vertex goal.
def runAStar(Start, goal, grid, nodes, adjacency, distances):
    #start with an empty fringe.
    fringe = fring.Fringe()
    Closed = closed.Closed()
  #make sure we're starting at a real grid vertex.
#    startx = round(Start.x / grid.xstep)*grid.xstep
#    starty = round(Start.y / grid.ystep)*grid.ystep
#    heu = heur(startx, starty, 0, goal[0], goal[1], 0)
  #  start = anode.Node(startx, starty, 0, None, heu)
  #NEED TO ADD START AND GOAL TO NODELISTS. 
    start = anode.Node(nodes[0][0], nodes[0][1], 0, None, 0) #CHANGE TO CLOSEST NODE TO START POSITION!!!!!!!!!  
    start.parent = None #start
    #start.h = heu 
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
#        print adjacency[0]
#        print distances[0]
#        print nodes[0]
        #find index of s
        s_ind = -1
        for ind in range(len(nodes)):
            if nodes[ind][0] == s.x and nodes[ind][1] == s.y:
                s_ind = ind
                break
        if s_ind == -1: sys.exit("Error: s_ind not found! s={}".format(str(s)))
        for ni in range(len(adjacency[s_ind])): #get k nearest neighbors of s: FIX THIS!!!!!(Which index for adjacency???)!
            x = nodes[adjacency[s_ind][ni]][0] #neighbor's x value
            y = nodes[adjacency[s_ind][ni]][1] #neighbor's y value
        #    theta = nn.theta #does neighbor have theta value?
            if grid.checkObstacle(s.x, s.y, x, y):
                continue
            if not Closed.check(x, y, 0): #, theta):
                if not fringe.check(x,y,0): #theta):
                    n = anode.Node(x, y, 0, s, heur(x, y, 0, goal[0], goal[1], goal[2]))
                    fringe.insert(n)
              #update n so its parent is s.
              #  print "s: {} s_ind: {} nodes[s_ind]: {} distances[s_ind]: {} adjacency[s_ind]: {}".format(s, s_ind, nodes[s_ind], distances[s_ind], adjacency[s_ind])
                if fringe.update(s, x, y, 0, distances[s_ind][ni]):  #What is correct first index for distances?????????????????!!!!!!!!!!
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
        twoDnodes, twoDadjacency, twoDdistances = PRM.PRM2D(nnodes)
        new2Dnodes, new2Dadjacency, new2Ddistances, startIndex, goalIndex = PRM.startAndGoal2DPRM(twoDnodes, twoDadjacency, twoDdistances, (1.1, 2.2), (3.3, 4.4))
        Start = anode.Node(new2Dnodes[startIndex][0], new2Dnodes[startIndex][1], 0, None, 0)
        print "Start = " + str(Start.x) + ", " + str(Start.y)
        Goal = (twoDnodes[goalIndex][0], twoDnodes[goalIndex][1], 0)
    else:
        twoDnodes, twoDadjacency = RRT.RRT2D(Start, (5,5), nnodes, 0.2)
        twoDdistances = [[0.2 for x in range(nnodes)] for y in range(nnodes)]
    apath = runAStar(Start, Goal, grid, new2Dnodes, new2Dadjacency, new2Ddistances)
    if rrt:
        RRT.RRT2Dshow(twoDnodes, twoDadjacency)
     #   plt.show()
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
            if not ros: print("Turtlebot pos: x={}, y={}".format(current.x, current.y))
            tbx.append(current.x)
            tby.append(current.y)
            current = path.pop()
            if ros:
                import turtlebot_control_client
                 # add points to list to send to turtlebot_control_client
                p = Point(current.x, current.y, 0)
                points.append(p)
        if not ros: print("Turtlebot pos: x={}, y={}".format(current.x, current.y))
        tbx.append(current.x)
        tby.append(current.y)
        plt.plot(tbx, tby)
        plt.plot(grid.Start(num).x, grid.Start(num).y, marker='.', color='green', markersize=8)
        plt.plot(grid.Goal(num)[0], grid.Goal(num)[1], marker='.', color='green', markersize=8)
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
