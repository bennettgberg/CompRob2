import astar_fringe as fring
import astar_node as anode
import astar_closed as closed
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import sys

#return heuristic of start while searching for goal (given by Equation 1 in assignment instructions). Or straight-line distance if running FDA*.
def heur(startx, starty, goalx, goaly, FDA):
    return math.sqrt((startx-goalx)**2 + (starty-goaly)**2)

#function to run the A* search algorithm from vertex start to vertex goal.
def runAStar(Start, goal, grid):
    #start with an empty fringe.
    fringe = fring.Fringe()
    Closed = closed.Closed()
  #make sure we're starting at a real grid vertex.
    startx = round(Start.x / grid.xstep)*grid.xstep
    starty = round(Start.y / grid.ystep)*grid.ystep
    heu = heur(startx, starty, goal[0], goal[1])
    start = anode.Node(startx, starty, None, heu)
    start.parent = None #start
    start.h = heu 
    # g is 0 for the start
    start.f = start.h + 0
    fringe.insert(start) 

    while not fringe.empty():
        s = fringe.fringePop()
        if s.x == goal[0] and s.y == goal[1]:
            return s
        Closed.insert(s)
# Now go to neighbors of s, check if visited or in fringe.
        updated = False
        for nn in s.nearest_neighbors():
            x = nn.x
            y = nn.y
            if grid.checkObstacle(s.x, s.y, x, y):
                continue
            if not Closed.check(x,y):
                if not fringe.check(x,y):
                    n = anode.Node(x, y, s, heur(x, y, goal[0], goal[1]))
                    fringe.insert(n)
              #update n so its parent is s.
                if fringe.update(s, x, y):
                    updated = True
        if updated:
            fringe.updateFringeHeap()
#if haven't found the goal by the time the fringe is all empty, there is no valid path to the goal.
    return None

def main():
    #create grid, define barriers, etc. 
    #locate start vertex of turtlebot and goal vertex.
    """Read the input arguments"""
    mapn = 1
    num = 0
    fda = False
    gweight = 0.5
    ros = False
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "-n" or sys.argv[i] == "-num" or sys.argv[i] == "-number":
            num = int(sys.argv[i+1])
            i=i+1
        elif sys.argv[i] == "-map" or sys.argv[i] == "-m":
            mapn = sys.argv[i+1]
            i = i+1
        elif sys.argv[i] == "-fda" or sys.argv[i] == "-f" or sys.argv[i] == "-FDA" or sys.argv[i] == "-F":
            fda = True 
        elif sys.argv[i] == "-opt" or sys.argv[i] == "-o" or sys.argv[i] == "-optimize":
            gweight = float(sys.argv[i+1])
            if gweight > 1 or gweight < 0: sys.exit("Error: gweight must be between 0 and 1 (inclusive)")
            i = i+1
        elif sys.argv[i] == "-ros" or sys.argv[i] == "-r":
            ros = True
    #runAStar will return goal node if there's a path to goal from start
    if not ros:
        filename = "comprobfall2018-hw1-master/turtlebot_maps/map_" + str(mapn) + ".txt"
    else: filename = "/home/steven/catkin_ws/src/turtlebot_maps/map_" + str(mapn) + ".txt"
    grid = astar_grid.astarGrid(filename)
#    [l.set_visible(False) for (i,l) in enumerate(plt.subplots()[1].xaxis.get_ticklabels()) if i % 4 != 0]
    plt.xlim(grid.xmin, grid.xmax+grid.xstep)
    plt.ylim(grid.ymin, grid.ymax+grid.ystep)
    plt.xticks(np.arange(grid.xmin, grid.xmax, grid.xstep))
    plt.yticks(np.arange(grid.ymin, grid.ymax, grid.ystep))
    plt.grid(b=True)
    for key, obst in grid.obstacleDict.iteritems():
        xes = [obst[0], obst[0], obst[0]+grid.xstep, obst[0]+grid.xstep]
        yes = [obst[1], obst[1]+grid.ystep, obst[1]+grid.ystep, obst[1]] 
        plt.fill(xes, yes, 'blue')
    apath = runAStar(grid.Start(num), grid.Goal(num), grid, fda, gweight)
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
    if ros:
        for p in points:
            print "Sending (%s,%s)"%(p.x, p.y)
            if turtlebot_control_client.TurtlebotControlClient(p).data !=True:
                print("Error! Collsion!")
            else: print "Sent turtlebot to (%s, %s) with no collision."%(p.x, p.y)
main()
