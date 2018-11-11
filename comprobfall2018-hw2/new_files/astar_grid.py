# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 19:45:15 2018

@author: Mike Nitzsche
"""

import astar_node
import math
import numpy as np
import sys

#nx = 60
#ny = 60
tbd = 0.5 #turtlebot diameter
print "tbd = " + str(tbd)
tbr = tbd/2 #turtlebot radius
xstep = 0.25
ystep = 0.25

obstacles = []

def str_to_xy(string):
    w = string   
    w = w[1:-1]
    xy = w.split(',')
    (x, y) = (float(xy[0]), float(xy[1]))
    return (x, y)

#find line connecting two points, in y=mx+b form (also keep endpoints).
#Return slope m and y-intercept b.
def f(x_1, y_1, x_2, y_2):
    if x_2 == x_1:
        m = float("inf")
        b = float("inf")
    else:
        m = (y_2-y_1) / float(x_2-x_1)
        b = y_1 - m*x_1
    minx = min(x_1, x_2)
    maxx = max(x_1, x_2)
    miny = min(y_1, y_2)
    maxy = max(y_1, y_2)
    return (m, b, minx, maxx, miny, maxy)
    
class astarGrid:
    obstacleDict=None
    starts = []
    goals = []
    xmin=0
    ymin=0
    xmax=0
    ymax=0
    xstep=0
    ystep=0
    goal = None
    def __init__(self, filename):
        self.obstacleDict=dict()
        self.starts = []
        self.goals = []
        #@bennett: write this to deal with importing and interpreting the text file
        mapfile = open(filename, 'r')
        line = mapfile.readline()
        words = line.split()
        if len(words) != 4:
            print("Error! File should only contain 4 corners of the grid.")
            sys.exit()
        """insert obstacles surrounding the grid."""
        (x1, y1) = str_to_xy(words[0])
        self.xmax = x1
        self.ymax = y1
        (x2, y2) = str_to_xy(words[1])
        (x3, y3) = str_to_xy(words[2])
        self.xmin = x3
        self.ymin = y3
        self.xstep = xstep  #(self.xmax-self.xmin) / float(nx)
        self.ystep = ystep  #(self.ymax-self.ymin) / float(ny)
        y = self.ymin
        while y <= self.ymax:
            xy = str(x1) + '_' + str(y)
            self.obstacleDict[xy] = (x1, y)
            y = y + self.ystep
        x = self.xmin
        while x <= self.xmax:                                            
            xy = str(x) + '_' + str(y2)
            self.obstacleDict[xy] = (x, y2)
            x = x + self.xstep
        (x4, y4) = str_to_xy(words[3])
        y = self.ymin
        while y <= self.ymax:                                             
            xy = str(x3) + '_' + str(y)
            self.obstacleDict[xy] = (x3, y)
            y = y + self.ystep
        x = self.xmin
        while x <= self.xmax:
            xy = str(x) + '_' + str(y4)
            self.obstacleDict[xy] = (x, y4)
            x = x + self.xstep
        line = mapfile.readline() #---
        """Read the polygonal vertices"""
        line = mapfile.readline()
        edges = []
        while line != '---\n':
            words = line.split()
            p = len(words)
            for i in range(0, p-1):
                (x1, y1) = str_to_xy(words[i])
                (x2, y2) = str_to_xy(words[i+1])
                edges.append(f(x1, y1, x2, y2))
            (x1, y1) = str_to_xy(words[p-1])
            (x2, y2) = str_to_xy(words[0])
            edges.append(f(x1, y1, x2, y2))
            line = mapfile.readline()

        """now for each x-value along the grid, check if there's a y that should be an obstacle (if it's within 1 turtlebot diameter of a polygon edge).""" 
        x = self.xmin-self.xstep #because it will increase by xstep as soon as the loop starts
        y = self.ymin
        while x <= self.xmax:
            x = x + self.xstep
            y = self.ymin-self.ystep
            while y <= self.ymax:
                y = y + self.ystep
                xy = str(x) + '_' + str(y)
                if xy in self.obstacleDict:
                    continue
                for ed in edges:
                    """if slope of line is infinity"""
                    if ed[0] == float("inf"):
                        if min(abs(x - ed[2]), abs(x + self.xstep - ed[2])) <= tbr and y >= ed[4]-tbr and y+self.ystep  <= ed[5] + tbd:
                            self.obstacleDict[xy] = (x, y)
                            break
                #if slope of line is 0
                    elif ed[0] == 0:
                        if min(abs(y - ed[4]), abs(y + self.ystep - ed[4])) <= tbr and x >= ed[2]-tbr and x+self.xstep <= ed[3] + tbr:
                            self.obstacleDict[xy] = (x, y)
                            break
                    else: 
                        if min(abs(ed[0] * x + ed[1] - y), abs(ed[0] * (x+self.xstep) + ed[1] - y), abs(ed[0] * x + ed[1] - (y+self.ystep)), abs(ed[0] * (x+self.xstep) + ed[1] - (y+self.ystep)) ) <= tbr and x >= ed[2]-tbr and x+self.xstep <= ed[3]+tbr:
                            self.obstacleDict[xy] = (x, y)
                            break

        #now do the same checking for each y-value (in case the slope is too high, some blocks will be skipped if only x's are checked).
        x = self.xmin
        y = self.ymin - self.ystep
        while y <= self.ymax:
            y = y + self.ystep
            x = self.xmin-self.xstep
            while x <= self.xmax:
                x = x + self.xstep
                xy = str(x) + '_' + str(y)
                if xy in self.obstacleDict:
                    continue
                for ed in edges:
                    """if slope of line is infinity then inverse-slope is 0."""
                    if ed[0] == float("inf"):
                        if min(abs(x - ed[2]), abs(x + self.xstep - ed[2])) <= tbr and y >= ed[4]-tbr and y+self.ystep <= ed[5] + tbr:
                            self.obstacleDict[xy] = (x, y)
                            break
                #if slope of line is 0 then inverse-slope is infinity.
                    elif ed[0] == 0:
                        if min(abs(y - ed[4]), abs(y + self.ystep - ed[4])) <= tbr and x >= ed[2]-tbr and x +self.xstep <= ed[3] + tbr:
                            self.obstacleDict[xy] = (x, y)
                            break
                    else: 
                        #y=mx+b => x=(y-b)/m
                        if min(abs(1 / ed[0] * y - ed[1] / ed[0] - x), abs(1 / ed[0] * (y+self.ystep) - ed[1] / ed[0] - x), abs(1 / ed[0] * y - ed[1] / ed[0] - (x+self.xstep)), abs(1 / ed[0] * (y+self.ystep) - ed[1] / ed[0] - x) ) <= tbr and y >= ed[4]-tbr and y+self.ystep <= ed[5]+tbr:
                            self.obstacleDict[xy] = (x, y)
                            break


        """Now read the possible starts and goals."""
        while True:                                      
            line = mapfile.readline()
            if not line:
                break
            words = line.split()
            self.starts.append(str_to_xy(words[0]))
            self.goals.append(str_to_xy(words[1]))
        mapfile.close()
        return

        
    """Returns a tuple of goal coordinates to the main driver function"""    
    def Goal(self, num):     
        #needs to get updated values
        return self.goals[num]
    
    """Returns a start node to the main driver function""" 
    def Start(self, num):
        #gets starting x and y
        x=self.starts[num][0]
        y=self.starts[num][1]
        goalx = self.goals[num][0]
        goaly = self.goals[num][1]
        heur=math.sqrt(2)*min(abs(x-goalx), abs(y-goaly)) + max(abs(x-goalx), abs(y-goaly)) - min(abs(x-goalx), abs(y-goaly))
        return astar_node.Node(x,y,None,heur)
    
    """Returns true if an obstacle is present, False if the path is free"""
    #Note: obstacles are represented by the lower left vertex of their grid cell!
    def checkObstacle(self, startx,starty,endx,endy):
        if startx == endx:
      #only moving up or down--just make sure blocks on either side of path aren't obstacles.
            x1 = startx
            x2 = startx - self.xstep
            y = min(starty, endy)
            xy1 = str(x1) + '_' + str(y)
            xy2 = str(x2) + '_' + str(y)
            if xy1 in self.obstacleDict and xy2 in self.obstacleDict:
                return True
            return False
            
        if starty == endy:
      #only moving left or right; just make sure blocks above and below the path aren't obstacles.
            x = min(startx, endx)
            y1 = starty       
            y2 = starty - self.ystep
            xy1 = str(x) + '_' + str(y1)
            xy2 = str(x) + '_' + str(y2)
            if xy1 in self.obstacleDict and xy2 in self.obstacleDict:
                return True
            return False

      #otherwise, moving through middle of a block. Make sure it isn't blocked.
        x = min(startx, endx)
        y = min(starty, endy)
        xy = str(x) + '_' + str(y)
        if xy in self.obstacleDict:
            return True

        if endx > self.xmax:
            print "No obstacle hmmm startx {} starty {} endx {} endy {}".format(startx, starty, endx, endy)
            print self.obstacleDict
#            sys.exit()
        return False

    def LineOfSight(self,x1,y1,x2,y2):
        f = 0
        dx = x2-x1
        dy = y2-y1
        if dx<0:
            dx = abs(x2-x1)
            sx = -1
        else: sx = 1
        if dy<0:
            dy = abs(y2-y1)
            sy = -1
        else: sy = 1
        
        if dx>= dy:
            while x1!=x2:
                f = f + dy
                if f>=dx:
                    if self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1+self.ystep*(sy-1)/2) or self.checkObstacle(x1,y1,x1+self.xstep*sx, y1+self.ystep*sy): 
                        return False
                    y1 = y1 + self.ystep*sy
                    f = f - dx
                if f!=0 and self.checkObstacle(x1, y1,x1+self.xstep*(sx-1)/2, y1+self.ystep*(sy-1)/2): 
                    return False
                if dy==0 and self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1) and self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1-self.ystep): 
                #if dy==0 and (self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1) or self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1-self.ystep)): 
                    return False
                x1 = x1 + self.xstep*sx
        else:
            while y1!=y2:
                f = f + dx
                if f>=dy:
                    if self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1+self.ystep*(sy-1)/2) or self.checkObstacle(x1,y1,x1+self.xstep*sx, y1+self.ystep*sy):
                        return False
                    x1 = x1 + self.xstep*sx
                    f = f - dy
                if f!=0 and self.checkObstacle(x1,y1,x1+self.xstep*(sx-1)/2, y1+self.ystep*(sy-1)/2): 
                    return False
                if dx==0 and self.checkObstacle(x1,y1,x1,y1+self.ystep*(sy-1)/2) and self.checkObstacle(x1,y1,x1-self.xstep, y1+self.ystep*(sy-1)/2): 
                #if dx==0 and (self.checkObstacle(x1,y1,x1, y1+self.ystep*(sy-1)/2) or self.checkObstacle(x1,y1,x1-1, y1+self.ystep*(sy-1)/2)): 
                    return False
                y1 = y1 + self.ystep*sy
        return True

