# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 00:28:17 2018

@author: Mike Nitzsche
"""

import math 
#needs some kind of interfacing to get x and y, but returns an astar node for the goal

class Node:
    parent=None
    h=0
    g=0
    f=0
    x=0
    y=0
    nodeNum=0 #for visibility graph only
    def __init__(self,nodeX,nodeY,nodeParent,heur):
        self.x=nodeX
        self.y=nodeY
        self.h=heur
        if nodeParent is not None:
            self.parent=nodeParent
            self.g=nodeParent.g+math.sqrt((self.x-nodeParent.x)**2+(self.y-nodeParent.y)**2)
            self.f=heur+self.g
            
