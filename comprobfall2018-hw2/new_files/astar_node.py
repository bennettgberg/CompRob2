# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 00:28:17 2018

@author: Mike Nitzsche
"""

import math 
import planning
#needs some kind of interfacing to get x and y, but returns an astar node for the goal

class Node:
    parent=None
    h=0
    g=0
    f=0
    config=(0,0,0,0,0,0,0)
    nodeNum=0 #for visibility graph only
    def __init__(self,nodeConfig,nodeParent,heur):
        self.config=nodeConfig
        self.h=heur
        if nodeParent is not None:
            self.parent=nodeParent
            self.g=nodeParent.g + planning.distance(self.config, self.parent.config)
            self.f=heur+self.g
