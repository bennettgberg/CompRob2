# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 00:24:19 2018

@author: Mike Nitzsche
"""
#makes and maintains a closed dictionary
class Closed:
    closedDict=None;
    """Inserts a given node into the closed list"""
    def insert(self,node):
        key=str(node.x)+'_'+str(node.y) + '_' + str(node.theta)
        self.closedDict[key]=node
        
    """Checks whether a given node has been inserted into the closed list 
       (returns true) or not (returns false)"""    
    def check(self,x,y,theta):#node):
        key=str(x)+'_'+str(y) + '_' + str(theta)
        if key in self.closedDict:
            return True
        else:
            return False
        
    def __init__(self):
        self.closedDict=dict()
    
