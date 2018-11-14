# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 00:24:19 2018

@author: Mike Nitzsche
"""
import planning
#makes and maintains a closed dictionary
class Closed:
    closedDict=None;
    """Inserts a given node into the closed list"""
    def insert(self,node):
        ind=planning.key(node.config)
        self.closedDict[ind]=node
        
    """Checks whether a given node has been inserted into the closed list 
       (returns true) or not (returns false)"""    
    def check(self,x,y,z,w,i,j,k):#node):
        ind=planning.key((x,y,z,w,i,j,k)) 
        if ind in self.closedDict:
            return True
        else:
            return False
        
    def __init__(self):
        self.closedDict=dict()
