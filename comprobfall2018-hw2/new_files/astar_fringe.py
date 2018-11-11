# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import astar_node
import heapq
import math
import astar_grid

class Fringe:
    #object variables for a dictionary and a heap
    fringeDict=None
    fringeHeap=None
    
    def __init__(self):
        self.fringeHeap=[]
        self.fringeDict=dict()
        
    """Checks whether fringeDict is empty"""
    def empty(self):
        if not self.fringeDict:
            return True
        return False

    """Inserts node into fringe heap and open dictionary"""
    def insert(self,node):
        key=str(node.x)+'_'+str(node.y)
        heapKey=node.f
        self.fringeDict[key]=node
        heapq.heappush(self.fringeHeap,(heapKey,node))
        
        
    """Uses the open dictionary to update nodes in the fringe heap/dict
        returns true if it updated, false if it remained the same"""    
    def update(self,parent,x,y, gweight):
        key=str(x)+'_'+str(y)
        updateNode=self.fringeDict[key]
        newG=parent.g + math.sqrt((parent.x-x)**2+(parent.y-y)**2) 
        newF=gweight * newG + (1 - gweight) * updateNode.h 
        if newF<updateNode.f:
            updateNode.g=newG
            updateNode.f=newF
            updateNode.parent = parent
            return True            
        else:
            return False
     
        
    def FDAupdate(self,s,x,y,grid, gweight):   
        key=str(x)+'_'+str(y)
        updateNode=self.fringeDict[key]
        if s.parent is None:
            return False
        parent=s.parent
        updated = False 
        while grid.LineOfSight(parent.x, parent.y, updateNode.x, updateNode.y) and grid.LineOfSight(updateNode.x, updateNode.y, parent.x, parent.y):
            newG = parent.g+math.sqrt((updateNode.x-parent.x)**2 + (updateNode.y-parent.y)**2) 
            newF = gweight * newG + (1 - gweight) * updateNode.h
            if newF < updateNode.f:
                updateNode.g = newG 
                updateNode.f = newF
                updateNode.parent = parent
                updated = True
            parent = parent.parent
            if parent == None: break 
        if not updated: updated = self.update(s, x, y, gweight)
        return updated

    """Checks to see if node for this x,y coordinate is present: 
        if it is, returns True, returns False otherwise"""
    def check(self,x,y):
        key=str(x)+'_'+str(y)
        if key in self.fringeDict:
            return True
        else:
            return False
             
    """Updates the heap after an update"""    
    def updateFringeHeap(self):
        heapq.heapify(self.fringeHeap)
        
    """returns the top node in the heap, removes it from the open list and heap"""    
    def fringePop(self):
        tempNode=heapq.heappop(self.fringeHeap)
        key=key=str(tempNode[1].x)+'_'+str(tempNode[1].y)
        del self.fringeDict[key]
        return tempNode[1]
