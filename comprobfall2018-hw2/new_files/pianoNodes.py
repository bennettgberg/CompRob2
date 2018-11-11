import PRM
import numpy as np

class pianoNode:
    parent=None
    #constructor: takes a tuple with (x,y,z), a tuple of quaternion coordinates
    def __init__(self,xyz,quatCoords):
        self.xyz=xyz
        self.quatCoords=quatCoords
    
    #adds a neighboring node:
    #stores the neighbor pointer and the distance between them
    #then adds the node and distance to the other node's neighbor list
    def addNeighbor(self, otherNode):
        self.neighbors.append(otherNode)
        dist=self.nodeDistance(otherNode)
        self.distances.append(dist)
        otherNode.neighbors.append(self)
        otherNode.distances.append(dist)
        
    #returns the distance between two nodes:
    #here, our distance is the time it takes to travel
    #found in euclidean and in quaternion
    def nodeDistance(self,otherNode):
        #velocity is linear velocity: need to scale this later
        v=1
        qd=PRM.quatDistance(self.quatCoords,otherNode.quatCoords)
        ed=PRM.euclideanDistance(self.xyz,otherNode.xyz)
        return qd+ed
        