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
import geometry_msgs.msg
import publisher

path = []
tbx = []
tby = []
poses = []
path.append(anode.Node((5.0, 9.0, 0.4, 1.0, 0.0, 0.0, 0.0), None,0))
#path.append(anode.Node((1.846, 3.482, 2.200, 0.894, 0.177, 0.00356, 0.4106), None, 0))
path.append(anode.Node((4.063, 1.978, 1.070, 0.403,-0.670, 0.18307, 0.5961), None, 0))
path.append(anode.Node((1.5, 1.5, 0.4, 1.0, 0.0, 0.0, 0.0), None, 0))
for i in range(0, 2):
    current = path[i]
    subpath = planning.getPath(path[i].config, path[i+1].config, 5) 
    for j in range(5):
       current = subpath[j] 
       print("Position pos: x={}, y={}".format(current[0], current[1]))

    # add points to list to send to turtlebot_control_client
       q = geometry_msgs.msg.Quaternion(current[4], current[5], current[6], current[3])
       p = geometry_msgs.msg.Point(*current[0:3])
       pose = geometry_msgs.msg.Pose(p, q)
       poses.append(pose)
   
       print("Position: x={}, y={}".format(current[0], current[1]))
       tbx.append(current[0])
       tby.append(current[1])
       plt.plot(tbx, tby)
plt.show()
# PRM.PRM2Dshow(twoDnodes, twoDadjacency, twoDdistances)
# plt.show()
print("poses = {}".format(poses))
# twist = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), geometry_msgs.msg.Vector3(0,0,0))
for pose in poses:
    print "Sending (%s, %s, %s)"%(pose.position.x, pose.position.y, pose.position.z)
    publisher.model_state_publisher(pose, model_name="piano2")
    print "Sent piano to (%s, %s, %s)."%(pose.position.x, pose.position.y, pose.position.z)
