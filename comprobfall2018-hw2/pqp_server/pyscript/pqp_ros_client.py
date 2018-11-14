#!/usr/bin/env python

import rospy
from pqp_server.srv import *

def pqp_client(T, R):
    if len(T) != 3 or len(R) != 9:
        print "Incorrect list size for pqp request"
        return True
    rospy.wait_for_service('pqp_server')
    try:
        pqp_server = rospy.ServiceProxy('pqp_server', pqpRequest)
        result = pqp_server(T, R)
        return result
    except rospy.ServiceException, e:
        print "Service Call Failed: %s"%e

def usage():
    return "%s Enter 3 numbers for translation vector, followed by 9 numbers for rotation matrix in row flattend order"%sys.argv[0]

if __name__ == '__main__':
    # T = [3., 4., 2]
    # R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    if len(sys.argv) == 13:
        T = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        R = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]), float(sys.argv[12])]
    else:
        print usage()
        sys.exit(1)

    print "Sending Translation %s, Rotation %s"%(T, R)
    result = pqp_client(T, R)
    print result
