#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import geometry_msgs.msg

sys.path.append('../ackermann/ackermann_vehicle_gazebo/nodes/')
import ackermann_controller

def ackermann_publisher(speed, steering_angle, steering_angle_velocity, acceleration, jerk, time_step):
	pub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=1)
	# rospy.init_node('ackermann_controller', anonymous=True)

	rate = rospy.Rate(float(1/time_step)) # 1/time_step Hz
	print("hello")
	rospy.sleep(1.0) #0.5)

	while not rospy.is_shutdown():
		
		state = AckermannDrive()
		state.steering_angle = steering_angle
		state.steering_angle_velocity=steering_angle_velocity
		state.speed = speed
		state.acceleration = acceleration
		state.jerk = jerk

#		rate.sleep()
#
#		state.steering_angle = 0
#		state.steering_angle_velocity=0
#		state.speed = 0
#		state.acceleration = 0
#		state.jerk = 0
#
		rospy.loginfo(state)
		print("publishing state")
		pub.publish(state)
		print("state published, sleeping for []".format(rate))
		rate.sleep()
	return

def model_state_publisher(pose, twist=geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), geometry_msgs.msg.Vector3(0,0,0)), model_name="piano2"):
	pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
	rospy.init_node('model_state_controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		model_state = ModelState()
		model_state.model_name = model_name
		model_state.pose = pose
		model_state.twist = twist
		model_state.reference_frame = "world"
		# model_state.pose.orientation.x = tf.transformations.quaternion_from_euler(0,0,steering_angle)[0]
		# model_state.pose.orientation.y = tf.transformations.quaternion_from_euler(0,0,steering_angle)[1]
		# model_state.pose.orientation.z = tf.transformations.quaternion_from_euler(0,0,steering_angle)[2]
		# model_state.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,steering_angle)[3]
		rospy.loginfo(model_state)
		pub.publish(model_state)
		rate.sleep()
		break

def usage():
	return "%s Enter two numbers speed and steering_angle"%sys.argv[0]
 
if __name__ == '__main__':
	try:
		if len(sys.argv) == 4:
			# print "Sending to ackermann_cmd speed %s steering_angle %s)"%(sys.argv[1], sys.argv[2])
			# ackermann_publisher(sys.argv[1], sys.argv[2])
			q = geometry_msgs.msg.Quaternion(0.0,0.0,0.0,1.0)
			p = geometry_msgs.msg.Point(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
			pose = geometry_msgs.msg.Pose(p, q)
			model_state_publisher(pose, model_name="piano2")
		else:
			print usage()
			sys.exit(1)
	except rospy.ROSInterruptException:
		pass
