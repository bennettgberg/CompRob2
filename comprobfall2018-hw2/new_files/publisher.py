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

def ackermann_publisher(speed, steering_angle, acceleration, jerk, time_step):
	pub = rospy.Publisher("ackermann_cmd", AckermannDrive, queue_size=1)
	rospy.init_node('ackermann_controller', anonymous=True)

	rate = rospy.Rate(float(1/time_step)) # 1/time_step Hz
	rospy.sleep(0.5)
	while not rospy.is_shutdown():
		
		state = AckermannDrive()
		state.speed = speed
		state.steering_angle = steering_angle
		state.acceleration = acceleration
		state.jerk = jerk

		# rospy.loginfo(state)
		pub.publish(state)
		rate.sleep();

def ackermann_model_state(msg):
    global model_state_x
    global model_state_y
    global model_state_theta

    model_state_x = msg.pose.position.x
    model_state_y = msg.pose.position.y

    model_state_quaternion = msg.pose.orientation
    (model_state_roll,model_state_pitch,model_state_theta) = tf.transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])


def model_state_publisher(pose, twist=geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), geometry_msgs.msg.Vector3(0,0,0))):
	pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
	rospy.init_node('model_state_controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		model_state = ModelState()
		model_state.model_name = "piano2"
		# model_state.model_name = "ackermann_vehicle"
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
		if len(sys.argv) == 3:
			print "Sending to ackermann_cmd speed %s steering_angle %s)"%(sys.argv[1], sys.argv[2])
			ackermann_publisher(sys.argv[1], sys.argv[2])
		else:
			print usage()
			sys.exit(1)
	except rospy.ROSInterruptException:
		pass