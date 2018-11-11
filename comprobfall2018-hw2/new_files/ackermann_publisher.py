#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState, GetModelState, ModelState

def ackermann_publisher(speed, steering_angle):
	pub = rospy.Publisher("ackermann_cmd", AckermannDrive, self.ackermann_cmd_cb, queue_size=1)
	rospy.init_node('ackermann_controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		state = AckermannDrive()
		state.speed = speed
		state.steering_angle = steering_angle
		rospy.loginfo(state)
		pub.publish(state)
		rate.sleep();

def model_state_publisher(pose, twist):
	pub = rospy.Publisher("/gazebo/set_model_state,", ModelState, queue_size=1)
	rospy.init_node('model_state_controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		model_state = ModelState()
		model_state.model_name = "ackermann_vehicle"
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

if __name__ == '__main__':
	try:
		ackermann_publisher()
	except rospy.ROSInterruptException:
		pass