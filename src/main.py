#!/usr/bin/env python

import rospy

from std_msgs.msg import String

from webots_ros.srv import set_float,get_float

robot_name = None
left_motor_service = None
right_motor_service = None

def name_parser(model):
	"""
	Identify name of robot.
	"""
	global robot_name

	if 'Turtle' in model.data:
		robot_name = model.data
		print('robot: ',robot_name)

def init_navigation():
	"""
	Initialize motor position and velocity.
	"""
	global left_motor_service, right_motor_service
	
	rospy.wait_for_service(service=robot_name+'/left_wheel_motor/set_velocity')

	try:
		left_motor_service = rospy.ServiceProxy(robot_name+'/left_wheel_motor/set_velocity',set_float)
		right_motor_service = rospy.ServiceProxy(robot_name+'/right_wheel_motor/set_velocity',set_float)

		pos_left_motor = rospy.ServiceProxy(robot_name+'/left_wheel_motor/set_position',set_float)
		pos_right_motor = rospy.ServiceProxy(robot_name+'/right_wheel_motor/set_position',set_float)

		left_motor_service(float(0))
		right_motor_service(float(0))

		pos_left_motor(float('inf'))
		pos_right_motor(float('inf'))
	
	except:
		print("Service call failed: init navigation")

def robot_navigation(left_vel,right_vel):
		"""
		Navigate the robot using given left and right velocities.
		"""
		left_motor_service(float(left_vel))
		right_motor_service(float(right_vel))

def main():
	rospy.init_node('main',anonymous=True)
	rospy.Subscriber('/model_name', String, name_parser)

	left_velocity = 5
	right_velocity = 5

	while True:
		if robot_name != None:
			init_navigation()
			print('Initialised navigation')
			break
	
	while not rospy.is_shutdown():
		robot_navigation(left_velocity,right_velocity)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')
