#!/usr/bin/env python

import rospy

from firebase import firebase
from std_msgs.msg import String

def get_directions_from_user():
	firebase_handle = firebase.FirebaseApplication('https://autonomous-delivery-bot-default-rtdb.asia-southeast1.firebasedatabase.app', None)
	direction = firebase_handle.get('user/dir', None)
	return direction


def main():
	rospy.init_node('access_database',anonymous=True)

	direction_pub = rospy.Publisher('directions', String, queue_size=10) 

	rate = rospy.Rate(100) # 1Hz
	
	while not rospy.is_shutdown():

		direction = get_directions_from_user()
		rospy.loginfo( direction )

		direction_pub.publish(direction)

		rate.sleep()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')