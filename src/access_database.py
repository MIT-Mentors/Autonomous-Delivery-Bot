#!/usr/bin/env python

from firebase import firebase
import rospy

from std_msgs.msg import String

def get_directions_from_user():
	firebase_h = firebase.FirebaseApplication('https://autonomous-delivery-bot-default-rtdb.asia-southeast1.firebasedatabase.app', None)
	result = firebase_h.get('user/dir', None)
	return result


def main():
	rospy.init_node('access_database',anonymous=True)

	direction_pub = rospy.Publisher('directions', String, queue_size=10) 

	rate = rospy.Rate(1) # 1Hz
	
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