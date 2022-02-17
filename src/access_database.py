#!/usr/bin/env python

import rospy

from firebase import firebase
from std_msgs.msg import String

# Global variables
app_handle = firebase.FirebaseApplication('https://autonomous-delivery-bot-default-rtdb.asia-southeast1.firebasedatabase.app', None)

def get_directions_from_user():
	direction = app_handle.get('user/dir', None)
	return direction

def publish_delivery_info(sender_location_pub, receiver_location_pub):
	sender_location = app_handle.get('delivery/ID-1/location/sender',None)
	receiver_location = app_handle.get('delivery/ID-1/location/receiver',None)
	
	sender_location_pub.publish(sender_location)
	receiver_location_pub.publish(receiver_location)

def set_availability_status(status):
	'''
	status = "yes"/"no"
	'''
	app_handle.put(url='',name='availability',data=status)
	print("Setting availability to",status)
	
def set_delivery_status(status):
	app_handle.put(url='delivery',name='status',data=status)
	print("Setting delivery status to",status)

def main():
	rospy.init_node('access_database',anonymous=True)

	direction_pub = rospy.Publisher('directions', String, queue_size=10) 
	sender_location_pub = rospy.Publisher('sender_location', String, queue_size=10)
	receiver_location_pub = rospy.Publisher('receiver_location', String, queue_size=10)


	rate = rospy.Rate(100) # 1Hz

		
	while not rospy.is_shutdown():

		direction = get_directions_from_user()
		rospy.loginfo( direction )

		direction_pub.publish(direction)
		publish_delivery_info(sender_location_pub, receiver_location_pub)

		rate.sleep()

		set_availability_status()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')