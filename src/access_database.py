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
	app_handle.put(url='',name='availability',data=status.data)
	# print("Setting availability to",status.data)

def set_progress_status(status):
	'''
	status = "in progress"/"done"
	'''
	progress_status = status.data
	app_handle.put(url='delivery',name='status',data=progress_status)
	# print("Setting progress status to",progress_status)

	if progress_status == "done":
		print("Setting sender receiver to nil")
		app_handle.put(url='delivery/ID-1/location',name='sender',data="nil")
		app_handle.put(url='delivery/ID-1/location',name='receiver',data="nil")
		
def main():
	rospy.init_node('access_database',anonymous=True)

	# direction_pub = rospy.Publisher('directions', String, queue_size=10) 
	sender_location_pub = rospy.Publisher('sender_location', String, queue_size=10)
	receiver_location_pub = rospy.Publisher('receiver_location', String, queue_size=10)

	availability_status_sub = rospy.Subscriber("availability", String,set_availability_status)
	progress_status_sub = rospy.Subscriber("progress", String,set_progress_status)

	rate = rospy.Rate(1) # 1Hz

		
	while not rospy.is_shutdown():

		# direction = get_directions_from_user()
		# rospy.loginfo( direction )

		# direction_pub.publish(direction)
		publish_delivery_info(sender_location_pub, receiver_location_pub)

		rate.sleep()

		# set_availability_status()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')