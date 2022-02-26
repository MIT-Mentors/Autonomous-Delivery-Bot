#!/usr/bin/env python

import rospy

from firebase import firebase
from std_msgs.msg import String

appHandle = firebase.FirebaseApplication(dsn='https://autonomous-delivery-bot-default-rtdb.asia-southeast1.firebasedatabase.app', authentication=None)

def publishDeliveryInfo(senderLocationPub, receiverLocationPub):
	'''
	Gets the delivery location data from the database and publishes it.
	'''
	senderLocation = appHandle.get('currentDelivery/Sender/Location',None)
	receiverLocation = appHandle.get('currentDelivery/Receiver/Location',None)
	
	senderLocationPub.publish(senderLocation)
	receiverLocationPub.publish(receiverLocation)

def setAvailabilityStatus(status):
	'''
	Sets the availability status of the robot.
	status = "yes"/"no"
	'''
	appHandle.put(url='',name='availability',data=status.data)

def setProgressStatus(status):
	'''
	Sets the status of the current delivery.
	status = "in progress"/"done"
	'''
	progressStatus = status.data
	appHandle.put(url='currentDelivery',name='status',data=progressStatus)

	if progressStatus == "done":
		appHandle.put(url='currentDelivery/Receiver',name='Location',data="nil")
		appHandle.put(url='currentDelivery/Sender',name='Location',data="nil")
		
def main():
	rospy.init_node('access_database',anonymous=True)

	# Publishers
	senderLocationPub = rospy.Publisher('senderLocation', String, queue_size=10)
	receiverLocationPub = rospy.Publisher('receiverLocation', String, queue_size=10)

	# Subscribers
	availabilityStatusSub = rospy.Subscriber("availability", String,setAvailabilityStatus)
	progressStatusSub = rospy.Subscriber("progress", String,setProgressStatus)

	rate = rospy.Rate(10) # 1Hz
		
	while not rospy.is_shutdown():

		publishDeliveryInfo(senderLocationPub, receiverLocationPub)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.loginfo('rospy.ROSInterruptException')