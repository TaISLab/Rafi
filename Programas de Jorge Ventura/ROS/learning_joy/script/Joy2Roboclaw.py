#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
twist = Twist()
g = 0.3

def callback(data):

	global twist

	twist.linear.x = ((data.axes[5] - 1)*g - (data.axes[4] - 1)*g + 2.2*g)*data.axes[1]
	twist.linear.y = ((data.axes[5] - 1)*g - (data.axes[4] - 1)*g + 2.2*g)*data.axes[0]
	twist.angular.z = ((data.axes[5] - 1)*g - (data.axes[4] - 1)*g + 2.2*g)*data.axes[2]
	

# Intializes everything
def start():
	
	global twist

	# publishing to "cmd_vel"
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)

	# starts the node
	rospy.init_node('Joy2Roboclaw',anonymous=True)

	rate = rospy.Rate(30)

	while not rospy.is_shutdown():

		pub.publish(twist)
		rate.sleep()
		
		

if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
