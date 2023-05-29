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

def callback(data):
	twist = Twist()
	twist.linear.x = 4*data.axes[1]
	twist.linear.y = 4*data.axes[0]
	twist.angular.z = 4*data.axes[2]
	pub.publish(twist)

# Intializes everything
def start():
	# publishing to "turtle1/cmd_vel" to control turtle1
	global pub
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
	# subscribed to joystick inputs on topic "joy"
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('Joy2Turtle',anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass
