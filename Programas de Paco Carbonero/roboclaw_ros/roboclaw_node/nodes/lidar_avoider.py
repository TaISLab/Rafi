#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans



def lid_callback(msg):
    global obstaculo
    global range_min
    global range_max
    global umbral
    
    min_dist = umbral
    
    for i in range(range_min,range_max):
        if msg.ranges[i] < min_dist and msg.ranges[i]> 0.0:
            #print('OBSTACULO DETECTADO')
            obstaculo = True
        else:
            #print('OBSTACULO NO DETECTADO')
            obstaculo = False


def mov_callback(msg):
    global obstaculo
    global pub_vel
    global lin_x
    global lin_y
    global ang_z
    
    print(obstaculo)
    vel=Twist()
    if obstaculo == False:
        vel.linear.x= msg.linear.x
        vel.linear.y= msg.linear.y
        vel.angular.z = msg.angular.z
    
    else: 
        vel.linear.x= lin_x
        vel.linear.y= lin_y
        vel.angular.z = ang_z
    
    pub_vel.publish(vel)
    

def listener():
    rospy.init_node("laser_avoider_node")
    global obstaculo
    global pub_vel
    global range_min
    global range_max
    global umbral
    global lin_x
    global lin_y
    global ang_z
     
    range_min = rospy.get_param("~range_min")
    range_max = rospy.get_param("~range_max")
    umbral = rospy.get_param("~umbral")
    lin_x = rospy.get_param("~lin_x")
    lin_y = rospy.get_param("~lin_y")
    ang_z = rospy.get_param("~ang_z")
    
    obstaculo = False
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    
    
    sub_lid = rospy.Subscriber("/scan", LaserScan, lid_callback)
    sub_vel = rospy.Subscriber("/cmd_vel_joy", Twist, mov_callback)

    rospy.spin()
    
if __name__ == '__main__':
    listener()

3#<launch>
 #   <node pkg="laser_values" type="scan.py" name="scan_values" output="screen">
        
  #  </node>
#</launch>

