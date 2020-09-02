#!/usr/bin/env python
# Software License Agreement (BSD License)
# Modified from https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
# to work with TurtleBot3-Burger

import sys
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import sqrt, atan2, pow, sin, cos

x = 0.0
y = 0.0 
theta = 0.0
move_forward = False

#goal = Point()
#goal.x = input("Put in your target x point: ")
#goal.y = input("Put in your target y point: ")
#tolerance = input("Put in your distance tolerance: ")

if len(sys.argv) != 2:
    print("Usage: rosrun tb_controller tb_controller.py '(x1,y1)(x2,y2)' tolerance")
    sys.exit(1)

coordinate_data = {}
tolerance = sys.argv[2]

for i, pair in enumerate(sys.argv[1].split(")(")):
    filtered_pair = pair.replace("(", "").replace(")", "")
    filtered_pair_array = filtered_pair.split(",")
    coordinate_data[i] = filtered_pair_array
    print(coordinate_data)

# Callback function
def newOdom(msg):
    global x
    global y
    global theta

    #rospy.loginfo("got new data " + str(msg.pose.pose.position))

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Initialize mode
rospy.init_node("speed_controller")

# Subscribe to the odom topic to get information about the current position and velocity
# of the robot
sub = rospy.Subscriber("/odom", Odometry, newOdom)

# Publish linear and angular velocities to cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

# Strategy is to first turn to face the target coordinates
# and then move towards them
while not rospy.is_shutdown():

    for key, value in coordinate_data.items():

        goal = Point()
        goal.x = value[0]
        goal.y = value[1]

        # Compute difference between current position and target position
        inc_x = goal.x - x
        inc_y = goal.y - y
        angle_to_goal = atan2(inc_y, inc_x)
        rospy.loginfo("x coordinate: " + str(x) + " y coordinate  " + str(y))
        rospy.loginfo("velocity: " + str(speed.linear.x))
        rospy.loginfo("angle to our goal " + str(angle_to_goal))

        distance = sqrt(pow(inc_x, 2) + pow(inc_y, 2))

        should_rotate = atan2(sin(angle_to_goal - theta), cos(angle_to_goal - theta))

        if abs(angle_to_goal - theta) < 0.1:
            move_forward = True

        speed.angular.z = 0.25 * should_rotate

        if move_forward:
            if 0.1 * distance > 0.4 and 0.1 * distance < 0.8:
                speed.linear.x = 0.05 * distance
            elif 0.1 * distance > 0.8:
                speed.linear.x = 0.8
            elif distance <= tolerance:
                speed.linear.x = 0
                speed.angular.z = 0
            else:
                speed.linear.x = 0.4

        pub.publish(speed)
        r.sleep()
