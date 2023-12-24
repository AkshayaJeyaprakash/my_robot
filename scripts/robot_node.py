#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from my_robot.msg import Velocity

def command_callback(data):
    global linear_velocity, angular_velocity
    if data.data == 'w':
        linear_velocity += 1
    elif data.data == 's':
        linear_velocity -= 1
    elif data.data == 'a':
        angular_velocity += 1
    elif data.data == 'd':
        angular_velocity -= 1
    elif data.data == ' ':
        linear_velocity = 0
        angular_velocity = 0

# def robot_node():
#     global linear_velocity, angular_velocity
#     linear_velocity = 0
#     angular_velocity = 0

#     rospy.init_node('robot_node')
#     pub = rospy.Publisher('velocity', Velocity, queue_size=10)
#     rospy.Subscriber('movement_commands', String, command_callback)
#     rate = rospy.Rate(10)  # 10 Hz

#     while not rospy.is_shutdown():
#         velocity_msg = Velocity()
#         velocity_msg.linear_velocity = linear_velocity
#         velocity_msg.angular_velocity = angular_velocity
#         pub.publish(velocity_msg)
#         rate.sleep()

def robot_node():
    global linear_velocity, angular_velocity
    linear_velocity = 0
    angular_velocity = 0

    rospy.init_node('robot_node')
    velocity_pub = rospy.Publisher('velocity', Velocity, queue_size=10)
    right_pub = rospy.Publisher('right_wheel_speed', Float64, queue_size=10)
    left_pub = rospy.Publisher('left_wheel_speed', Float64, queue_size=10)
    rospy.Subscriber('movement_commands', String, command_callback)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        velocity_msg = Velocity()
        velocity_msg.linear_velocity = linear_velocity
        velocity_msg.angular_velocity = angular_velocity
        l_velocity = linear_velocity
        r_velocity = linear_velocity + angular_velocity
        velocity_pub.publish(velocity_msg)
        right_pub.publish(r_velocity)
        left_pub.publish(l_velocity)
        rate.sleep()


if __name__ == '__main__':
    robot_node()


