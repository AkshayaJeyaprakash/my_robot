#!/usr/bin/env python3

from flask import Flask, render_template, request
import rospy
from std_msgs.msg import String
from my_robot.msg import Velocity 
import threading

path = '/home/akshaya/catkin_ws/src/my_robot/scripts/templates'
app = Flask(__name__, template_folder=path)

# Initialize ROS node
rospy.init_node('human_node')
pub = rospy.Publisher('movement_commands', String, queue_size=10)

def velocity_callback(data):
    rospy.loginfo("Current velocities - Linear: %d, Angular: %d", data.linear_velocity, data.angular_velocity)

# ROS Publisher function
def publish_command(command):
    pub.publish(command)

# Flask route for the control page
@app.route("/")
def index():
    return render_template('control.html')  # HTML file with control buttons

# Flask route to handle control commands
@app.route("/command", methods=['POST'])
def command():
    cmd = request.form['command']
    command_mapping = {
        'up': 'w',
        'down': 's',
        'left': 'a',
        'right': 'd',
        'stop': ' '  
    }

    if cmd in command_mapping:
        rospy.loginfo(f"Command: {cmd}")
        publish_command(command_mapping[cmd])
    return ('', 204)


# Separate thread for the ROS subscriber
def ros_thread():
    rospy.Subscriber('velocity', Velocity, velocity_callback)
    rospy.spin()

if __name__ == '__main__':
    thread = threading.Thread(target=ros_thread)
    thread.start()
    app.run(host='0.0.0.0', port=8081, debug=True, use_reloader=False)
