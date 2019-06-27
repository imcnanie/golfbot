#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import serial
import time
ser = serial.Serial('/dev/ttyUSB0',9600)  # open serial port
print(ser.name)         # check which port was really used

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    #print data
    #print int((data.axes[0]+1.0)*127.0)
    ser.write("A"+str(int((data.axes[4]+1.0)*127.0)))
    ser.write("B"+str(int((data.axes[0]+1.0)*127.0)))

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()
    ser.close()
