import time
import math
import rospy
import rosbag
import numpy as np
from simple_pid import PID
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

class SteerController():
    def __init__(self):
        self.mode = ''
        self.car_fields = [127,127]
        
        # Joystick
        self.deadzone = 20

        # ROS Bag
        self.odomain_m = []
        bag_m = rosbag.Bag('bags/main_track_five.bag')
        for topic, msg, t in bag_m.read_messages(topics=['main_track']):
            self.odomain_m.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg])

        self.odomain_s = []
        bag_s = rosbag.Bag('bags/track-20190620-190856.bag')
        for topic, msg, t in bag_s.read_messages(topics=['main_track']):
            self.odomain_s.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg])

        bag_m.close()
        bag_s.close()

        timestr = time.strftime("%Y%m%d-%H%M%S")
        self.bag_name = 'bags/track-'+timestr+'.bag'
        
        # ROS
        rospy.init_node('SteerController')
        rospy.Subscriber('joy', Joy, self.joyCallback)
        rospy.Subscriber('car_odom', Odometry, self.odomCallback)
        rospy.Subscriber('/set_mode', String, self.modeCallback)
        
        self.car_pub = rospy.Publisher('/car_target', Int32MultiArray, queue_size=1)
        self.is1_pub = rospy.Publisher('/centerPoint', Point32, queue_size=1)
        self.is2_pub = rospy.Publisher('/indicatorSphere2', Point32, queue_size=1)
        self.is3_pub = rospy.Publisher('/indicatorSphere3', Point32, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    def modeCallback(self, msg):
        self.mode = msg.data
        print "External mode set: " + self.mode + " MODE"
            
    def odomCallback(self, msg):
        sel_odo = self.odomain_m
        if self.mode == 'main':
            sel_odo = self.odomain_m
        if self.mode == 'secondary':
            sel_odo = self.odomain_s

        min_mag = [1000000, sel_odo[0]]

        for odo in sel_odo:
            magnitude = math.sqrt(((msg.pose.pose.position.x-odo[0])**2) \
                                + ((msg.pose.pose.position.y-odo[1])**2))
            if magnitude < min_mag[0]:
                min_mag[0] = magnitude
                min_mag[1] = odo

        is1_msg = Point32()
        is1_msg.x = min_mag[1][0]
        is1_msg.y = min_mag[1][1]
        is1_msg.z = 0.5
        self.is1_pub.publish(is1_msg)

        index = sel_odo.index(min_mag[1])
        is2_msg = Point32()
        if index < len(sel_odo)-50:
            is2_msg.x = sel_odo[index+50][0]
            is2_msg.y = sel_odo[index+50][1]
            is2_msg.z = 0.5
            self.is2_pub.publish(is2_msg)

            magnitude = math.sqrt((msg.twist.twist.linear.x**2) + (msg.twist.twist.linear.y**2))
        else:
            print "End ",
        
        if self.mode == 'main' or self.mode == 'rc' or self.mode == 'secondary':
            # Global Position Vector
            ax = msg.pose.pose.position.x
            ay = msg.pose.pose.position.y
    
            # Advanced Target Vector (target + forward offset)
            bx = is2_msg.x
            by = is2_msg.y
    
            # Vector from car to Target Vector
            cx = ax-bx
            cy = ay-by
            cmag = math.sqrt(cx**2 + cy**2)
    
            # 2D Rotation Matrix
            rm = msg.pose.covariance
    
            # Car local Vector
            dx = msg.pose.covariance[6]
            dy = msg.pose.covariance[7]
    
            # Change target vector to car frame
            ex = cx*rm[2] + cy*rm[6]
            ey = cx*rm[3] + cy*rm[7]
    
            # Target relative to car frame theta 
            t1 = math.atan2(ey, ex)+math.pi/2
    
            carf = t1*100+127
            #print t1
            if self.mode == 'main' or self.mode == 'secondary':
                self.car_fields[1] = carf

        if self.mode == 'record':
            self.bag_r.write('main_track', msg)

        car_msg = Int32MultiArray()
        car_msg.data = self.car_fields
        self.car_pub.publish(car_msg)
            
    def joyCallback(self, msg):
        if msg.buttons[0] == 1:
            self.mode = 'rc_pid_speed'
            print 'RC PID SPEED MODE'
        if msg.buttons[1] == 1:
            # New recording
            self.bag_r = rosbag.Bag(self.bag_name, 'w')
            self.mode = 'record'
            print 'RECORD MODE'
        if msg.buttons[2] == 1:
            self.mode = 'main'
            print 'MAIN FOLLOW TRACK MODE'
        if msg.buttons[3] == 1:
            self.mode = 'secondary'
            print 'SECONDARY FOLLOW TRACK MODE'
        if msg.buttons[4] == 1:
            self.mode = 'rc'
            print 'RC MODE'
            try:
                self.bag_r.close()
                print "Bag saved!"
            except:
                print "Not recording!"
    
        speed = int((msg.axes[1]+1.0)*127.0)        
        if (speed > 127 - self.deadzone) and (speed < 127 + self.deadzone):
            speed = 127
        
        steer = int((msg.axes[3]+1.0)*127.0)
        if (steer > 127 - self.deadzone) and (steer < 127 + self.deadzone):
            steer = 127

        if self.mode == 'rc' or self.mode == 'main' or self.mode == 'record' or self.mode == 'secondary':
            self.car_fields[0] = speed
        
        if self.mode == 'rc' or self.mode == 'rc_pid_speed' or self.mode == 'record':
            self.car_fields[1] = steer

if __name__ == '__main__':
    sc = SteerController()
