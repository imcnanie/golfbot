import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
import math
from simple_pid import PID
import rosbag
import numpy as np

car_publisher = None
max_speed = 1 # m/s
max_steering = 1.047198 # radians
deadzone = 20

#pid = PID(1.0, 0.0, 0.0, setpoint=1.0)

pid_vecx = PID(20, 0.0, 0.5, setpoint=1.0)
pid_vecy = PID(1.0, 0.0, 0.0, setpoint=1.5)

car_fields = [0,0]

# 'rc' 'record' 'main' 'secondary'
mode = 'rc'

odomain = []
odomains = []

bag = rosbag.Bag('main_track_three.bag')
for topic, msg, t in bag.read_messages(topics=['main_track']):
    odomain.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg])

bag_s = rosbag.Bag('main_track_five.bag')
for topic, msg, t in bag_s.read_messages(topics=['main_track']):
    odomains.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg])

odomainm = odomain
    
# print odomain[50]
bag.close()
bag_s.close()

print "bag done"

bag = rosbag.Bag('main_track.bag', 'w')


lowest_mag = [1000000, odomain[0]]

def odomCallback(msg):
    global car_fields, mode, bag, lowest_mag, odomain

    if True:
        odym = odomain
        if mode == 'main':
            odym = odomainm
        if mode == 'secondary':
            odym = odomains
        
        
        lowest_mag = [1000000, odym[0]]
        for odo in odym:
            magnitude = math.sqrt(((msg.pose.pose.position.x-odo[0])**2) \
                                + ((msg.pose.pose.position.y-odo[1])**2))
            if magnitude < lowest_mag[0]:
                lowest_mag[0] = magnitude
                lowest_mag[1] = odo

        #print msg
        ct_msg = Point32()
        ct_msg.x = lowest_mag[1][0]
        ct_msg.y = lowest_mag[1][1]
        ct_msg.z = 0.5
        ct_publisher.publish(ct_msg)

        index = odym.index(lowest_mag[1])
        is2_msg = Point32()
        if index < len(odym)-50:
            is2_msg.x = odym[index+50][0]
            is2_msg.y = odym[index+50][1]
            is2_msg.z = 0.5
            is2_publisher.publish(is2_msg)

            magnitude = math.sqrt((msg.twist.twist.linear.x**2) + (msg.twist.twist.linear.y**2))
    
        if mode == 'main' or mode == 'rc' or mode == 'secondary':
            vecta_x = msg.pose.pose.position.x
            vecta_y = msg.pose.pose.position.y

            vectb_x = is2_msg.x
            vectb_y = is2_msg.y

            vcx = vecta_x - vectb_x
            vcy = vecta_y - vectb_y

            vc_mag = math.sqrt(vcx**2 + vcy**2)

            vcx_norm = vcx/vc_mag
            vcy_norm = vcy/vc_mag
            
            #2 3 6 7

            rm = msg.pose.covariance

            vdx = msg.pose.covariance[6]
            vdy = msg.pose.covariance[7]

            qtx = vcx*rm[2] + vcy*rm[6]
            qty = vcx*rm[3] + vcy*rm[7]

            
            t1 = math.atan2(qty, qtx)+math.pi/2
            t2 = math.atan2(vdy, vdx)
                        
            vx = vcx
            vy = vcy

            is3_msg = Point32()
            is3_msg.x = vcx
            is3_msg.y = vcy
            is3_msg.z = 0.5
            is3_publisher.publish(is3_msg)
                        
            pid_vecx.setpoint = 0.0

            carf = t1*100+127

            if mode == 'main' or mode == 'secondary':
                car_fields[1] = carf

    
        if mode == 'record':
            bag.write('main_track', msg)
        
        if mode == 'record' or mode == 'main' or mode == 'secondary' or mode == 'rc_pid_speed':

            if car_fields[0] < 127:

                print car_fields[0], magnitude
        
        car_msg = Int32MultiArray()
        car_msg.data = car_fields
        car_publisher.publish(car_msg)

def joyCallback(msg):
    global car_fields, mode, bag
    if msg.buttons[4] == 1:
        mode = 'rc'
        print 'RC MODE'
        bag.close()
    if msg.buttons[1] == 1:
        bag = rosbag.Bag('main_track.bag', 'w')
        mode = 'record'
        print 'RECORD MODE'
    if msg.buttons[2] == 1:
        mode = 'main'
        print 'MAIN FOLLOW TRACK MODE'
    if msg.buttons[3] == 1:
        mode = 'secondary'
        print 'SECONDARY FOLLOW TRACK MODE'
    if msg.buttons[0] == 1:
        mode = 'rc_pid_speed'
        print 'RC PID SPEED MODE'


    speed = int((msg.axes[1]+1.0)*127.0)

    if (speed > 127 - deadzone) and (speed < 127 + deadzone): speed = 127
    
    steer = int((msg.axes[3]+1.0)*127.0)
    if (steer > 127 - deadzone) and (steer < 127 + deadzone): steer = 127

    if mode == 'rc' or mode == 'rc_pid_speed' or mode == 'record':
        car_fields[1] = steer

    if mode == 'rc' or mode == 'main' or mode == 'record' or mode == 'secondary':
        car_fields[0] = speed

if __name__ == '__main__':
    rospy.init_node('Ryan_F_example')
    max_speed = rospy.get_param("~max_speed", 1)
    max_steering = rospy.get_param("~max_steering", 1.047198)
    rospy.Subscriber('joy', Joy, joyCallback)
    rospy.Subscriber('car_odom', Odometry, odomCallback)
    car_publisher = rospy.Publisher('car_target', Int32MultiArray, queue_size=1)
    ct_publisher = rospy.Publisher('/centerPoint', Point32, queue_size=1)
    is2_publisher = rospy.Publisher('/indicatorSphere2', Point32, queue_size=1)
    is3_publisher = rospy.Publisher('/indicatorSphere3', Point32, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()
