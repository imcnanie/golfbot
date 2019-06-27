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
            # or GPS reported speed
            magnitude = math.sqrt((msg.twist.twist.linear.x**2) + (msg.twist.twist.linear.y**2))
    
        #print msg.pose.pose.orientation.x
        #print lowest_mag[1][2].pose.pose.orientation.x


        if mode == 'main' or mode == 'rc' or mode == 'secondary':
            # TODO: Somehow get rotation to line up with vectors
            
            #vec_x = lowest_mag[1][2].pose.pose.orientation.x - msg.pose.pose.orientation.x
            #vec_y = lowest_mag[1][2].pose.pose.orientation.y - msg.pose.pose.orientation.y

            #vect2x = lowest_mag[1][2].pose.pose.position.x - msg.pose.pose.position.x
            #vect2y = lowest_mag[1][2].pose.pose.position.y - msg.pose.pose.position.y

            #vect2_mag = math.sqrt((vect2x**2) + (vect2y**2))

            #vect2x = vect2x/vect2_mag
            #vect2y = vect2y/vect2_mag
            
            #pid_vecx.setpoint = lowest_mag[1][2].pose.pose.orientation.x #+ vect2y
            #pid_vecy.setpoint = lowest_mag[1][2].pose.pose.orientation.y #+ vect2x

            #sp_mag = math.sqrt(pid_vecx.setpoint**2 +  pid_vecy.setpoint**2)

            #pid_vecx.setpoint = pid_vecx.setpoint/sp_mag
            #pid_vecy.setpoint = pid_vecy.setpoint/sp_mag
            
            #outx = pid_vecx(msg.pose.pose.orientation.x)
            #outy = pid_vecy(msg.pose.pose.orientation.y)

            
            #relvect_x = lowest_mag[1][2].pose.pose.orientation.x - msg.pose.pose.orientation.x
            #relvect_y = lowest_mag[1][2].pose.pose.orientation.y - msg.pose.pose.orientation.y

            #relvect_mag = math.sqrt(relvect_x**2 + relvect_y**2)

            #relvect_x = relvect_x/relvect_mag
            #relvect_y = relvect_y/relvect_mag

            vecta_x = msg.pose.pose.position.x
            vecta_y = msg.pose.pose.position.y

            vectb_x = is2_msg.x
            vectb_y = is2_msg.y

            #ox = vectb_x - ct_msg.x  
            #oy = vectb_x - ct_msg.y 

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
            
            #print t1, t2 #math.atan2(math.sin(t2-t1), math.cos(t2-t1)),vcx_norm*vdx + vcy_norm*vdy
            
            vx = vcx
            vy = vcy
            

            is3_msg = Point32()
            is3_msg.x = vcx
            is3_msg.y = vcy
            is3_msg.z = 0.5
            is3_publisher.publish(is3_msg)
            
            #theta_car = math.atan2(msg.pose.pose.orientation.y,msg.pose.pose.orientation.x)
            #theta_target = math.atan2(vectc_y,vectc_x)

            #car_fields[1] = (theta_target+theta_car)*500+127
            #carf = (theta_target + theta_car)*10+127
            
            #theta_car = math.atan(msg.pose.pose.orientation.y/msg.pose.pose.orientation.x)
            #theta_target = math.atan(lowest_mag[1][2].pose.pose.orientation.y/lowest_mag[1][2].pose.pose.orientation.x)

            #pv = math.atan(relvect_y/relvect_x) - theta_car
            
            
            pid_vecx.setpoint = 0.0

            #car_fields[1] = (theta_target+theta_car)*100+127
            #carf = (theta_target+theta_car)*10+127

            carf = t1*100+127
            #print carf
            if mode == 'main' or mode == 'secondary':
                car_fields[1] = carf

            #a = np.matrix(msg.pose.covariance[1:13])
            #a.shape = (4,3)
            #print(" ")
            # 1, 2, 4, 5 rotate around "z"
            #aa = [2,3,6,7]
            
            #my_list = [msg.pose.covariance[x] for x in aa]
            #print(['%.2f' % elem for elem in my_list ])
            
            #print theta_target+theta_car, car_fields[1]
            #print theta_target, theta_car, carf
            #print msg.pose.covariance[1:4]
            #print msg.pose.covariance[4:7]
            #print msg.pose.covariance[8:11]
            #print msg.pose.covariance[12:15]
            
            #car_fields[1] = -outx*20+127

            #print theta_target, theta_car, theta_target + theta_car, car_fields[1],  carf
            
            #car_fields[1] = (pid_rot(cur_theta)*15+127)
            
            #setpoint_theta = msg.pose.pose.orientation.x #math.atan(vec_y/vec_x)
            #cur_theta = lowest_mag[1][2].pose.pose.orientation.x #math.atan(msg.pose.pose.orientation.y/msg.pose.pose.orientation.x)
            
            #pid_rot.setpoint = 0.0
            #car_fields[1] = (pid_rot(cur_theta)*15+127)
            #print pid_vecx.setpoint, msg.pose.pose.orientation.x #, setpoint_theta
    
        if mode == 'record':
            bag.write('main_track', msg)
        
        if mode == 'record' or mode == 'main' or mode == 'secondary' or mode == 'rc_pid_speed':
            #car_fields[0] = (pid_vecy(magnitude)*10+127)
            if car_fields[0] < 127:
                #car_fields[0] = 127
                #pid_vecy.setpoint = 1.0
                print car_fields[0], magnitude
    
        #print(magnitude, car_fields[0])
        
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

            
    #print msg.axes
    #car_msg = Int32MultiArray()
    speed = int((msg.axes[1]+1.0)*127.0)
    #print speed
    if (speed > 127 - deadzone) and (speed < 127 + deadzone): speed = 127
    
    steer = int((msg.axes[3]+1.0)*127.0)
    if (steer > 127 - deadzone) and (steer < 127 + deadzone): steer = 127

    if mode == 'rc' or mode == 'rc_pid_speed' or mode == 'record':
        car_fields[1] = steer

    if mode == 'rc' or mode == 'main' or mode == 'record' or mode == 'secondary':
        car_fields[0] = speed
    
    #car_msg.data[1] = [speed,steer]
    #car_publisher.publish(car_msg)

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
