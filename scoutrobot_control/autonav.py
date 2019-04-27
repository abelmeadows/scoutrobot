#!/usr/bin/env python
# import rospy
# from std_msgs.msg import String
# import sensor_msgs.msg
# import random
# import numpy as np
# from geometry_msgs.msg import Twist
# from itertools import *
# from operator import itemgetter

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry # from /opt/ros/indigo/share/nav_msgs/cmake
from sensor_msgs.msg import Imu   # from opt/ros/indigo/share/nav_msgs/cmake
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import LaserScan

from itertools import *
from operator import itemgetter
from std_msgs.msg import String
import random
import numpy as np
from itertools import *
from operator import itemgetter

LINX = 0.0 #Always forward linear velocity.
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = 3.14
Kp = 0.05
angz = 0

def LaserScanProcess(msg):
    range_angels = np.arange(len(msg.ranges)) #example: if len(msg.ranges) =5,range_angles = [0,1,2,3,4]
    ranges = np.array(msg.ranges) # create a numpy array of 720 elements i.e 720 scan points
    range_mask = (ranges > THRESHOLD) # stores TRUE or FALSE in an array if value greater/less than THRESHOLD in range_mask
    ranges = list(range_angels[range_mask]) # add range_angels value greater than threshold to a list stored in ranges
    max_gap = 40
    # print(ranges)
    gap_list = []
    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x): #enumerate(ranges) creates a tuple of (index,element) in ranges. The groupby groups similar items together
        gap_list.append(map(itemgetter(1), g))
    gap_list.sort(key=len)
    largest_gap = gap_list[-1]#last element in gap_list
    min_angle, max_angle = largest_gap[0]*((msg.angle_increment)*180/PI), largest_gap[-1]*((msg.angle_increment)*180/PI)
    average_gap = (max_angle - min_angle)/2

    turn_angle = min_angle + average_gap

    print(min_angle, max_angle)
    print(max_gap,average_gap,turn_angle)

    global LINX
    global angz
    if average_gap < max_gap:
        angz = -0.5
    else:
        LINX = 0.5
        angz = Kp*(-1)*(90 - turn_angle)



def main():
    rospy.init_node('autonav')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    takeoff_command = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=1) #(name of topic, message type, queue size - helps subscribers to have space and time to process the message)
    land_command = rospy.Publisher("/ardrone/land", Empty,queue_size=1)
    

    rospy.Subscriber("/laser_scan", LaserScan , LaserScanProcess)
    #rospy.on_shutdown(land_command.publish(Empty()))
    rate = rospy.Rate(10) # 10hz
   

    while not rospy.is_shutdown():
        takeoff_command.publish(Empty())
        
        pos = Twist()
        pos.linear.x = LINX
        pos.angular.z = angz
        pub.publish(pos)
        rate.sleep()

# def takeoff():
#     takeoff_command.publish(Empty())
#     rate.sleep()

# def land():
#     rospy.loginfo("SmartScout Landing")
#     land_command.publish(Empty())

if __name__ == '__main__':
    main()