#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class PIDControl:
    def __init__(self):
        self.data = 0 

    def callback(self,cam_data):
        self.data = cam_data.data

    def publisher_node(self):
        # GEtting the camera data
        cam_subscriber = rospy.Subscriber('color_mono',String,self.callback)
        des_pos = 320
        rospy.loginfo(des_pos)

        kp = 0.006 #0.005 #0.009 #0.005 
        ki = 0.00005 #0.00005 #0.000009 #0.000005 
        kd = 1.3 #2 #1.3 #2.1 #1.3 
        lasterror = 0 
        integral = 0
        derivative = 0

        cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.05 #0.22 #0.15 
 
            curr_pos = int(self.data)
            error = des_pos - curr_pos

            integral = integral + error
            derivative = error - lasterror
            if abs(integral) > 850:
                integral = 1

            correction = error*kp + integral*ki + derivative*kd
            twist.angular.z = twist.angular.z + correction
            print(error, twist.linear.x, twist.angular.z) 
            lasterror = error
        
            cmd_pub.publish(twist)
            rate.sleep()

    def main(self):
        try:
            rospy.init_node('motor')
            self.publisher_node()
        except rospy.ROSInterruptException:
            pass
    

if __name__ == '__main__':
    pid = PIDControl()
    pid.main()
