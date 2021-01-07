#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class PIDControl:
    def __init__(self):
        self.data = 0
        self.x = 0
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher('cmd_vel_noisy', Twist, queue_size=1)

    def callback(self,cam_data):
        self.data = cam_data.data

    def loc_callback(self,loc):
        self.x = loc.data

    def record_stop(self, twist):
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        self.pub.publish(twist)
        rospy.sleep(2)
        twist.linear.x = 0.1   
        return True
    
    def publisher_node(self):
        # GEtting the camera data
        cam_subscriber = rospy.Subscriber('color_mono',String,self.callback)
        loc_subscriber = rospy.Subscriber('state',String,self.loc_callback)
        des_pos = 320

        kp = 0.00045 #0.006 #0.005 #0.009 #0.005 
        ki = 0 #0.00005 #0.00005 #0.000009 #0.000005 
        kd = 0 #2 #1.3 #2.1 #1.3 
        derivative = 0
        lasterror = 0 
        integral = 0

        rate = rospy.Rate(10) #10Hz

        stopped1 = False #61
        stopped2 = False #122
        stopped3 = False #244
        stopped4 = False #305

        while not rospy.is_shutdown():
            print(self.x)
            twist = Twist()
            twist.linear.x = 0.1 #0.22 #0.15 
 
            curr_pos = int(self.data)
            error = des_pos - curr_pos

            integral = integral + error
            derivative = error - lasterror
            if abs(integral) > 850:
                integral = 1
            correction = error*kp + integral*ki + derivative*kd
            twist.angular.z = twist.angular.z + correction 
            lasterror = error
       
            # lab 4 related component
            if float(self.x) >= 61 and stopped1 == False:
                stopped1 = self.record_stop(twist)
            elif float(self.x) >= 122 and stopped2 == False:
                stopped2 = self.record_stop(twist)
            elif float(self.x) >= 244 and stopped3 == False:
                stopped3 = self.record_stop(twist)
            elif float(self.x) >= 305 and stopped4 == False:
                stopped4 = self.record_stop(twist)

            self.pub.publish(twist)
            self.cmd_pub.publish(twist)
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
