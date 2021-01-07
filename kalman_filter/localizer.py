#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h # y distance to tower
        self.d = d # x distance to tower initially 

        self.Q = Q # Process noise covariance
        self.R = R # Measurement noise covariance
        self.P = P_0 # Initial state Covariance
        self.x = x_0 # Initial state position

        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        self.position = []
        self.covariance = []
        self.t = 0
        self.time = []
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scan_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, u = 0):
        # update state via the motion model, and update the covariance with the process noise
        self.x = self.x + u*100*(1.0/30.0)
        return self.x

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self):
        # update state when a new measurement has arrived using this function
        measurement = np.arctan2(self.h*100, self.d*100 - self.x)
        return measurement

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi
        self.t = self.t + 1.0/30.0

        D = self.h*100 / ((self.h*100)**2 + (self.x - self.d*100)**2)
        self.P = self.P + self.Q
        self.x = self.predict(current_input)
        measurement = self.measurement_update()	

        if math.isnan(current_measurement):
            self.x = self.x
            self.P = self.P
        else:
            S = (D*self.P*D) + self.R
            W = (self.P * D)/S
            self.P = self.P - (W*S*W)
            self.x = self.x + W*(current_measurement -measurement)
             
        # update the state with current_input and current_measurement
        rospy.loginfo(self.x)
        self.time.append(self.t)
        self.covariance.append(self.P)
        self.position.append(self.x)

        # save plot 
        if self.x > 310:
            plt.plot(self.time, self.covariance, self.time, self.position)
            plt.savefig("plot.png")
        
        self.state_pub.publish(str(float(self.x)))
	

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 0.6  # y distance to tower
        d = 1.5  # x distance to tower (from origin)  
        
        x_0 = 0  # initial state position
        
        Q = 1  # process noise covariance
        R = 1  # measurement noise covariance
        P_0 = 1  # initial state covariance 
        kf = KalmanFilter(h, d, x_0, Q, R, P_0)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            kf.run_kf()  
            rate.sleep()
            
    #except:
        #print(e)

    finally:
        rospy.loginfo("goodbye")

