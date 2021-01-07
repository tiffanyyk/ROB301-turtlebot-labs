#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
import pandas as pd


if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

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


class BayesLoc:

    def __init__(self, colour_map):
        self.colour_sub = rospy.Subscriber('mean_img_rgb', String, self.measurement_callback)
        self.line_idx_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.colour_map = colour_map
        self.measured_rgb = np.array([0,0,0]) # updated with the measurement_callback
        self.line_idx = 0 # updated with the line_callback with the index of the detected black line.

        # PID settings
        self.des_pos = 320
        self.kp = 0.002
        self.ki = 0.0001
        self.kd = 2.4
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
        self.bound = 160

        # colour map
        self.orange = np.array([225,88,0])
        self.yellow = np.array([235,205,0])
        self.green = np.array([194,221,0])
        self.blue = np.array([8,90,182])
        self.location = ''

        # Bayesian
        self.stateModel = pd.DataFrame(
            np.array([[0.85, 0.05, 0.05], 
                      [0.10, 0.90, 0.10], 
                      [0.05, 0.10, 0.85]]), 
                     index=['X-chi', 'X', 'X+chi'], columns=[-1, 0, 1])
        self.measurementModel = pd.DataFrame(
            np.array([[0.60, 0.20, 0.05, 0.05], 
                      [0.20, 0.60, 0.05, 0.05], 
                      [0.05, 0.05, 0.65, 0.20], 
                      [0.05, 0.05, 0.15, 0.60], 
                      [0.10, 0.10, 0.10, 0.10]]),
            index=['Blue', 'Green', 'Yellow', 'Orange', 'Nothing'], columns=['Blue', 'Green', 'Yellow', 'Orange'])
        self.first = pd.DataFrame(data={'xk': [i for i in range(0,len(colour_map))],                           
                                            'colour': colour_map, 
                                            'initial': [ 1.0/float(len(colour_map)) for i in range(0,len(colour_map)) ]})

 
    def measurement_callback(self, msg):
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.measured_rgb = np.array([r,g,b])
        
    def line_callback(self, data):
        index = int(data.data)
        self.line_idx = index

    def pid_control(self):
        twist = Twist()
        twist.linear.x = 0.05 #0.22 #0.15 

        self.get_colour()	
        print(self.location)

        if self.location != 'line':
            twist.angular.z = 0
        else:
            curr_pos = int(self.line_idx)
            error = self.des_pos - curr_pos
            self.integral = self.integral + error
            self.dervative = error - self.lasterror
            if abs(self.integral) > 850:
                self.integral = 1
            correction = error*self.kp + self.integral*self.ki + self.derivative*self.kd
            twist.angular.z = twist.angular.z + correction 
            self.lasterror = error
        print(self.location) 
        self.cmd_pub.publish(twist)

    def get_colour(self):
        if (self.measured_rgb[2] > 120) and (self.measured_rgb[0] + self.measured_rgb[1] < 150):
            self.location = 'blue'
        elif (self.measured_rgb[2] < 5) and ((self.measured_rgb[1] < 100) and (self.measured_rgb[1] > 50)):
            self.location = 'orange'
        elif (self.measured_rgb[2] < 5) and (self.measured_rgb[1] > self.measured_rgb[0]):
            self.location = 'green'
        elif (self.measured_rgb[2] < 5) and (self.measured_rgb[0] > self.measured_rgb[1]):
            self.location = 'yellow'
        else:
            self.location = 'line'
                                        
    def fillpredictioncolumn(self, k, uk):
        # This function seeks to fill the next prediction column given uk
        # uk is the action done at time k.
    
        # Initialize the empty column. 
        # For each location, predict the probability of it ending up there.
        predictioncolumn = []
        for i in range(0,len(self.colour_map),1):
            prediction_i = self.prediction(i, uk)
            predictioncolumn = predictioncolumn + [prediction_i]
    
        # Add that to the big overall chart
        self.first['prediction'+str(k)] = predictioncolumn
  
    def prediction(self, i, uk):
        # This function for each location, it predict the probability of being at that location
        XminusChi = self.stateProbability('X-chi', uk) * self.getProbability(i + 1)
        X = self.stateProbability('X', uk) * self.getProbability(i)
        XplusChi = self.stateProbability('X+chi', uk) * self.getProbability(i - 1)
        return XminusChi + X + XplusChi
  
    def stateProbability(self, curr_pos, uk):
        return self.stateModel.get_value(curr_pos, uk)
        #.loc[[curr_pos],[uk]].values.tolist()[0][0]

    def getProbability(self, i):
        if i == -1:
            i = len(self.colour_map)-1
        if i == len(self.colour_map):
            i = 0
        return self.first.loc[[i], [self.first.columns[-1]]].values.tolist()[0][0]
  
    def fillupdatecolumn(self, k, measurement):
        total = self.summation(measurement)
        updatecolumn = []
        for i in range(0,len(self.colour_map),1):
            update_i = self.update(measurement, i, total)
            updatecolumn = updatecolumn + [update_i]
        self.first['update'+str(k)] = updatecolumn
  
    def summation(self, measurement):
        total = 0
        for i in range(0,len(self.colour_map),1):
            colour_i = self.first.loc[[i], ['colour']].values.tolist()[0][0]
            prob_i = self.first.loc[[i], [self.first.columns[-1]]].values.tolist()[0][0]
            total = total + (self.measurementProbability(measurement, colour_i) * prob_i)
        return total
  
    def update(self, measurement, i, total):
        colour_i = self.first.loc[[i], ['colour']].values.tolist()[0][0]
        prob_i = self.first.loc[[i], [self.first.columns[-1]]].values.tolist()[0][0]
        update = (self.measurementProbability(measurement, colour_i) * prob_i)/total
        return update

    def measurementProbability(self, measurement, locationcolour):
        return self.measurementModel.loc[[measurement], [locationcolour]].values.tolist()[0][0]                                
  
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
      
    colour_map = ['green', 'orange', 'green', 'yellow', 'blue', 'green', 'orange', 'yellow', 'blue', 'blue', 'orange', 'yellow'] 
                 
    rospy.init_node('bayes_loc')
    BL = BayesLoc(colour_map)
    rospy.sleep(0.5)
    
    ### Initialize your PID controller here ( to merge with the bayes_loc node )
    #PID = PIDcontrol()

    try:
        
        k = 0 # step number
        prediction = False

        while (1):
            BL.pid_control()
            print(k)
            
            BL.get_colour()
            if BL.location != 'line':
                prediction = True
                BL.fillpredictioncolumn(k, 1)
                BL.fillupdatecolumn(k + 1, BL.location) 
            elif BL.location == 'line':
                prediction = False

            key = getKey()
            if (key == '\x03'): #1.22:bayesian.curPos >= 1.6 or
                rospy.loginfo('Finished!')
                break
            
            rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
            rospy.loginfo("Line index: {}".format(BL.line_idx))
                
    # except Exception as e:
    #     print("comm failed:{}".format(e))

    finally:

        ### Stop the robot when code ends
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_publisher.publish(twist)





