#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,copy,math
from geometry_msgs.msg import Twist
from pimouse_ros.msg import MotorFreqs
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from pimouse_ros.msg import LightSensorValues

class Left_hand():
    def __init__(self):
        rospy.wait_for_service('/timed_motion')
        self.raw = rospy.Publisher('/motor_raw', MotorFreqs,queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def wall_front(self,value = 100):
        return 0.5*(self.sensor_values.left_forward + self.sensor_values.right_forward) > value

    def wall_right(self,value = 100):
        return self.sensor_values.right_side > value

    def wall_left(self,value = 100):
        return self.sensor_values.left_side > value
    
    def move_straight(self):
        #self.motor_timed_motion(400,400,1150)
        rate = rospy.Rate(20)
        for i in range(23):
            if self.sensor_values.left_side > 100:
                e = int(260 - self.sensor_values.left_side)  # 目標値とセンサ値の偏差
	    elif self.sensor_values.right_side > 100:
                e = -int(260 - self.sensor_values.right_side)  # 目標値とセンサ値の偏差
            else:
                e = 0
            self.raw.publish(400-e,400+e)   # 偏差が0になるようにフィードバック制御
            rate.sleep()
	self.raw.publish(0,0)
	#self.motor_timed_motion(0,0,500)            
    def turn_left(self):
        #self.motor_timed_motion(-400,400,465)
    	rate = rospy.Rate(20)
        for i in range(10):
            self.raw.publish(-400,400)
            rate.sleep()
	self.raw.publish(0,0)
        #self.motor_timed_motion(0,0,500)
    def turn_right(self):
        #self.motor_timed_motion(400,-400,465)
	rate = rospy.Rate(20)
        for i in range(10):
            self.raw.publish(400,-400)
            rate.sleep()
	self.raw.publish(0,0)
        #self.motor_timed_motion(0,0,500)
    def turn(self):
        #self.motor_timed_motion(400,-400,465*2)
	rate = rospy.Rate(20)
        for i in range(19):
            self.raw.publish(-400,400)
            rate.sleep()
	self.raw.publish(0,0)
        #self.motor_timed_motion(0,0,500)
    
    def run(self):
	self.raw.publish(0,0)
        self.motor_timed_motion = rospy.ServiceProxy('/timed_motion', TimedMotion)
        self.motor_timed_motion(0,0,1000)        # data.left_hz,data.right_hz,time
        # switch0が押されるまで待機
        switch0 = False
        while switch0 == False:
            with open('/dev/rtswitch0','r') as f:
                switch0 = True if '0' in f.readline() else False
        while not rospy.is_shutdown():
            if not self.wall_left():
                self.turn_left()
                self.move_straight()
            elif not self.wall_front():
                self.move_straight()
            elif not self.wall_right():
                self.turn_right()
                self.move_straight()
            else:
                self.turn()
                self.move_straight()
            

if __name__ == '__main__':
    rospy.init_node('wall_trace')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    p = Left_hand()
    p.run()

