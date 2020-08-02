#!/usr/bin/env python
#encoding: utf-8

#motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import rospy,copy,math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class Left_hand():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = 0.0
        while not rospy.is_shutdown():
            data.linear.x = 0.3

            e = 0.5 * (200 - self.sensor_values.left_side)  # 目標値とセンサ値の偏差
            data.angular.z = e * math.pi / 180.0            # 偏差が0になるようにフィードバック制御
	    if self.sensor_values.left_forward > 100 and self.sensor_values.right_forward > 100:
		data.linear.x = 0.0
                data.angular.z = -200 * math.pi / 180.0
            """
            if self.sensor_values.left_side > 200
                e = 1.0 * (200 - self.sensor_values.left_side)
                data.angular.z = e * math.pi / 180.0 
            if self.wall_front(self.sensor_values):
                data.angular.z = - math.pi
            elif self.too_right(self.sensor_values):
                data.angular.z = math.pi
            elif self.too_left(self.sensor_values):
                data.angular.z = - math.pi
            else:
                e = 1.0 * (50 - self.sensor_values.left_side)
                data.angular.z = e * math.pi / 180.0
            """

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_trace')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()
                           
    p = Left_hand()
    p.run()
                                            
