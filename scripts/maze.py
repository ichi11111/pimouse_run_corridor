#!/usr/bin/env python
#encoding: utf-8

# maze(全探索＋最短経路)

import rospy,copy,math
from geometry_msgs.msg import Twist
from pimouse_ros.msg import MotorFreqs
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from pimouse_ros.msg import LightSensorValues

class Maze():
    def __init__(self):
        rospy.wait_for_service('/timed_motion')
        self.raw = rospy.Publisher('/motor_raw', MotorFreqs,queue_size=1)
        #self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback_lightsensors)
        self.raw.publish(0,0)
        self.motor_timed_motion = rospy.ServiceProxy('/timed_motion', TimedMotion)
        self.motor_timed_motion(0,0,0)        # data.left_hz,data.right_hz,time

    def callback_lightsensors(self,messages):
        self.sensor_values = messages

    def wall_front(self,sensor,value = 200):
        return 0.5*(sensor.left_forward + sensor.right_forward) > value

    def wall_right(self,sensor,value = 200):
        return sensor.right_side > value

    def wall_left(self,sensor,value = 200):
        return sensor.left_side > value
    
    def move_straight(self,mode=0):
        if mode == 1:
            time = 24
            hz =800
        else:
            time = 46
            hz =400          
        rate = rospy.Rate(40)
        for i in range(time):
            if self.sensor_values.left_side > 300:
                e = int(0.3*(800 - self.sensor_values.left_side))  # 目標値とセンサ値の偏差
            elif self.sensor_values.right_side > 300:
                e = -int(0.3*(600 - self.sensor_values.right_side))  # 目標値とセンサ値の偏差
            else:
                e = 0
            self.raw.publish(hz-e,hz+e)   # 偏差が0になるようにフィードバック制御
            rate.sleep()
        self.raw.publish(0,0) 
    
    def turn_left(self,mode=0):
        if mode == 1:
            time = 9
            hz =800
        else:
            time = 19
            hz =400 
        rate = rospy.Rate(40)
        for i in range(time):
            self.raw.publish(-hz,hz)
            rate.sleep()
        self.raw.publish(0,0)
        
    def turn_right(self,mode=0):
        if mode == 1:
            time = 9
            hz =800
        else:
            time = 19
            hz =400 
        rate = rospy.Rate(40)
        for i in range(time):
            self.raw.publish(hz,-hz)
            rate.sleep()
        self.raw.publish(0,0)
        
    def turn(self,mode=0):
        if mode == 1:
            time = 18
            hz =800
        else:
            time = 38
            hz =400 
        rate = rospy.Rate(40)
        for i in range(time):
            self.raw.publish(-hz,hz)
            rate.sleep()
        self.raw.publish(0,0)

# switch0が押されるまで待機する関数
    def wait_switch0(self):
        switch0 = False
        while switch0 == False and (not rospy.is_shutdown()):
            with open('/dev/rtswitch0','r') as f:
                switch0 = True if '0' in f.readline() else False

# 等高線マップ作成関数
    def h_map_func(self,wall,goal_x,goal_y):
        x_length = len(wall)
        y_length = len(wall[0])
        h_map = [[1000 for _ in range(y_length)] for _ in range(x_length)]
        h_map[goal_x][goal_y] = 0
        que = [[goal_x,goal_y]]
        tmp_x, tmp_y = goal_x, goal_y

        while que != []:
            posi = que.pop(0)
            tmp_x, tmp_y = posi[0],posi[1]
            if (wall[tmp_x][tmp_y] & 0b1000) == 0b1000:
                if h_map[tmp_x][tmp_y+1] > h_map[tmp_x][tmp_y] + 1:
                    h_map[tmp_x][tmp_y+1] = h_map[tmp_x][tmp_y] + 1
                    que.append([tmp_x,tmp_y+1])
            if (wall[tmp_x][tmp_y] & 0b0100) == 0b0100:
                if h_map[tmp_x+1][tmp_y] > h_map[tmp_x][tmp_y] + 1:
                    h_map[tmp_x+1][tmp_y] = h_map[tmp_x][tmp_y] + 1
                    que.append([tmp_x+1,tmp_y])
            if (wall[tmp_x][tmp_y] & 0b0010) == 0b0010:
                if h_map[tmp_x][tmp_y-1] > h_map[tmp_x][tmp_y] + 1:
                    h_map[tmp_x][tmp_y-1] = h_map[tmp_x][tmp_y] + 1
                    que.append([tmp_x,tmp_y-1])
            if (wall[tmp_x][tmp_y] & 0b0001) == 0b0001:
                if h_map[tmp_x-1][tmp_y] > h_map[tmp_x][tmp_y] + 1:
                    h_map[tmp_x-1][tmp_y] = h_map[tmp_x][tmp_y] + 1
                    que.append([tmp_x-1,tmp_y])
        return h_map
    
# 目的地まで移動する関数
    def navigation_func(self, wall, h_map, dest_x, dest_y,mode):
        reach = False
        while reach == False and (not rospy.is_shutdown()):
            sensor = self.sensor_values
            q = []
            cos = int(math.cos(0.5*math.pi*self.head))
            sin = int(math.sin(0.5*math.pi*self.head))
            if not self.wall_left(sensor):
                q.append([self.x - cos, self.y + sin])
            if not self.wall_front(sensor):
                q.append([self.x + sin, self.y + cos])
            if not self.wall_right(sensor):
                q.append([self.x + cos, self.y - sin])

            tmp_h = h_map[self.x][self.y]
            tmp_posi = [self.x, self.y]
            for i in range(len(q)):
                if h_map[q[i][0]][q[i][1]] < tmp_h:
                    tmp_h = h_map[q[i][0]][q[i][1]]
                    tmp_posi = [q[i][0], q[i][1]]

            if tmp_posi == [self.x - cos, self.y + sin]:
                self.turn_left(mode)
                self.move_straight(mode)
                self.x = self.x - cos
                self.y = self.y + sin
                self.head = (self.head-1)%4
            elif tmp_posi == [self.x + sin, self.y + cos]:
                self.move_straight(mode)
                self.x = self.x + sin
                self.y = self.y + cos
                self.head = (self.head-0)%4
            elif tmp_posi == [self.x + cos, self.y - sin]:
                self.turn_right(mode)
                self.move_straight(mode)
                self.x = self.x + cos
                self.y = self.y - sin
                self.head = (self.head+1)%4

            if self.x == dest_x and self.y == dest_y:
                reach = True
            
# 足立法
    def run(self): 
        # 変数設定
        x_length = 5  # 横幅
        y_length = 3  # 縦幅
        wall = [[0 for _ in range(y_length)] for _ in range(x_length)]         # 壁情報配列作成 (0:壁あり,1:壁なし)(****:北東南西)
        passed_cell = [[0 for _ in range(y_length)] for _ in range(x_length)]  # 通過確認配列作成(0:未通過,1:通過済)
        all_passed = [[1 for _ in range(y_length)] for _ in range(x_length)]   # 全通過判定用配列
        h_map = [[1000 for _ in range(y_length)] for _ in range(x_length)]     # 等高線マップ
        self.x, self.y = 0,0         # 現在のxy座標
        start_x, start_y = 0,0     # スタートのxy座標
        goal_x, goal_y = 5-1,3-1   # ゴールのxy座標
        self.head = 0  # 姿勢(0:北,1:東,2:南,3:西)
        mode = 0  # 0:探索中, 1:最短距離移動中, 2:ゴール
        return_flag = 0 #リターン中かどうか

        # 変数初期化
        passed_cell[start_x][start_y] = 1   

        # switch0が押されるまで待機
        self.wait_switch0()

        # 探索モード
        while mode == 0 and (not rospy.is_shutdown()):  
            sensor = self.sensor_values
            # 現在地の壁情報記録
            if not self.wall_front(sensor):
                wall[self.x][self.y] |= (0b1000 >> ((0+self.head)%4))
            if not self.wall_right(sensor):
                wall[self.x][self.y] |= (0b1000 >> ((1+self.head)%4))
            if not self.wall_left(sensor):
                wall[self.x][self.y] |= (0b1000 >> ((3+self.head)%4))
            # 背後の壁情報記録
            if not (self.x==0 and self.y==0):
                wall[self.x][self.y] |= (0b1000 >> ((2+self.head)%4))

            # 探索(拡張左手法)
            cos = int(math.cos(0.5*math.pi*self.head))
            sin = int(math.sin(0.5*math.pi*self.head))
            if not self.wall_left(sensor) and passed_cell[self.x - cos][self.y + sin]==0:
                return_flag = 0 
                self.turn_left()
                self.move_straight()
                self.x = self.x - cos
                self.y = self.y + sin
                self.head = (self.head-1)%4
                passed_cell[self.x][self.y] = 1
            elif not self.wall_front(sensor) and passed_cell[self.x + sin][self.y + cos]==0:
                return_flag = 0 
                self.move_straight()
                self.x = self.x + sin
                self.y = self.y + cos
                self.head = (self.head-0)%4
                passed_cell[self.x][self.y] = 1
            elif not self.wall_right(sensor) and passed_cell[self.x + cos][self.y - sin]==0:
                return_flag = 0 
                self.turn_right()
                self.move_straight()
                self.x = self.x + cos
                self.y = self.y - sin
                self.head = (self.head+1)%4
                passed_cell[self.x][self.y] = 1
            else:
                if return_flag == 0:  #リターン中でない場合
                    return_flag = 1
                    self.turn()
                    self.move_straight()
                    self.x = self.x - sin
                    self.y = self.y - cos
                    self.head = (self.head+2)%4
                else:                 #リターン中の場合
                    if not self.wall_left(sensor):
                        self.turn_left()
                        self.move_straight()
                        self.x = self.x - cos
                        self.y = self.y + sin
                        self.head = (self.head-1)%4
                    elif not self.wall_front(sensor):
                        self.move_straight()
                        self.x = self.x + sin
                        self.y = self.y + cos
                        self.head = (self.head-0)%4
                    elif not self.wall_right(sensor):
                        self.turn_right()
                        self.move_straight()
                        self.x = self.x + cos
                        self.y = self.y - sin
                        self.head = (self.head+1)%4
                    else:
                        self.turn()
                        self.move_straight()
                        self.x = self.x - sin
                        self.y = self.y - cos
                        self.head = (self.head+2)%4

            if passed_cell == all_passed:
                wall[self.x][self.y] |= (0b1000 >> ((2+self.head)%4))  # 背後の壁情報記録
                with open("/home/ubuntu/test/wall",'w') as f:
                    f.write(str(wall))  
                h_map = self.h_map_func(wall,start_x,start_y)   # 等高線マップ計算   
                self.turn()
                self.head = (self.head+2)%4
                mode = 2
                
        #スタート地点まで戻る
        h_map = self.h_map_func(wall,start_x,start_y)   # 等高線マップ計算
        self.navigation_func(wall, h_map, start_x, start_y,mode)
        self.turn()
        self.head = (self.head+2)%4
        mode = 1

	while mode==1 and (not rospy.is_shutdown()):                
      	    # switch0が押されるまで待機
            self.wait_switch0()
                
            # 最短距離移動モード
	    self.x, self.y = 0,0
            self.head = 0
	    h_map = self.h_map_func(wall,goal_x,goal_y)   # 等高線マップ計算
            with open("/home/ubuntu/test/h_map",'w') as f:
                f.write(str(h_map))
            self.navigation_func(wall, h_map, goal_x, goal_y,mode)

if __name__ == '__main__':
    rospy.init_node('maze')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    p = Maze()
    p.run()
