#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf
import math

from math import pi
from xml.etree.ElementTree import PI #なんでPI？

#######added by suetake 2022/02/17###############
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
##################################################

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from nav_msgs.msg import _Odometry


#######added by okubo 2022/02/22###############
from detect_enemy import EnemyDetector
from params import *
from move_pole import MovePole
from escape import select_escape_point
##################################################


# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # subscriber
        # self.odom = rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=1)
        #######added by suetake 2022/02/17###############
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        self.mp = MovePole()


        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]

        # speed [m/s]
        self.speed = 0.07

        self.is_near_wall = False
        
        # lidar scan
        self.scan = []

        #敵の位置 大久保追加
        self.enemy_pos = []
        self.is_escaping = False


        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = -self.speed; self.near_enemy_twist.linear.y = 0.; self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0.; self.near_enemy_twist.angular.y = 0.; self.near_enemy_twist.angular.z = 0

        self.is_initialized_pose = False
        self.enemy_detector = EnemyDetector()
        #################################################

        # ---
        self.odom = 0
        self.next_way_point = 0
        self.ctrl_state = "rotate_run"
        self.ctrl_state_list = ["enemy_detected", "near_pole", "rotate_run"]
        self.state_priority = {"enemy_detected": 1,
                               "near_pole": 2,
                               "rotate_run": 3}
        self.self_pos = [-5, -5]

        #######added by suetake 2022/02/17###############
    def poseCallback(self, data):
        #自己位置
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.self_pos = [self.pose_x, self.pose_y]

        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

        self.th = rpy[2] # 自分の向き
        th_xy = self.calcTargetTheta(self.pose_x,self.pose_y)

        self.updatePoseTwist(self.th, th_xy)
        self.is_initialized_pose = True


    def updatePoseTwist(self, th, th_xy):
        # update pose twist
        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        
        delta_th = self.calcDeltaTheta(th_diff)
        new_twist_ang_z = max(-0.3, min((th_diff + delta_th) * self.k , 0.3))
        
        self.pose_twist.angular.z = new_twist_ang_z
        self.pose_twist.linear.x = self.speed
        #print("th: {}, th_xy: {}, delta_th: {}, new_twist_ang_z: {}".format(th, th_xy, delta_th, new_twist_ang_z))


    def calcTargetTheta(self, pose_x, pose_y):
        x = self.poseToindex(pose_x)
        y = self.poseToindex(pose_y)
        th = TARGET_TH[x][y]
        #print("POSE pose_x: {}, pose_y: {}. INDEX x:{}, y:{}".format(pose_x, pose_y, x, y))
        return th


    def calcDeltaTheta(self, th_diff):
        if not self.scan:
            return 0.
        R0_idx = self.radToidx(th_diff - PI/8)
        R1_idx = self.radToidx(th_diff - PI/4)
        L0_idx = self.radToidx(th_diff + PI/8)
        L1_idx = self.radToidx(th_diff + PI/4)
        R0_range = 99. if self.scan[R0_idx] < 0.1 else self.scan[R0_idx]
        R1_range = 99. if self.scan[R1_idx] < 0.1 else self.scan[R1_idx]
        L0_range = 99. if self.scan[L0_idx] < 0.1 else self.scan[L0_idx]
        L1_range = 99. if self.scan[L1_idx] < 0.1 else self.scan[L1_idx]

        #print("Ranges R0: {}, R1: {}, L0: {}, L1: {}".format(R0_range, R1_range, L0_range, L1_range))
        if R0_range < 0.3 and L0_range > 0.3:
            return PI/4
        elif R0_range > 0.3 and L0_range < 0.3:
            return -PI/4
        elif R1_range < 0.2 and L1_range > 0.2:
            return PI/8
        elif R1_range > 0.2 and L1_range < 0.2:
            return -PI/8
        else:
            return 0.
    

    def radToidx(self, rad):
        deg = int(rad / (2*PI) * 360)
        while not 360 > deg >= 0:
            if deg > 0:
                deg -= 360
            elif deg < 0:
                deg += 360
        return deg

    def poseToindex(self, pose):
        i = 7 - int((pose + WIDTH) / (2 * WIDTH) * 8)
        i = max(0, min(7, i))
        return i


    #大久保　追加
    def change_state(self, next_state):
        """
        状態変化する際に呼び出す
        キューに溜まったナビ情報も削除
        Parameters
        -----------
        next_state: str
            変更する状態
        """
        # キューに溜まったナビ情報も削除
        self.client.cancel_all_goals()
        #状態更新
        self.ctrl_state = next_state


    def lidarCallback(self, data):
        '''
        lidar scan use for bumper , and find enemy
        controll speed.x
        '''
        scan = data.ranges
        self.scan = scan
        self.is_near_wall = self.isNearWall(scan)

        #柱に近づいたときの処理
        near_pole_idx = self.mp.check_dist_with_self_pole(self.self_pos)
        if near_pole_idx >= 0:
            self.front_marker, self.back_marker = self.mp.calc_goal_pos(self.self_pos, near_pole_idx)
            if self.ctrl_state != "enemy_detected":
                self.change_state("near_pole")


        # enemy detection
        if self.is_initialized_pose:
            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan, self.pose_x, self.pose_y, self.th)
        
        if self.is_near_enemy and self.enemy_dist < 0.35:
            #敵の位置
            enemy_x = self.pose_x + self.enemy_dist * math.cos(self.th + self.enemy_direction)
            enemy_y = self.pose_y + self.enemy_dist * math.sin(self.th + self.enemy_direction)
            self.enemy_pos = [enemy_x, enemy_y]
            print(self.enemy_pos)
            if self.is_escaping == False:
                self.is_escaping = True
                self.change_state("enemy_detected")
        
        print("!!!!!!!!!!state!!!!!!!!!!!!!:", self.ctrl_state)


    def updateNearEnemyTwist(self):
        # update pose twist
        th_diff = self.enemy_direction
        # th_diff: 敵との相対角度
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        new_twist_ang_z = max(-0.3, min((th_diff) * self.k , 0.3))

        if self.enemy_dist > 0.36:
            speed = self.speed
        else:
            speed = -self.speed
        #print("enemy_dist {}".format(self.enemy_dist))
        
        self.near_enemy_twist.angular.z = new_twist_ang_z
        self.near_enemy_twist.linear.x = speed
        #print("Update near Enemy Twist")

    def isNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False
    ########################################################################


    # def odomCallback(self,data):
        # self.odom = data

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    # hPPÞ
    # def rotateRun(self):

    #     way_point = WAY_POINTS[self.next_way_point]
    #     result = self.setGoal(way_point["x"],way_point["y"],way_point["yaw"])
    #     # print("result-----------------",result)

    #     if(self.next_way_point >= len(WAY_POINTS)-1):
    #         self.next_way_point = 0
    #     else:
    #         self.next_way_point+=1


    def rondomRun(self):
        next_idx = random.randint(0, 3)
        way_point = WAY_POINTS[next_idx]
        result = self.setGoal(way_point["x"],way_point["y"],way_point["yaw"])
        # print("result-----------------",result)

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        #フロー
        #1. ぐるぐる回る状態
        #2. 自己位置と各柱との距離を各フレームごとに算出
        #3. 距離がしきい値以内のときに、マーカ読み取り状態に遷移
        #4. 目的地に到達したら読み込んだことにする
        #5. ctrl_stateを"rotate_run"にする←2/24完了
        #6. 末武さんの部分マージ
        #   マージ内容：
        #状態の優先度（値が小さいほど大きい）
        #1 enemy_detected
        #2 near_pole
        #3 rotate_run

        #打ち合わせ内容
        # グルグルまわなくても、ランダムにwaypoint設定でも良さそう
        while not rospy.is_shutdown():
        #     このループは、setGoalで目的地が設定されていると、移動制御が優先される
        #     そのため、このループは、移動制御が終わったあとにループに入る
        #     優先度はかなり低いため、短周期で行いたい処理は、センサーデータ取得のコールバックに記述
        #     （高優先度処理の例：敵検知、柱との距離算出）
            if(self.ctrl_state == "rotate_run"):
                # self.rotateRun()
                self.rondomRun()

            elif self.ctrl_state == "near_pole":
                self.setGoal(self.front_marker[0], self.front_marker[1], self.front_marker[2])
                self.setGoal(self.back_marker[0], self.back_marker[1], self.back_marker[2])
                self.change_state("rotate_run")
            
            elif self.ctrl_state == "enemy_detected":
                #segGoalで逃げる場所を決める
                seleceted_way_p = select_escape_point(self.enemy_pos)
                self.setGoal(seleceted_way_p["x"], seleceted_way_p["y"], seleceted_way_p["yaw"])
                self.setGoal(seleceted_way_p["x"], seleceted_way_p["y"], seleceted_way_p["yaw"])
                self.is_escaping = False
                self.change_state("rotate_run")

            r.sleep()



if __name__ == '__main__':
    rospy.init_node('bigkatsu')
    bot = NaviBot()
    bot.strategy()