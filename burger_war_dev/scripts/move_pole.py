# coding: UTF-8

from turtle import back
from params import *

#pole_points_listは、柱の中心座標のため、goalに設定すると目的地までの経路が算出できない
#経路算出可能な地点にするためのバイアス
#中心の柱は読みに行かないため、x方向にのみ適用
MARKER_BIAS = 0.4
#柱との距離のしきい値 
SELF_POLE_DIST_THRES = 0.4
approach_dir= {"front":PI,
                "back": 0.0}

class MovePole():

    def __init__(self):
        self.old_pole_idx = -1

    def calc_dist(self, self_pos, pole_pos):
        """
        自己位置と柱の距離を算出

        Parameters
        ----------
        self_pos: list or tupple
            自己位置
        pose_pos: list or tupple
            柱の位置

        Returns
        ---------
        dist: float
            距離
        """
        x1, y1 = self_pos
        x2, y2 = pole_pos
        temp_x = (x2- x1) ** 2
        temp_y = (y2- y1) ** 2
        dist = (temp_x + temp_y) ** 0.5
        return dist

    def check_dist_with_self_pole(self, self_pos):
        """
        自己位置と柱の距離を算出し、しきい値以内の柱のIDを返す

        Parameters
        ----------
        self_pos: list or tupple
            自己位置

        Returns
        ----------
        near_pole_idx: int
            しきい値以内に最初に入った柱のidx
            複数しきい値に入ったら最も近い柱のidxにする
            しきい値以内に入ってない場合は、-1を返す。
        """
        #各柱までの距離をリストに格納
        dist_list = [self.calc_dist(self_pos, pole_points_list[i]) for i in range(len(pole_points_list))]
        min_dist = min(dist_list)
        min_idx = dist_list.index(min_dist)

        #最も近い柱との距離がしきい値以内のときは柱のidx, 違う場合は-1
        near_pole_idx = min_idx if min_dist < SELF_POLE_DIST_THRES else -1
        # print("dist with self position and closest pole:", min_dist)
        # print("closest pole idx:", near_pole_idx)

        #連続していないかも判定
        if self.old_pole_idx == near_pole_idx:
            near_pole_idx = -1
        return near_pole_idx

    def judge_which_marker(self, self_pos, near_pole_pos):
        """
        柱のfront or back どちらのマーカを読みに行くか決定
        
        Parameters
        -----------
        self_pos: list or tupple
            自己位置
        near_pole_pos: list or tupple
            複数しきい値に入ったら最も近い柱のidxにする
            しきい値以内に入ってない場合は、-1を返す。

        Return
        ---------
        judge_result: str
            "front" or "back"
        """
        self_x = self_pos[0]
        pole_x = near_pole_pos[0]
        which_marker = ""
        if self_x > pole_x:
            which_marker = "front"
        else:
            which_marker = "back"
        
        return which_marker

        


    def calc_goal_pos(self, self_pos, near_pole_idx):
        """
        誘導する位置を算出
        
        Parameters
        -----------
        near_pole_idx: int
            複数しきい値に入ったら最も近い柱のidx
        which_marker: str
            "front" or "back"

        Return
        ---------
        goal_pos: list 
            [x, y, yaw]
        """
        near_pole_pos = pole_points_list[near_pole_idx]
        x, y = near_pole_pos
        dir = 0
        which_marker = self.judge_which_marker(self_pos, near_pole_pos)

        # if which_marker == "front":
        #     x += MARKER_BIAS
        #     dir = approach_dir[which_marker]
        # elif which_marker == "back":
        #     x -= MARKER_BIAS
        #     dir = approach_dir[which_marker]
        
        front_marker = [x+MARKER_BIAS, y, approach_dir["front"]]
        back_marker = [x-MARKER_BIAS, y, approach_dir["back"]]
        self.old_pole_idx = near_pole_idx
        return front_marker, back_marker
        # return [x, y, dir]
