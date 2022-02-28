# coding: UTF-8
import math
from params import *

def calc_dist(p1, p2):
    """
    p1, p2間の距離を算出
    
    Parameters
    ----------
    p1, p2: list
        [x, y]
    
    Returns
    ----------
    dist: float
        p1, p2間の距離
    """
    x1, y1 = p1
    x2, y2 = p2
    temp_x = (x2- x1) ** 2
    temp_y = (y2- y1) ** 2
    dist = (temp_x + temp_y) ** 0.5
    return dist
    

def select_escape_point(enemy_pos):
    """
    self.enemy_posから最も遠いWAY_POINTを選択する
    Returns
    ---------
    最も遠いWAY_POINTの要素（辞書型）
    """
    selected_w_p = None
    max_dist = -1

    for w_p in WAY_POINTS:
        w_pos = [w_p['x'], w_p['y']]
        dist = calc_dist(enemy_pos, w_pos)
        if (dist > max_dist):
            max_dist = dist
            selected_w_p = w_p
            print(selected_w_p)

    return selected_w_p
    