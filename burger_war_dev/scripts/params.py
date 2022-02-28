# coding: UTF-8
import math

PI = math.pi

# 8x8  [rad]
TARGET_TH = (
    (-PI/4, -PI/4, -PI/2, -PI/2, -PI*3/4, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/3, -PI/2, -PI*3/5, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/6,     0,   -PI/2, -PI*3/4,     -PI,  PI*3/4),
    (-PI/4, -PI/5,     0,     0,      PI,  PI*6/10,  PI*3/4,    PI/2),
    (    0,     0,  PI/2,  PI/2,      PI,  PI*3/4,  PI*3/4,    PI/2),
    (    0,  PI/4,  PI/3,  PI/2,  PI*5/6,  PI*3/4,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/3,  PI*5/6,    PI/2,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/4,    PI/3,    PI/2,  PI*3/4,  PI*3/4),
)


WAY_POINTS = [
    {"x":0.0,"y":1.0,"yaw":-PI/2},
    {"x":1.0,"y":0.0,"yaw":-PI},
    {"x":0.0,"y":-1.0,"yaw":PI/2},
    {"x":-1.0,"y":0.0,"yaw":0}
]


#WIDTH = 1.2 * (2 **0.5) # [m]
WIDTH = 1.2 / (2 **0.5) # [m]

#柱間の距離
POLE_DIST = 0.53

#各柱の座標
#格納番地とField.jpgに記述したIDが対応関係
pole_points_list = [
    (-POLE_DIST, POLE_DIST), (POLE_DIST, POLE_DIST),
    (POLE_DIST, -POLE_DIST), (-POLE_DIST, -POLE_DIST)
]

