#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import threading

import numpy as np

# 包含根目录
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from ryzquo_tools.log_tools import logger

from tools import ThreadAction
from car_wrap import MyCar


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False
    
    # 全局巡航速度
    cruise_speed = 0.5
    # 左右传感器巡航速度
    sensor_speed = 0.5
    # 目标检测巡航速度
    det_speed = 0.1

    def block_positioning():
        """
        方块定位
        Returns:

        """
        # 初始化
        my_car.task.block_arm_reset(reset=True)
        # pt_tar = [0, 0, "block", 0, -0.03, -0.28, 0.92, 0.80]
        # pt_tar = [0, 0, "block", 0, -0.1033, 0.4759, 0.8894, 1.0384]
        pt_tar = [0, 0, "block", 0, 0.0168, 0.375, 0.9086, 1.25]
        
        res = my_car.lane_det_location_block(
            speed=det_speed, pt_tar=pt_tar, 
            is_location=True, show=True
        )

    def ball_positioning():
        """
        小球定位
        """
        # 初始化抓球arm位置
        my_car.task.ball_arm_reset(reset=True)
        
        # pt_tar = [4, 0, 'blue_ball', 0.6, -0.05, 0.15, 0.47, 0.62]
        # pt_tar = [4, 0, 'blue_ball', 0.6, -0.0120, 0.4927, 0.7932, 1.0048]
        # pt_tar = [4, 0, 'blue_ball', 0.6, -0.1538, 0.3197, 0.7980, 1.0528]
        pt_tar = [4, 0, 'blue_ball', 0.6, -0.0240, -0.0024, 0.8076, 1.0721]
        
        res = my_car.lane_det_location_ball(
            speed=det_speed, pt_tar=pt_tar, side=1, 
            is_location=True, show=True
        )

    def cyclinder_positioning():
        """
        圆柱定位
        """
        arm_side = 1
        # 初始化arm
        my_car.task.cylinder_arm_reset(arm_side=arm_side, reset=True)
        
        # 目标列表
        cylinder_tar_list = [
            [
                1, 0, "cylinder1", 0, 
                -0.038461538461538464, 0.4230769230769231, 1.3557692307692308, 1.1538461538461537
            ],
            [
                2, 0, "cylinder2", 0, 
                0.0, 0.43990384615384615, 1.0961538461538463, 1.1201923076923077
            ],
            [
                3, 0, "cylinder3", 0, 
                -0.002403846153846154, 0.4831730769230769, 0.8028846153846154, 1.0336538461538463
            ]
        ]
        cylinder_id = 3
        tar = cylinder_tar_list[cylinder_id-1]
        my_car.task.cylinder_arm_reset(arm_side=arm_side, cylinder_id=tar[0])
        # 根据给定信息定位目标
        my_car.lane_det_location_cylinder(
            speed=det_speed, pt_tar=tar, side=arm_side, 
            dis_out=0.4, 
            is_location=True, show=True
        )
        
    def criminal_positioning():
        """
        罪犯定位
        """
        criminal_pt_tar = [
            0, 1, 'pedestrian', 0, 
            -0.034375, 0.03958333333333333, 0.525, 1.5958333333333334
        ]
        
        # 调整机械手到识别与打击罪犯位置
        my_car.task.criminal_arm_reset(reset=True)
        
        my_car.lane_det_location_mot(
            speed=0.1, pt_tar=criminal_pt_tar, side=-1, 
            is_location=True, show=True
        )
        
    def camp_positioning():
        """
        营地定位
        """
        camp_pt_tar = [
            0, 1, 'camp', 0, 
            -0.09615384615384616, 0.10336538461538461, 1.1153846153846154, 0.6105769230769231
        ]
        
        # 调整arm位置
        my_car.task.camp_arm_reset(reset=True)
        
        my_car.lane_det_location_camp(
            speed=0.1, pt_tar=camp_pt_tar, side=-1
            , is_location=True, show=True
        )
        

    my_car.beep()
    time.sleep(0.2)
    functions = [
        block_positioning,
        ball_positioning,
        cyclinder_positioning, 
        camp_positioning, 
        criminal_positioning, 
    ]
    block_positioning()
    # ball_positioning()
    # my_car.manage(functions, 0)
