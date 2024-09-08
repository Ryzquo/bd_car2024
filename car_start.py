#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    my_car.STOP_PARAM = False

    # my_car.task.reset()
    
    # 全局巡航速度
    cruise_speed = 0.6
    # 左右传感器巡航速度
    sensor_speed = 0.6
    # 目标检测巡航速度
    det_speed = 0.2
    
    block_pt_tar = [
        0, 0, "block", 0, 
        -0.12259615384615384, 0.3605769230769231, 0.8894230769230769, 1.2692307692307692
    ]
    ball_pt_tar = [
        4, 0, 'blue_ball', 0.6, 
        -0.1466346153846154, 0.07932692307692307, 0.8125, 1.0721153846153846
    ]
    cylinder_tar_list = [
        [
            1, 0, "cylinder1", 0, 
            -0.13701923076923078, 0.2956730769230769, 1.6009615384615385, 1.3990384615384615
        ],
        [
            2, 0, "cylinder2", 0, 
            -0.10576923076923077, 0.31009615384615385, 1.2115384615384615, 1.3605769230769231
        ],
        [
            3, 0, "cylinder3", 0, 
            -0.07692307692307693, 0.35096153846153844, 0.8653846153846154, 1.2884615384615385
        ]
    ]
    criminal_pt_tar = [
        0, 1, 'pedestrian', 0, 
        0.021875, 0.06458333333333334, 0.45, 1.6125
    ]

    def grap_block_func():
        """
        抓取方块
        """
        # 初始化
        my_car.task.block_arm_reset()
        
        def grab_first_block():
            """
            抓取第一个方块
            """
            logger.info(f"开始抓取第一个方块...")
            # 巡航一定距离到方块台前
            my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.12)
            # 巡航到右侧0.1m有障碍
            my_car.lane_sensor(speed=sensor_speed, value_h=0.15, sides=-1, stop=False)
            
            # 定位目标
            res = my_car.lane_det_location(speed=det_speed, pt_tar=block_pt_tar, side=-1)
            if not res:
                return
            
            # 抓取第一个方块
            my_car.task.block_grab_first()
            
            logger.info(f"抓取第一个方块完成...")
        
        def grab_second_block():
            """
            抓取第二个方块
            """
            logger.info(f"开始抓取第二个方块...")
            # 巡航一段距离, 走这一段是为了第二次抓取时不重复判断第一个物料台
            my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.08, stop=False)
            # 巡航到右侧0.1m有障碍
            my_car.lane_sensor(speed=sensor_speed, value_h=0.15, sides=-1, stop=False)
            
            # 根据抓取物体的位置定位目标
            res = my_car.lane_det_location(speed=det_speed, pt_tar=block_pt_tar, side=-1)
            if not res:
                return

            # 抓取方块
            my_car.task.block_grab_second()
            
            # 脱离黄线
            my_car.set_pos_offset(pos=[0, 0.03, 0])
            
            logger.info(f"抓取第二个方块完成...")
            
        grap_block_task_list = [
            grab_first_block, 
            grab_second_block
        ]
        
        for task in grap_block_task_list:
            task()

    def release_block_func():
        """
        放下方块
        """
        # 巡航一定距离走出方块台
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=2.2, stop=False)
        
        # 巡航右侧感应器感应到0.4m有障碍
        my_car.lane_sensor(speed=sensor_speed, value_h=0.4, sides=-1, stop=False)
        
        # === 放下第一个方块
        # 巡航一定距离
        # my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.1, stop=True)
        # 调整位置,这个根据巡航效果调整
        # my_car.set_pos_offset(pos=[0.12, -0.008, -12.5])
        # my_car.set_pos_offset(pos=[0.12, -0.008, -11])
        # my_car.set_pos_offset(pos=[0.075, 0.002, -7])
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.17, stop=True)
        my_car.set_pos_offset(pos=[0, 0.005, 0])
        # 放下方块x
        my_car.task.block_release_first()
        
        # 放下第二个方块
        # 前移
        my_car.set_pos_offset(pos=[0.09, 0.003, 0])
        # 放下方块
        my_car.task.block_release_second()
        
        # # 向右偏移一定距离
        # my_car.set_pos_offset(pos=[0, -0.02, 0])

    def get_ball_func():
        """
        抓取小球
        """
        # 初始化抓球arm位置
        my_car.task.ball_arm_reset()
        
        # 巡航
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.7)
        my_car.lane_sensor(speed=sensor_speed, value_h=0.4, sides=1, stop=True)
        
        # 获取开始时的已巡航距离
        start_dis = my_car.get_dis_traveled()
        for i in range(3):
            # 初始化抓球arm位置
            my_car.task.ball_arm_reset()
            
            # 根据给定目标和位置、方向定位调整车子的位置
            res = my_car.lane_det_location(
                speed=det_speed, pt_tar=ball_pt_tar, side=1
            )
            
            # 没有检测到目标
            if not res:
                # 如果已巡航距离超过0.4m就跳出
                if my_car.get_dis_traveled() - start_dis < 0.40:
                    logger.info("dis out {}".format(i))
                else:
                    logger.info("can not find ball")
                    break
                continue
            
            # 抓球
            my_car.task.ball_pick_up()
            # 放球
            my_car.task.ball_put_down_to_self()
        
        # 向右一定距离, 脱离黄线
        my_car.set_pos_offset(pos=[0.0, -0.03, 0])

    def elevation_pole_func():
        """
        升起信号塔
        """
        # 复位
        my_car.task.rotate_servo_reset()
        
        # 巡航一定距离
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.5)
        
        # 巡航到左侧感应器感应到障碍
        my_car.lane_sensor(speed=sensor_speed-0.2, value_h=0.2, sides=1)
        
        # 调整位置对准旋转杆
        my_car.set_pos_offset(pos=[0.1115, 0.02, 0])
        
        # 旋转
        my_car.task.elevation_pole()
        
        # 脱离
        my_car.set_pos_offset(pos=[0.0, -0.03, 0])
        
        # 复位
        my_car.task.rotate_servo_reset()

    def get_high_ball_func():
        """
        取高球
        """
        # 初始化
        my_car.task.high_ball_reset()
        
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=1.2)
        
        my_car.lane_sensor(speed=sensor_speed-0.2, value_h=0.4, sides=1)
        
        # 调整位置
        my_car.set_pos_offset(pos=[-0.08, 0.02, 0])
        
        # # 取球
        my_car.task.high_ball_pick()
        
        my_car.set_pos_offset(pos=[0, -0.03, 0])


    def pull_ball_func():
        """
        放下小球
        """
        # 巡航一定距离
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=1.7)
        
        # 巡航到右侧有障碍
        my_car.lane_sensor(speed=sensor_speed-0.2, value_h=0.30, sides=1)
        
        # 调整位置
        my_car.lane_dis_offset(speed=cruise_speed-0.1, dis_hold=0.184)
        my_car.set_pos_offset(pos=[0, 0.07, 0])
        
        # 放下小球
        my_car.task.ball_put_down()


    def hanoi_tower_func():
        """
        叠汉诺塔
        """
        # 前进到拐角
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.45) 
        # 过拐角
        my_car.lane_dis_offset(speed=0.4, dis_hold=0.4) 
        # 直到卡片前
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.15) 
        # 定位到转向卡片前
        my_car.lane_det_dis2pt(speed=0.25, dis_end=1.55)
        
        # 检测卡片方向
        side = my_car.get_card_side()
        # 机械手方向相反
        arm_side = side * -1
        # 调机械手方向
        my_car.task.cylinder_arm_reset(arm_side=arm_side)
        # 调整车子朝向
        # my_car.set_pos_offset([0, 0, 67.5 * side], 1)
        my_car.set_pos_offset(pos=[0, 0, 60*side])
                
        # 走一段距离
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.6)
        my_car.set_pos_offset(pos=[0, 0.00*side, 5.5*arm_side])
        # 两次感应到侧面位置
        my_car.lane_sensor(speed=0.3, value_h=0.3, sides=arm_side)
        logger.info(f"感应到平台起始柱: {np.array(my_car.get_odom())}")
        my_car.lane_sensor(speed=0.2, value_l=0.3, sides=arm_side, stop=True)
        logger.info(f"感应到平台结束柱: {np.array(my_car.get_odom())}")
        # 记录此时的位置
        pos_start = np.array(my_car.get_odom())
        logger.info(f"start pos:{pos_start}")
        
        # 开始抓取每个圆柱并叠加
        for pt_tar in cylinder_tar_list:
            # 获取当前抓取的圆柱id
            cylinder_id = pt_tar[0]
            
            # 初始化arm
            my_car.task.cylinder_arm_reset(cylinder_id=cylinder_id)
            
            if cylinder_id == 3:
                # 往前走一段, 防止重复判断1和3
                my_car.lane_dis_offset(speed=0.2, dis_hold=0.1)
            
            # 根据给定信息定位目标
            my_car.lane_det_location(
                speed=det_speed, pt_tar=pt_tar, side=arm_side, 
                dis_out=0.4)
            
            # 抓取圆柱
            my_car.task.cylinder_pick_up(cylinder_id=cylinder_id)
            
            if cylinder_id == 1:
                # 计算走到记录位置的距离
                run_dis = my_car.calculation_dis(
                    pos_dst=pos_start, 
                    pos_src=np.array(my_car.get_odom())
                )
                print(f"run_dis:{run_dis}")
                # 后移刚才计算的距离，稍微少走一点儿
                # my_car.set_pos_offset([0 - (run_dis + 0.065), 0, 0])
                my_car.set_pos_offset(pos=[-(run_dis - 0.060), 0, 0])
                
                # 记录位置
                tar_pos = my_car.get_odom()
                logger.info("tar_pos:{}".format(tar_pos))
            else: 
                # 回到叠加位置
                my_car.set_pos(tar_pos)
            
            # 放下圆柱
            my_car.task.cylinder_put_down(cylinder_id)
            
        my_car.set_pos_offset(pos=[0, 0.03*side, 0])

    def camp_fun():
        """
        营地任务
        """
        def thread_yiyan_get_actions(text):
            """
            获取动作
            """
            global actions_map
            actions_map = []
            # 大模型理解
            actions_map = my_car.yiyan_get_actions(text, request_timeout=20)
            # 如果一次生成失败就多次生成
            while actions_map == []:
                # text = my_car.get_ocr()
                # logger.info("text:{}".format(text))
                actions_map = my_car.yiyan_get_actions(text, request_timeout=20)
            logger.info(f"actions_map:{actions_map}")
            
                
        # 调整位置准备进行ocr识别
        my_car.task.ocr_arm_reset()
        
        # 巡航到文字
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=1.3)
        
        my_car.lane_dis_offset(speed=0.5, dis_hold=0.7)
        
        # 感应到右侧障碍距离小于0.4
        my_car.lane_sensor(
            speed=0.4, value_h=0.4, 
            dis_offset=0.02, sides=-1, 
            stop=True
        )
        
        # 识别
        text = my_car.get_ocr(is_test=False)
        text = text if text else "照亮3秒"
        logger.info(f"det_text: {text}")
        
        global actions_map
        actions_map = []
        # 开始后台文心生成进程
        threading.Thread(
            target=thread_yiyan_get_actions, 
            args=(text,), daemon=True
            ).start()
        
        # # 前移到营地左侧
        # my_car.set_pos_offset(pos=[0.48, -0.11, -34.377], time_dur=2.5)
        my_car.set_pos_offset(pos=[0.48, -0.13, -33])
        pos_start = np.array(my_car.get_odom())
        
        # 离开道路到修整营地
        # my_car.set_pos_offset(pos=[0.15, -0.4, 0], time_dur=2)
        my_car.set_pos_offset(pos=[0, -0.477, 0])
        
        # 等待大模型生成完成
        while actions_map == []:
            logger.info("文心大模型推理中...")
            time.sleep(0.1)
        
        # 做任务
        my_car.do_action_list(actions_map)
        # 回到原来位置
        my_car.set_pos(pos_start)

    def find_criminal():
        """
        找到罪犯打击罪犯
        """
        def thread_yiyan_get_actions(text):
            """
            获取动作
            """
            global criminal_attr
            criminal_attr = {}
            # 大模型理解
            criminal_attr = my_car.hum_analysis.get_res_json(text, request_timeout=20)
            logger.info(f"criminal_attr:{criminal_attr}")
            while criminal_attr == {}:
                # text = my_car.get_ocr()
                # logger.info("text:{}".format(text))
                criminal_attr = my_car.hum_analysis.get_res_json(text, request_timeout=20)
                logger.info(f"criminal_attr:{criminal_attr}")

        # 调整位置准备进行ocr识别
        # my_car.task.ocr_arm_reset()
        threading.Thread(
            target=my_car.task.ocr_arm_reset, 
            daemon=True
        ).start()
        
        # 移动一段距离
        my_car.lane_dis_offset(speed=cruise_speed, dis_hold=0.7)
        
        # 感应到右侧障碍距离小于0.4
        my_car.lane_sensor(
            speed=0.3, value_h=0.4, dis_offset=0.02, sides=-1, stop=True
        )
        
        # 文本识别
        text = my_car.get_ocr()
        text = text if text else "犯人是一个老年人"
        logger.info(f"text:{text}")
        
        global criminal_attr
        criminal_attr = {}
        
        # 开始后台文心生成进程
        threading.Thread(
            target=thread_yiyan_get_actions, 
            args=(text,), daemon=True
            ).start()
        
        # 调整机械手到识别与打击罪犯位置
        # my_car.task.criminal_arm_reset()
        threading.Thread(
            target=my_car.task.criminal_arm_reset, 
            daemon=True
        ).start()
        
        # 巡航到识别位置
        my_car.lane_sensor(
            speed=0.1, value_h=0.4, sides=-1, stop=True
        )
        
        # 等待大模型推理完成
        if criminal_attr == {}:
            print("文心大模型推理中", end="")
        while criminal_attr == {}:
            print(".", end="")
            time.sleep(0.1)
        print()
        
        max_matching_results_count = -1
        match_criminal_idx = -1
        match_criminal_pos = my_car.get_odom()
        # 遍历4个人物
        for i in range(4):
            if i == 0:
                # 人物定位
                my_car.lane_det_location_mot(
                    speed=0.1, pt_tar=criminal_pt_tar, side=-1
                )
            # 获取人物属性
            attr_hum = my_car.get_hum_attr(criminal_pt_tar)
            logger.info(f"idx: {i}\natter_hum: {attr_hum}")
            # 获取匹配的结果数量
            matching_results_count, match_keys = my_car.compare_humattr(
                criminal_attr, attr_hum
            )
            logger.info(f"count: {matching_results_count}")
            print()
            
            # 结果处理
            if matching_results_count == len(criminal_attr):
                logger.info("找到罪犯")
                my_car.task.punish_criminal()
                break
            else:
                # 记录匹配度最高的罪犯的匹配结果数和位置
                if matching_results_count > max_matching_results_count:
                    max_matching_results_count = matching_results_count
                    match_criminal_idx = i
                    match_criminal_pos = my_car.get_odom()
                # 前往下一个位置
                if i != 3:
                    my_car.set_pos_offset(pos=[0.076, 0, 0])
        else:
            logger.info("没有找到罪犯")
            # 击倒匹配度最高的
            logger.info("开始击倒匹配度最高的罪犯...")
            logger.info(
                f"count: {max_matching_results_count}/{len(criminal_attr)} - {match_keys}"
                f", idx: {match_criminal_idx}"
                f", pos: {match_criminal_pos}"
            )
            my_car.set_pos(match_criminal_pos)
            # 击倒
            my_car.task.punish_criminal()

    def go_start():
        """
        回到起始点
        Returns:

        """
        my_car.lane_sensor(speed=cruise_speed, value_l=0.4, sides=-1)
        my_car.set_pos_offset(pos=[0.72, 0, 0], sp=[0.6, 0.2, 45])


    my_car.beep()
    time.sleep(0.2)
    functions = [
        # 拿方块
        grap_block_func,
        # 放方块
        release_block_func,
        # 拿球
        get_ball_func,
        # 升起信号塔
        elevation_pole_func,
        # 拿高处的球
        get_high_ball_func,
        # 放球
        pull_ball_func,
        # 汉诺塔
        hanoi_tower_func,
        # 做ocr任务
        camp_fun, 
        # 寻找并击倒罪犯    
        find_criminal,
        # 回到起始点
        go_start
    ]
    my_car.manage(functions, 7)
