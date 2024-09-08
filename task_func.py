#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from vehicle import ArmBase, ScreenShow, Key4Btn, ServoBus, ServoPwm
import cv2
import time
import numpy as np
import yaml, os


class MyTask:
    """
    使用机器臂完成任务
    """
    def __init__(self):
        # 获取当前文件所在目录的绝对路径
        self.path_dir = os.path.dirname(os.path.realpath(__file__))
        # 拼接配置文件路径
        self.yaml_path = os.path.join(self.path_dir, "config_task.yml")
        
        self.config = None
        try:
            # 打开并读取配置文件
            with open(self.yaml_path, 'r') as stream:
                self.config = yaml.load(stream, Loader=yaml.FullLoader)
        except:
            print("配置文件打开失败")
            self.config = {
                "reset": {
                    
                }, 
                "block": {
                    
                }, 
                "ball": {
                    
                }, 
                "cylinder": {
                    
                }, 
                "high_ball": {
                    
                }, 
            }
        
        self.reset_cfg = self.config["reset"]
        self.block_cfg = self.config["block"]
        self.ball_cfg = self.config["ball"]
        self.cylinder_cfg = self.config["cylinder"]
        self.high_ball_cfg = self.config["high_ball"]
        self.ocr_cfg = self.config["ocr"]
        self.camp_cfg = self.config["camp"]
        self.criminal_cfg = self.config["criminal"]
        
        # 初始化机械臂基座
        self.arm = ArmBase()
        self.arm.reset()
        
        # 控制旋转信号塔舵机(id-2)
        self.servo_rotate = ServoBus(2)
        self.servo_rotate.set_angle(speed=90, angle=0)

        # 控制放球平台舵机(id-3)
        self.servo_ball = ServoBus(3)
        self.servo_ball.set_angle(
            speed=80, angle=self.reset_cfg["servo_ball_init_angle"]
        )

        # 等待舵机动作完成
        time.sleep(0.3)
        # 设置旋转舵机和夹手舵机旋转状态
        self.servo_ball.set_rotate(speed=0)
        self.servo_rotate.set_rotate(speed=0)

        # 控制抓取高塔小球舵机
        self.servo_high = ServoPwm(2)

    def save_yaml(self):
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream)
            
    def servo_ball_reset(self):
        """
        放球平台复位
        """
        self.servo_ball.set_angle(
            speed=80, angle=self.reset_cfg["servo_ball_init_angle"]
        )
        # time.sleep(0.3)
        self.servo_ball.set_rotate(speed=0)
               
    # ============== 方块部分 start ==============
    def block_arm_reset(self, reset=False):
        """
        重置机器臂到方块任务初始位置
        """
        if reset:
            # 抓夹
            self.arm.set_grap_angle(angle=self.block_cfg["grab_close"])
            self.arm.set(
                horiz_pos=self.reset_cfg["arm_init_pos"][0], 
                vert_pos=self.reset_cfg["arm_init_pos"][1]
            )
        
        # 夹手转换到右侧
        self.arm.switch_side(side=self.block_cfg["arm_side"])
        
        # 设置机械臂初始位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_init_pos"][0], 
            vert_pos=self.block_cfg["arm_init_pos"][1]
        )
        
        # 打开抓夹
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
    
    # -------------- 夹取方块部分 start --------------
    def block_grab_first(self):
        """
        抓取第一个球
        """
        # 打开抓夹
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
        
        # 设置机器臂到抓取位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_grab_pos"][0], 
            vert_pos=self.block_cfg["arm_grab_pos"][1]
        )
        # 抓取方块
        self.arm.set_grap_angle(angle=self.block_cfg["grab_close"])
        time.sleep(0.5)
        
        # 放平放球平台
        self.servo_ball.set_angle(
            speed=80, angle=self.block_cfg["servo_ball_flattening_angle"]
        )
        # 机械臂到达指定位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_drop_first_pos"][0], 
            vert_pos=self.block_cfg["arm_drop_first_pos"][1]
            )
        # 抓夹面向指定角度
        self.arm.set_arm_angle(angle=self.block_cfg["arm_grab_angle"])
        time.sleep(0.5)
        
        # 下移
        vert_offset = 0.055
        self.arm.set_offset(horiz_offset=0, vert_offset=-vert_offset, time_run=0.8)
        # 抓夹张开, 放下方块
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
        time.sleep(0.4)
        
        # 上移
        self.arm.set_offset(horiz_offset=0, vert_offset=vert_offset, time_run=0.8)
        # 返回初始角度
        self.arm.set_arm_dir(dir=self.block_cfg["arm_side"])
        time.sleep(0.5)
        
        # 初始化机械臂位置
        self.block_arm_reset()
    
    def block_grab_second(self):
        """
        夹取第二个方块
        """
        # 打开抓夹
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
        
        # 设置机器臂到抓取位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_grab_pos"][0], 
            vert_pos=self.block_cfg["arm_grab_pos"][1]
        )
        # 抓取方块
        self.arm.set_grap_angle(angle=self.block_cfg["grab_close"])
        time.sleep(0.5)
        
        # 机械臂到达指定位置
        self.arm.set_offset(horiz_offset=0.06, vert_offset=0.02)
    # -------------- 夹取方块部分 end --------------

    # -------------- 放下方块部分 start --------------
    def block_release_first(self):
        """
        放下第一个方块
        """
        # 设置位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_drop_pos"][0], 
            vert_pos=self.block_cfg["arm_drop_pos"][1]
        )
        # 下降
        # self.arm.set_offset(horiz_offset=-0.13, vert_offset=-0.06, time_run=0.8)
        # self.arm.set_offset(horiz_offset=-0.125, vert_offset=-0.06, time_run=0.8)
        # 松开抓夹
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
        time.sleep(0.5)
        
        # 到达指定位置上方
        self.arm.set(horiz_pos=-0.11, vert_pos=0.10)
    
    def block_release_second(self):
        """
        放下第二个方块
        """
        # 设置机械手角度
        self.arm.set_arm_angle(angle=self.block_cfg["arm_drop_angle"])
        time.sleep(0.5)
        
        # === 取出方块
        vert_offset = 0.056
        # 下移 vert_offset
        self.arm.set_offset(horiz_offset=0, vert_offset=-vert_offset, time_run=0.8)
        # 抓取物品
        self.arm.set_grap_angle(angle=self.block_cfg["grab_close"])
        time.sleep(0.4)
        
        # 上移 vert_offset
        self.arm.set_offset(horiz_offset=0, vert_offset=vert_offset, time_run=0.8)
        # 抓手回到初始角度
        self.arm.set_arm_dir(dir=-1)
        time.sleep(0.5)
        
        # 设置位置
        self.arm.set(
            horiz_pos=self.block_cfg["arm_drop_pos"][0], 
            vert_pos=self.block_cfg["arm_drop_pos"][1]
        )
        # 下降
        # self.arm.set_offset(horiz_offset=-0.125, vert_offset=-0.06, time_run=0.8)
        # 松开
        self.arm.set_grap_angle(angle=self.block_cfg["grab_open"])
        time.sleep(0.5)
        
        # 复位
        self.arm.set(horiz_pos=-0.12, vert_pos=0.05)
        
        # 放球平台复位
        self.servo_ball_reset()
    # -------------- 放下方块部分 end --------------
    # ============== 方块部分 end ==============
    
    
    # ============== 小球部分 start ==============
    def ball_arm_reset(self, reset=False):
        """
        重置机器臂到小球任务初始位置
        """
        if reset:
            # 抓夹
            self.arm.set_grap_angle(angle=self.ball_cfg["grab_close"])
            self.arm.set(
                horiz_pos=self.reset_cfg["arm_init_pos"][0], 
                vert_pos=self.reset_cfg["arm_init_pos"][1]
            )
            
        # 设置arm位置
        self.arm.set(
            horiz_pos=self.ball_cfg["arm_init_pos"][0], 
            vert_pos=self.ball_cfg["arm_init_pos"][1]
        )
        
        # 机械手面向左侧
        self.arm.switch_side(side=self.ball_cfg["arm_side"])
        
        # 张开抓夹
        self.arm.set_grap_angle(angle=self.ball_cfg["grab_open"])
        
        # # 放球平台复位
        # self.servo_ball_reset()
        
    # -------------- 夹取小球部分 start --------------
    def ball_pick_up(self):
        """
        夹取小球
        """
        # 设置arm位置
        self.arm.set(
            horiz_pos=self.ball_cfg["arm_init_pos"][0], 
            vert_pos=self.ball_cfg["arm_init_pos"][1]
        )
        # 张开抓夹
        self.arm.set_grap_angle(angle=self.ball_cfg["grab_open"])
        
        # 抓取位移
        horiz_offset = 0.11
        # 设置抓取位置
        self.arm.set_offset(horiz_offset=horiz_offset, vert_offset=0, time_run=0.6)
        # 抓取
        self.arm.set_grap_angle(angle=self.ball_cfg["grab_close"])
        time.sleep(0.5)
        
        # 上升后退
        self.arm.set_offset(horiz_offset=0, vert_offset=0.04, time_run=0.1)
        self.arm.set_offset(horiz_offset=-horiz_offset, vert_offset=0, time_run=0.1)

    def ball_put_down_to_self(self):
        """
        把小球放到自身平台
        """
        # # 设置机械臂位置
        # self.arm.set(horiz_pos=horiz_pos, vert_pos=vert_pos)
        # 设置机械手角度, 旋转到平台上方
        self.arm.set_arm_angle(angle=self.ball_cfg["arm_drop_angle"])
        time.sleep(0.4)
        
        # 放下小球
        # self.arm.set(-0.1, 0.105, 0.3)
        self.arm.set_grap_angle(angle=self.ball_cfg["grab_open"])
        time.sleep(0.4)
        
        # 机械手复位
        # self.arm.set(-0.1, 0.105, 0.3)
        self.arm.set_arm_dir(dir=self.ball_cfg["arm_side"])
        time.sleep(0.3)
    # -------------- 夹取小球部分 end --------------
    
    # -------------- 放下小球部分 start --------------
    def ball_put_down(self):
        """
        放下小球
        """
        # 从-110度平滑过渡到-25度
        self.servo_ball.smooth_transition(
            start_angle=-110, end_angle=-25, step=6, 
            speed=80
        )
        time.sleep(1.2)
        
        # 复位
        self.servo_ball_reset()
    # -------------- 放下小球部分 end --------------
    # ============== 小球部分 end ==============
    
    
    # ============== 信号塔部分 start ==============
    def rotate_servo_reset(self):
        """
        旋转舵机
        """
        # 设置电机以90的速度旋转, 复位到0
        self.servo_rotate.set_angle(speed=90, angle=0)
        time.sleep(0.3)
        self.servo_rotate.set_rotate(speed=0)
    
    def elevation_pole(self):
        """
        升起信号塔
        """
        # 设置舵机旋转速度, 升起信号塔
        self.servo_rotate.set_rotate(127)
        time.sleep(7.8)
    # ============== 信号塔部分 end ==============
    
    
    # ============== 高球部分 start ==============
    def high_ball_reset(self):
        self.servo_high.set(80, self.high_ball_cfg["grip_init_angle"])
        self.arm.set(
            horiz_pos=self.high_ball_cfg["arm_init_pos"][0], 
            vert_pos=self.high_ball_cfg["arm_init_pos"][1]
        )
        
    def high_ball_pick(self): 
        # 上升
        self.arm.set_offset(
            horiz_offset=0, 
            vert_offset=self.high_ball_cfg["grip_grab_angle"]
        )
        
        # 抓球
        self.servo_high.set(50, self.high_ball_cfg["grip_grab_angle"])
        time.sleep(1)
        
        # 下降
        self.arm.set(
            horiz_pos=self.high_ball_cfg["arm_init_pos"][0], 
            vert_pos=self.high_ball_cfg["arm_init_pos"][1]
        )
    # ============== 高球部分 end ==============
    
    
    # ============== 圆柱部分 start ==============
    def cylinder_arm_reset(self, arm_side=0, cylinder_id=1, reset=False):
        """
        重置机器臂到圆柱任务初始位置
        arm_side: 左1 右-1
        """
        if reset or arm_side != 0:
            # 合上抓手
            self.arm.set_grap_angle(angle=self.cylinder_cfg["grab_close"][cylinder_id-1])
            # print(f'arm_init_pos: {self.reset_cfg["arm_init_pos"][0]}')
            # 上升, 防止与平台碰撞
            self.arm.set(
                horiz_pos=self.reset_cfg["arm_init_pos"][0], 
                vert_pos=self.reset_cfg["arm_init_pos"][1]
            )
        
        # 如果机械手需要在左右, 则旋转机械手
        if arm_side != 0:
            # 旋转机械手
            self.arm.switch_side(side=arm_side)
        
        # 复位到抓取初始位置
        self.arm.set(
            horiz_pos=self.cylinder_cfg["arm_init_pos"][0], 
            vert_pos=self.cylinder_cfg["arm_init_pos"][1]
        )
        
        # 打开抓夹
        self.arm.set_grap_angle(angle=self.cylinder_cfg["grab_open"][cylinder_id-1])
        
    # -------------- 抓取圆柱部分 start --------------
    def cylinder_pick_up(self, cylinder_id):
        """
        抓取圆柱
        cylinder_id: 圆柱id号
        """
        # 打开抓夹
        self.arm.set_grap_angle(angle=self.cylinder_cfg["grab_open"][cylinder_id-1])
        time.sleep(0.5)
        
        # 水平移动, 接触圆柱
        self.arm.set_offset(
            horiz_offset=self.cylinder_cfg["arm_grab_horiz_offset"][cylinder_id - 1] * self.arm.side, 
            vert_offset=0
        )
        # 抓取
        self.arm.set_grap_angle(self.cylinder_cfg["grab_close"][cylinder_id-1])
        time.sleep(0.5)
        
        # 上升
        self.arm.set_offset(
            horiz_offset=0, 
            vert_offset=self.cylinder_cfg["arm_grab_vert_offset"][cylinder_id - 1]
        )
    # -------------- 抓取圆柱部分 end --------------
    
    # -------------- 放下圆柱部分 start --------------
    def cylinder_put_down(self, cylinder_id):
        """
        放下圆柱
        cylinder_id: 圆柱id号
        """
        # 下降, 接触平台
        self.arm.set_offset(
            horiz_offset=0, 
            vert_offset=self.cylinder_cfg["arm_drop_vert_offset"][cylinder_id-1], 
            speed=[0.1, 0.05]
        )
        time.sleep(0.2)
        # time.sleep(2)
        
        # 放下
        self.arm.set_grap_angle(self.cylinder_cfg["grab_open"][cylinder_id-1])
        time.sleep(0.5)
    # -------------- 放下圆柱部分 end --------------
    # ============== 圆柱部分 end ==============
    
    # ============== ocr部分 start ==============
    def ocr_arm_reset(self, reset=False):
        """
        初始化ocr时arm位置
        """
        if reset:
            # 抓夹
            self.arm.set_grap_angle(angle=self.ball_cfg["grab_close"])
            self.arm.set(
                horiz_pos=self.reset_cfg["arm_init_pos"][0], 
                vert_pos=self.reset_cfg["arm_init_pos"][1]
            )
        
        # 初始化检测部分
        self.arm.set(
            horiz_pos=self.ocr_cfg["arm_init_pos"][0], 
            vert_pos=self.ocr_cfg["arm_init_pos"][1]
        )
        self.arm.switch_side(side=self.ocr_cfg["arm_side"])
        self.arm.set_grap_angle(angle=self.ocr_cfg["grip_angle"])
    # ============== ocr部分 end ==============
    
    
    # ============== 营地任务 start ==============
    pass
    # =============== 营地任务 end ==============
    
    
    # ============== 罪犯部分 start ==============
    def criminal_arm_reset(self, reset=False):
        """
        初始化识别罪犯时arm位置
        """
        if reset:
            self.arm.set(
                horiz_pos=self.reset_cfg["arm_init_pos"][0], 
                vert_pos=self.reset_cfg["arm_init_pos"][1]
            )
        
        # 设置arm位置
        self.arm.set(
            horiz_pos=self.criminal_cfg["arm_init_pos"][0], 
            vert_pos=self.criminal_cfg["arm_init_pos"][1]
        )
        self.arm.switch_side(side=self.criminal_cfg["arm_side"])
        self.arm.set_grap_angle(angle=self.criminal_cfg["grip_angle"])
        
    def punish_criminal(self, arm_set=False):
        """
        打击罪犯
        """
        # 设置arm位置
        self.arm.set(
            horiz_pos=self.criminal_cfg["arm_hit_pos"][0], 
            vert_pos=self.criminal_cfg["arm_hit_pos"][1], 
            speed=[0.05, 0.1]
        )
        # 合上抓夹
        self.arm.set_grap_angle(angle=self.criminal_cfg["hit_angle"])
        # 打击
        hit_offset = 0.2
        self.arm.set_offset(
            horiz_offset=-hit_offset, 
            vert_offset=0
        )
        self.arm.set_offset(
            horiz_offset=hit_offset, 
            vert_offset=0
        )
    # =============== 罪犯部分 end ==============

    def reset(self):
        self.arm.reset()
        self.arm.switch_side(-1)

    def task_test(self):
        self.servo_high.set(50, 50)


def task_reset():
    task = MyTask()
    task.reset()
    # task.reset()


def test_arm():
    task = MyTask()
    
    task.arm.set_offset(horiz_offset=-0.05, vert_offset=0.05)
    task.arm.set(horiz_pos=-0.06, vert_pos=0.06)
    # task


def test_block():
    task = MyTask()
    
    # 1
    task.block_arm_reset()
    # task.block_grab_first()
    
    # # 2
    # task.block_arm_reset()
    # task.block_grab_second()
    

def test_ball():
    task = MyTask()
    
    task.ball_arm_reset()
    
    
def test_cylinder():
    task = MyTask()
    
    task.cylinder_arm_reset(arm_side=-1)
    
    # task.cylinder_pick_up(cylinder_id=3)
    # task.cylinder_put_down(cylinder_id=3)
    
    # task.cylinder_arm_reset()
    

def test_high_ball():
    task = MyTask()
    
    task.high_ball_reset()
    
    task.high_ball_pick()


if __name__ == "__main__":
    # import argparse
    # args = argparse.ArgumentParser()
    # args.add_argument('--op', type=str, default="reset")
    # args = args.parse_args()
    # print(args)
    # if args.op == "reset":
    #     task_reset()
    # if args.op == "stop":
    #     punish_criminal_test("infer_back_end.py")
    
    # test_arm()
    # test_block()
    # test_ball()
    # test_cylinder()
    test_high_ball()
