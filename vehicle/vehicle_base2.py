#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import yaml
import time
import shutil

import numpy as np

from simple_pid import PID
from threading import Thread

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from ryzquo_tools.log_tools import logger

from vehicle.base.controller2 import *
from tools.base.tools_class import ThreadAction


def limit_val(val, min_val, max_val):
    """
    越界检测
    Args:
        val: 待检测的值
        min_val: 允许的最小值
        max_val: 允许的最大值

    Returns:
        在范围内返回 val 本身, 小于 min_val 则返回 min_val, 大于 max_val 则返回 max_val
    """
    return max(min(val, max_val), min_val)


class MotoConvert:
    """
    轮子运动参数 -> 编码器计数 转换工具类
    """

    def __init__(self, perimeter=None) -> None:
        """

        Args:
            perimeter: 轮子周长
        """
        if perimeter is None:
            perimeter = 0.06 * math.pi
        # 编码器一圈的编码值
        self.encoder_resolution = 2016
        # 编码速度转换值
        self.speed_rate = 100

        # 每个编码器读数对应的实际位移分辨率
        self.dis_resolution = perimeter / self.encoder_resolution

    def set_perimeter(self, perimeter):
        """
        设置轮子周长
        Args:
            perimeter: 周长

        Returns:

        """
        self.dis_resolution = perimeter / self.encoder_resolution
        # print(self.dis_resolution)

    def set_diameter(self, diameter):
        """
        设置轮子直径
        Args:
            diameter:

        Returns:

        """
        self.dis_resolution = diameter * math.pi / self.encoder_resolution

    def sp2virtual(self, speed):
        """
        速度 -> 编码器输出值
        Args:
            speed: 速度

        Returns:
            编码器输出
        """
        # 速度转为编码器输出
        speed_encoder = speed / self.dis_resolution
        # 编码器输出转为控制器设置值
        speed_out = int(speed_encoder / self.speed_rate)
        # 进行越界检测
        speed_out = limit_val(speed_out, -100, 100)
        return int(speed_out)

    def dis2true(self, encoder_dis):
        """
        编码器读数 -> 实际位移值
        Args:
            encoder_dis: 编码器读数

        Returns:
            实际位移值
        """
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    def sp2true(self, speed):
        """
        速度 -> 实际位移值
        Args:
            speed: 速度

        Returns:
            实际位移值
        """
        # 控制器速度转为encoder输出
        speed_encoder = int(speed * self.speed_rate)
        speed_out = speed_encoder * self.dis_resolution
        return speed_out

    def encoder2dis(self, encoder_dis):
        """
        编码器读数 -> 实际位移值
        与 dis2true 作用相同
        Args:
            encoder_dis:

        Returns:
            实际位移值
        """
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    def dis2encoder(self, dis):
        """
        实际位移值 -> 编码器读数
        Args:
            dis: 实际位移值

        Returns:
            编码器读数
        """
        encoder_out = dis / self.dis_resolution
        return encoder_out


class ArmBase():
    """
    机械臂
    用于读取和管理机械臂的配置信息, 包括硬件连接参数和控制参数
    """

    def __init__(self) -> None:
        """
        从配置文件读取参数，如果读取失败则使用默认参数
        """
        # 获取当前文件所在目录的绝对路径
        self.path_dir = os.path.dirname(os.path.abspath(__file__))
        # 拼接配置文件路径
        self.yaml_path = os.path.join(self.path_dir, "config_arm.yaml")

        # 重置标识(改了一下, 为真时不从文件读取)
        flag_reset = False
        # 读取成功就退出循环, 失败就获取默认设置
        while True:
            try:
                # 打开并读取配置文件
                if not flag_reset:
                    with open(self.yaml_path, 'r') as stream:
                        self.config = yaml.load(stream, Loader=yaml.FullLoader)

                # 从配置文件中提取各项参数
                # 水平阈值
                self.horiz_threshold = self.config["horiz_threshold"]
                # 垂直阈值
                self.vert_threshold = self.config["vert_threshold"]
                # 水平电机参数(端口号, 反转参数)
                self.horiz_moto_param = self.config["horiz_moto_param"]
                # 垂直电机参数(端口号, 反转参数)
                self.vert_moto_param = self.config["vert_moto_param"]

                # 水平电机的极限检测端口
                self.horiz_limit_port = self.config["horiz_limit_port"]
                # 垂直电机的极限检测端口
                self.vert_limit_port = self.config["vert_limit_port"]

                # 水平电机PID参数
                self.horiz_pid_param = self.config["horiz_pid_param"]
                # 垂直电机PID参数
                self.vert_pid_param = self.config["vert_pid_param"]

                # 机械臂抓取舵机(PWM D1)
                self.grap_servo_port = self.config["grap_servo_port"]
                # 机械臂左右活动舵机(id-4)
                self.arm_servo_id = self.config["arm_servo_id"]

                # 抓取部分的朝向(100 0 90)三个角度
                self.side = self.config["side"]

                # 定义初始化属性
                self.pos_enable = self.config["pos_enable"]
                if not self.pos_enable:
                    raise Exception("pos_enable is False")
                self.pos_vert = self.config["pos_vert"]
                self.pos_vert_start = self.pos_vert
                self.pos_horiz = self.config["pos_horiz"]
                self.pos_horiz_start = self.pos_horiz
                self.angle_list = self.config["angle_list"]

                break
            except:
                # 默认设置
                self.config = {
                    "pos_enable": True, "side": -1,
                    "pos_vert": 0,
                    "pos_horiz": 0,
                    "angle_list": [90, 0, -95],
                    "horiz_moto_param": {"port_id": 5, "reverse": -1},
                    "vert_moto_param": {"port_id": 6, "reverse": 1},
                    "horiz_limit_port": 2, "vert_limit_port": 3, "grap_servo_port": 1, "arm_servo_id": 4,
                    "vert_pid_param": {"Kp": 20, "Ki": 0.0, "Kd": 0.4, "setpoint": 0},
                    "horiz_pid_param": {"Kp": 20, "Ki": 0.0, "Kd": 0.4, "setpoint": 0},
                    "vert_threshold": [-0.2, 0.2], "horiz_threshold": [-0.23, 0.23]
                }
                self.pos_vert = 0
                self.pos_horiz = 0
                self.save_yaml()
                flag_reset = True

        # 定义垂直移动电机 限位传感器，速度转换参数
        self.vert_moto = Motor(**self.vert_moto_param)
        # 垂直电机编码器
        self.encoder_vert = EncoderMotor(self.vert_moto_param["port_id"])
        # self.encoder_vert.reset()
        # 垂直电机编码器 encoder_vert_start 开机时的值
        self.encoder_vert_start = self.encoder_vert.get()
        # 垂直电机限位
        self.limit_vert = SensorAi(self.vert_limit_port)
        # 运动参数->编码器计数的转换器
        vert_perimeter = 0.037 * math.pi / 56 * 8
        self.vert_convert = MotoConvert(vert_perimeter)
        # 垂直电机PID
        self.vert_pid = PID(**self.vert_pid_param)

        # 定义水平移动电机 限位传感器 速度转换参数
        self.horiz_moto = Motor(**self.horiz_moto_param)
        # 水平电机编码器
        self.encoder_horiz = EncoderMotor(self.horiz_moto_param["port_id"])
        # self.encoder_horiz.reset()
        # 开机时encoder_horiz_start的值
        self.encoder_horiz_start = self.encoder_horiz.get()
        # 水平电机限位
        self.limit_horiz = SensorAi(self.horiz_limit_port)
        # 运动参数->编码器计数的转换器
        horiz_perimeter = 0.06 / 15 * 8
        self.horiz_convert = MotoConvert(horiz_perimeter)
        # 水平电机PID
        self.horiz_pid = PID(**self.horiz_pid_param)

        # 顶部按钮
        self.key = Key4Btn(2)
        # 机械臂抓取舵机
        self.grap_servo = ServoPwm(self.grap_servo_port)
        self.grap_servo.set(127, 90)
        # 机械臂左右活动舵机
        self.arm_servo = ServoBus(self.arm_servo_id)

        # 未有默认设置文件
        if flag_reset:
            self.reset()

        # 位置调整
        if self.side == 0:
            self.switch_side(-1)
        # self.arm_servo.set_angle(50, 90)

    def save_yaml(self, pos_enable=True):
        self.config["pos_enable"] = pos_enable
        self.config["pos_vert"] = self.pos_vert
        self.config["pos_horiz"] = self.pos_horiz
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream)

    def set_pos_start(self, pos_vert):
        self.pos_vert_start = self.pos_vert
        self.pos_horiz_start = self.pos_horiz
        self.save_yaml()

    def set_by_yourself(self):
        logger.info("set arm by yourself...")
        while True:
            val = self.key.get_key()
            if val == 1:
                self.vert_moto.set(30)
            elif val == 3:
                self.vert_moto.set(-30)
            elif val == 4:
                self.horiz_moto.set(30)
            elif val == 2:
                self.horiz_moto.set(-30)
            else:
                self.horiz_moto.set(0)
                self.vert_moto.set(0)
            # 更新位置
            self.update_pos(0)
            self.update_pos(1)
            # time.sleep(0.1)

    def reset(self, speed=1):
        self.grap_servo.set(127, 90)
        
        self.reset_pos_dir(dir=2, speed=speed)
        # 回到初始位置

        # self.vert_moto.set(0)
        # self.horiz_moto.set(0)

    def switch_side(self, side, speed=80):
        """
        side: 左1 右-1
        """
        if self.side != side:
            self.side = side
            logger.info("change side to {}".format(self.side))
            # print("change side to {}".format(self.side))
        else:
            return
        angle_tar = self.angle_list[side + 1]
        logger.info("change arm angle to {}".format(angle_tar))
        self.config["side"] = side
        self.save_yaml()
        # self.set(-0.12, 0.08, speed=1)
        # logger.info("change to -0.12, 0.08")
        # angle_tar = -1*side*90
        self.set_arm_angle(angle_tar, speed=speed)
        # self.set_arm_angle(angle_tar, speed=speed)
        time.sleep(0.5)

    def reset_pos_dir(self, dir=0, speed=0.1):
        """
        用于重置arm
        dir: 水平0, 垂直1
        """
        vert_flag = True
        horiz_flag = True
        if dir == 0:
            horiz_flag = False
        elif dir == 1:
            vert_flag = False
        else:
            horiz_flag = False
            vert_flag = False

        while True:
            # print(f"flag - v: {vert_flag}, h: {horiz_flag}")
            if vert_flag and horiz_flag:
                break

            if not horiz_flag:
                horiz_val = self.limit_horiz.get()
                # print("horiz_val:", horiz_val)
                if horiz_val > 1000:
                    self.move_horiz(0)
                    self.pos_horiz = 0
                    self.pos_horiz_start = 0
                    self.encoder_horiz.reset()

                    self.pos_horiz_start = 0
                    self.pos_horiz = 0
                    horiz_flag = True
                else:
                    self.move_horiz(speed)

            if not vert_flag:
                vert_val = self.limit_vert.get()
                # print("vert_val:", vert_val)
                if vert_val > 1000:
                    self.move_vert(0)
                    self.encoder_vert.reset()

                    self.pos_horiz_start = 0
                    self.pos_vert = 0
                    vert_flag = True
                else:
                    self.move_vert(0 - abs(speed))

    def set_grap_angle(self, angle):
        """
        设置抓夹张开角度
        """
        # self.grap_servo.set(50, angle)
        self.grap_servo.set(255, angle)
        
    def set_grab_dis(self, dis, speed=127):
        """
        设置抓夹张开的间距
        """
        self.grap_servo.set_angle(dis, speed=speed)

    def set_arm_angle(self, angle, speed=80):
        """
        设置抓夹指向位置
        angle: 
        """
        self.arm_servo.set_angle(speed, angle)
        if angle in self.angle_list:
            self.side = self.angle_list.index(angle) - 1
        else:
            self.side = 0

    def set_arm_dir(self, dir=0, speed=80):
        assert dir == 0 or dir == 1 or dir == -1, "dir should be 0 or 1 or -1"
        self.set_arm_angle(self.angle_list[dir + 1], speed)

    def move_vert(self, speed_vert):
        speed_out = self.vert_convert.sp2virtual(speed_vert)
        # print(speed_out)
        self.vert_moto.set(speed_out)

    def move_horiz(self, speed_horiz):
        speed_out = self.horiz_convert.sp2virtual(speed_horiz)
        # print(speed_out)
        self.horiz_moto.set(speed_out)

    def update_pos(self, axis):
        """
        更新当前机械臂位置
        axis: 垂直1 水平0
        """
        if axis == 0:
            # 相对开始时的编码值
            encoder_vert = self.encoder_vert.get() - self.encoder_vert_start
            # print("encoder_vert 5",encoder_vert)
            self.pos_vert = self.pos_vert_start + self.vert_convert.dis2true(encoder_vert)
            self.config["pos_vert"] = self.pos_vert
        else:
            # 相对开始时的编码值
            encoder_horiz = 0 - (self.encoder_horiz.get() - self.encoder_horiz_start)
            # print("encoder_horiz 6",encoder_horiz)
            self.pos_horiz = self.pos_horiz_start + self.horiz_convert.dis2true(encoder_horiz)
            self.config["pos_horiz"] = self.pos_horiz

        # print(self.pos_vert, self.pos_horiz)

    def set_offset(self, horiz_offset, vert_offset, time_run=None, speed=None):
        """
        相对于当前位置移动机器臂
        horiz_offset: 左+ 右- 
        vert_offset: 
        speed: 水平, 垂直
        """
        if speed is None:
            speed = [0.1, 0.05]
        horiz_pos = self.pos_horiz + horiz_offset
        vert_pos = self.pos_vert + vert_offset
        self.set(
            horiz_pos=horiz_pos, vert_pos=vert_pos, 
            time_run=time_run, speed=speed
        )

    def set(self, horiz_pos, vert_pos, time_run=None, speed=None):
        """
        设置机械臂绝对位置(以初始化后的位置为基准)
        """
        if speed is None:
            speed = [0.1, 0.06]

        # 控制上下限
        horiz_pos = limit_val(horiz_pos, self.horiz_threshold[0], self.horiz_threshold[1])
        vert_pos = limit_val(vert_pos, self.vert_threshold[0], self.vert_threshold[1])

        # 获取结束时间和对应速度
        time_start = time.time()
        if time_run is not None:
            assert isinstance(time_run, int) or isinstance(time_run, float), "wrong time args"
            # 根据时间求速度
            time_end = time_start + time_run
            vert_time = time_run
            horiz_time = time_run
        elif speed is not None:
            # 根据速度求时间
            if isinstance(speed, int) or isinstance(speed, float):
                # print(speed)
                speed_horiz = speed
                speed_vert = speed
            elif isinstance(speed, list) or isinstance(speed, tuple):
                speed_horiz = speed[0]
                speed_vert = speed[1]
            else:
                logger.error("wrong speed args")
                return
            horiz_time = abs(horiz_pos - self.pos_horiz) / speed_horiz
            vert_time = abs(vert_pos - self.pos_vert) / speed_vert
            time_run = max(horiz_time, vert_time)
        else:
            logger.error("wrong args")
            return
        # 超时时间
        time_end = time_start + time_run

        # 定义结束标志和到达位置标记量
        vert_flag = False
        horiz_flag = False
        count_vert = 0
        count_horiz = 0

        # 获取对应的速度和pid位置
        if vert_time < 0.01:
            speed_vert = 0.1
            vert_flag = True
        else:
            speed_vert = abs(vert_pos - self.pos_vert) / vert_time

        self.vert_pid.setpoint = vert_pos
        self.vert_pid.output_limits = (-speed_vert, speed_vert)

        if horiz_time < 0.01:
            speed_horiz = 0.1
            horiz_flag = True
        else:
            speed_horiz = abs(horiz_pos - self.pos_horiz) / horiz_time
        self.horiz_pid.setpoint = horiz_pos
        self.horiz_pid.output_limits = (-speed_horiz, speed_horiz)
        # 开始移动前，位置信息定义，如果中间中断此时位置信息无用
        self.save_yaml(pos_enable=False)
        while True:
            # 到达结束标志结束
            if vert_flag and horiz_flag:
                break
            # 获取剩余时间
            time_remain = time_end - time.time()
            # print("time remain:", time_remain)
            # 超时处理
            if time_remain < -0.5:
                logger.warning("timeout")
                # 超时停止
                self.move_horiz(0)
                self.move_vert(0)
                break
            if not vert_flag:
                dis_vert = vert_pos - self.pos_vert
                # logger.debug("dis_vert:", dis_vert)
                # logger.debug("pos vert:", self.pos_vert)
                # 到达定义位置
                if abs(dis_vert) < 0.005:
                    count_vert += 1
                    # print("count_vert:", count_vert)
                    if count_vert > 10:
                        vert_flag = True
                        self.move_vert(0)
                        continue
                else:
                    count_vert = 0

                # 重置初始化位置
                if self.limit_vert.get() > 1000:
                    self.pos_vert = 0
                    self.pos_vert_start = 0
                    self.encoder_vert_start = self.encoder_vert.get()

                    self.save_yaml()
                    # 移动方向如果向下，则停止移动
                    if dis_vert < 0:
                        self.move_vert(0)
                        vert_flag = True
                        continue
                # pid控制输出
                speed_vert = self.vert_pid(self.pos_vert)
                # print("speed_vert:", speed_vert)
                self.move_vert(speed_vert)
                self.update_pos(0)

            if not horiz_flag:
                dis_horiz = horiz_pos - self.pos_horiz
                # print("dis_horiz:", dis_horiz)
                # print("pos horiz:", self.pos_horiz)
                # 到达定义位置
                if abs(dis_horiz) < 0.005:
                    count_horiz += 1
                    # print("count_horiz:", count_horiz)
                    if count_horiz > 10:
                        horiz_flag = True
                        self.move_horiz(0)
                        continue
                else:
                    count_horiz = 0
                # 重置初始化位置
                if self.limit_horiz.get() > 1000:

                    self.pos_horiz = 0
                    self.pos_horiz_start = 0
                    self.encoder_horiz_start = self.encoder_horiz.get()
                    # 移动方向如果向右，则停止移动
                    if dis_horiz > 0:
                        self.move_horiz(0)
                        horiz_flag = True
                        continue

                speed_horiz = self.horiz_pid(self.pos_horiz)
                # print("speed_horiz:", speed_horiz)
                self.move_horiz(speed_horiz)
                self.update_pos(1)
            # time.sleep(0.005)
        self.save_yaml()
        # logger.debug("pos vert:{}, horiz:{}".format(self.pos_vert, self.pos_horiz))


def get_pid(kp, ki, kd, val, setpoint=0):
    my_pid = PID(kp, ki, kd, setpoint=setpoint)
    my_pid.output_limits = (-val, val)
    return my_pid


class VehicleBase:
    def __init__(self) -> None:
        # self.ser = SerialWrap()
        # self.ser.ping_port()
        # print("VehicleBase init ok")
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, "config_vehicel.yaml")

        self.buzzer = Beep()

        wheel_perimeter = 0.06 * math.pi
        self.wheel_convert = MotoConvert(wheel_perimeter)

        self.base_moto = Motor4()

    def beep(self):
        self.buzzer.set(200, 10)

    def run2(self, speed_l, speed_r):
        speed_l = self.wheel_convert.sp2virtual(speed_l)
        speed_r = self.wheel_convert.sp2virtual(speed_r)
        # print(speed_l, speed_r)
        self.base_moto.set(speed_l, 0 - speed_r, speed_l, 0 - speed_r)

    def run4(self, *speeds):
        # print(speeds)
        dir = -1
        if isinstance(speeds[0], list):
            sp_tar = speeds[0]
        else:
            sp_tar = list(speeds)
        for i in range(4):
            dir = dir * -1
            sp_tar[i] = int(dir * self.wheel_convert.sp2virtual(sp_tar[i]))
        # print(sp_tar)
        self.base_moto.set(*sp_tar)


class MecanumBase():
    def __init__(self) -> None:
        logger.info("mecanum init")
        # 配置文件存储路径
        self.yaml_name = "config_vehicel.yaml"
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, self.yaml_name)
        # 根据路径获取配置信息
        self.get_yaml_config()

        self.buzzer = Beep()

        # 控制轮子电机
        self.base_moto = Motor4()

        self.encoders_motors = EncoderMotors()
        self.encoders_motors.reset()

        # 计算近似半径
        self.radius1 = (self.rx + self.ry) / 2
        self.radius2 = (self.rx + self.ry) * 2

        self.dis_now = 0
        # 小车初始位置
        self.pos_start = np.array([0, 0, 0])
        # 车子整体前进的路程变量
        self.dis_traveled = 0
        # 车子起始位置相对世界坐标系
        self.odom_x = 0
        self.odom_y = 0
        self.odom_theta = 0
        self.speed_now = np.array([0, 0, 0])

        # pid控制
        self.pid_turn = PID(self.pid_turn_params["kp"], self.pid_turn_params["ki"], self.pid_turn_params["kd"])
        self.pid_turn.setpoint = 0
        self.pid_turn.output_limits = (
            -abs(self.pid_turn_params["output_limit"]), 
            abs(self.pid_turn_params["output_limit"])
        )

        self.pid_dis_x = PID(self.pid_dis_x_params["kp"], self.pid_dis_x_params["ki"], self.pid_dis_x_params["kd"])
        self.pid_dis_x.setpoint = 0
        self.pid_dis_x.output_limits = (
            -abs(self.pid_dis_x_params["output_limit"]), abs(self.pid_dis_x_params["output_limit"]))

        self.pid_dis_y = PID(self.pid_dis_y_params["kp"], self.pid_dis_y_params["ki"], self.pid_dis_y_params["kd"])
        self.pid_dis_y.setpoint = 0
        self.pid_dis_y.output_limits = (
            -abs(self.pid_dis_y_params["output_limit"]), abs(self.pid_dis_y_params["output_limit"]))

        self.last_encoders = self.encoders_motors.get()
        self.odom_update_flag = True
        self.last_time = time.time()

        self.odom_thread = Thread(target=self.odom_update, args=())
        self.odom_thread.daemon = True
        self.odom_thread.start()
        time.sleep(0.2)

        # self.base_motors.set_speed(speeds)

    def get_yaml_config(self):
        while True:
            try:
                with open(self.yaml_path, 'r') as stream:
                    self.config = yaml.load(stream, Loader=yaml.FullLoader)

                # 轮胎直径
                self.wheel_diameter = self.config["wheel_diameter"]
                # 根据存储的轮胎直径获取轮胎的周长 
                self.wheel_perimeter = self.wheel_diameter * math.pi
                self.wheel_convert = MotoConvert(self.wheel_perimeter)
                self.rx = self.config["rx"]
                self.ry = self.config["ry"]

                self.pid_turn_params = self.config["pid_turn_params"]
                self.pid_dis_x_params = self.config["pid_dis_x_params"]
                self.pid_dis_y_params = self.config["pid_dis_y_params"]
                break
            except Exception as e:
                # print(e)
                self.config = {
                    "wheel_diameter": 0.06,
                    "rx": 0.305,
                    "ry": 0.28,
                    "pid_turn_params": {
                        "kp": 5,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 1.5
                    },
                    "pid_dis_x_params": {
                        "kp": 20,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 0.3
                    },
                    "pid_dis_y_params": {
                        "kp": 20,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 0.3
                    }
                }
                with open(self.yaml_path, 'w') as stream:
                    yaml.dump(self.config, stream)

    def save_yaml(self):
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream)

    def beep(self, time_dur=0.2, freq=200):
        # logger.info(f"time_dur*100: {time_dur*100}")
        self.buzzer.set(freq, int(time_dur*100))

    def run2(self, speed_l, speed_r):
        speed_l = self.wheel_convert.sp2virtual(speed_l)
        speed_r = self.wheel_convert.sp2virtual(speed_r)
        # print(speed_l, speed_r)
        self.base_moto.set(speed_l, 0 - speed_r, speed_l, 0 - speed_r)

    def run4(self, *speeds):
        """
        控制机器人的四个轮子以一定速度运动
        Args:
            *speeds: 四个轮子的速度

        Returns:

        """
        # print(speeds)
        if isinstance(speeds[0], list):
            sp_tar = speeds[0]
        else:
            sp_tar = list(speeds)

        # 表示四个轮子的方向
        run4_dir = -1
        # 将四个轮子的速度转为编码嚣输出值
        for i in range(4):
            # 1, -1, 1, -1
            run4_dir = run4_dir * -1
            sp_tar[i] = int(run4_dir * self.wheel_convert.sp2virtual(sp_tar[i]))
        # print(sp_tar)

        # 通过串口设置四个轮子的速度
        self.base_moto.set(*sp_tar)

    def stop(self):
        self.run2(0, 0)

    def get_encoders(self):
        re_list = self.encoders_motors.get()
        if not re_list:
            return [0, 0, 0, 0]
        re_list[1] = re_list[1] * -1
        re_list[3] = re_list[3] * -1
        return re_list

    def mecanum_inverse(self, vx, vy, vomega):
        """
        根据 x, y, 角速度omega 计算 四个轮子的输出速度
        Args:
            vx: x方向速度
            vy: y方向速度
            vomega: 角速度omega

        Returns:
            四个轮子的输出速度
        """
        speed_out = [0, 0, 0, 0]
        speed_out[0] = vx - vy - self.radius1 * vomega
        speed_out[1] = vx + vy + self.radius1 * vomega
        speed_out[2] = vx + vy - self.radius1 * vomega
        speed_out[3] = vx - vy + self.radius1 * vomega
        return speed_out

    def mecanum_forward(self, d_vect):
        # 车x方向位移速度
        car_dx = (d_vect[0] + d_vect[1] + d_vect[2] + d_vect[3]) / 4.0
        # 车y方向位移速度
        car_dy = (0 - d_vect[0] + d_vect[1] + d_vect[2] - d_vect[3]) / 4.0
        # 车角速度
        car_domega = (0 - d_vect[0] + d_vect[1] - d_vect[2] + d_vect[3]) / self.radius2
        # print(f"ori car do: {car_dx, car_dy, car_domega}")
        
        # # 车x方向位移速度
        # car_dx = (d_vect[0] + d_vect[1] + d_vect[2] + d_vect[3])
        # # 车y方向位移速度
        # car_dy = (0 - d_vect[0] + d_vect[1] + d_vect[2] - d_vect[3])
        # # 车角速度
        # car_domega = (0 - d_vect[0] + d_vect[1] - d_vect[2] + d_vect[3])
        # print(f"new car do: {car_dx, car_dy, car_domega}")
        
        return car_dx, car_dy, car_domega

    def mecanum_wheel(self, speed_vx, speed_vy, speed_vomega):
        """
        麦轮 移动
        Args:
            speed_vx: x方向速度
            speed_vy: y方向速度
            speed_vomega: 角速度omega

        Returns:

        """
        # self.odom_update()
        # 计算四个轮子的输出速度
        speeds = self.mecanum_inverse(speed_vx, speed_vy, speed_vomega)
        # speeds = self.
        # print(speeds)
        # 转换四个轮子的速度后设置到设备
        self.run4(speeds)

    def set_pos(self, pos, time_dur=None, sp=None):
        """
        在time_dur的时间内, 以sp的速度移动到pos位置
        """
        if sp is None:
            sp = [0.2, 0.2, self.angle_2_radians(45)]
        else:
            sp = [sp[0], sp[1], self.angle_2_radians(sp[2])]
            
        start_pos = np.array(self.get_odom())
        tar_pos = np.array(pos)

        if time_dur is None:
            # 时间=距离/速度
            # print(f"t-s: {tar_pos - start_pos}")
            # print(f"sp: {sp}")
            # print(f"(t-s)/sp: {(tar_pos - start_pos) / sp}")
            time_dur = np.max(abs((tar_pos - start_pos) / sp))
            sp = (tar_pos - start_pos) / time_dur
            # sp[2] = self.angle_2_radians(sp[2])
        else:
            # 速度=距离/时间
            sp = (tar_pos - start_pos) / time_dur
            # sp[2] = self.angle_2_radians(sp[2])

        self.pid_dis_x.setpoint = tar_pos[0]
        self.pid_dis_x.output_limits = (-abs(sp[0]), abs(sp[0]))

        self.pid_dis_y.setpoint = tar_pos[1]
        self.pid_dis_y.output_limits = (-abs(sp[1]), abs(sp[1]))

        self.pid_turn.setpoint = tar_pos[2]
        self.pid_turn.output_limits = (-abs(sp[2]), abs(sp[2]))

        start_time = time.time()
        count_flag = 0
        while True:
            current_time = time.time()
            dt = current_time - start_time
            # print(f"dt: {dt}")
            if dt > time_dur + 0.3:
                print("move pos time out")
                break
            
            # 获取当前距离值和角度值
            now_pos = np.array(self.get_odom())
            # 获取剩余距离值和角度值
            error_dis = tar_pos - now_pos
            # print(f"error_dis: {error_dis}")
            if (
                abs(error_dis[0]) < 0.01 
                and abs(error_dis[1]) < 0.01 
                and abs(error_dis[2]) < 0.1 
            ):
                count_flag += 1
                if count_flag > 20:
                    break
            else:
                count_flag = 0
            out_x = self.pid_dis_x(now_pos[0])
            out_y = self.pid_dis_y(now_pos[1])
            out_omega = self.pid_turn(now_pos[2])
            # 世界输出转为机器人输出
            angle_robot = now_pos[2]
            # 根据误差角度计算实际运行速度和角度值
            sp_x = math.cos(angle_robot) * out_x + math.sin(angle_robot) * out_y
            sp_y = 0 - math.sin(angle_robot) * out_x + math.cos(angle_robot) * out_y
            sp_omega = out_omega
            
            # print(
            #     f"now_pos: {now_pos}, tar_pos: {tar_pos}\n"
            #     f"pid out: {out_x, out_y, out_omega}\n"
            #     f"sp: {sp_x, sp_y, sp_omega}\n"
            #     f"speed_now: {self.speed_now}\n"
            #     f"====================================="
            # )

            self.mecanum_wheel(sp_x, sp_y, sp_omega)
        # print(self.get_odom())
        self.run4([0, 0, 0, 0])

    def set_pos_offset(self, pos, time_dur=None, sp=[0.2, 0.2, 45]):
        """
        x: +左 -右
        y: +左 -右
        z: +左 -右
        以当前位置为原点，移动到目标位置
        pos单位: 米 米 度
        sp单位: m/s, m/s, 度/s
        """
        current_pos = np.array(self.get_odom())
        # end_pos = current_pos + np.array(pos)

        start_angle = current_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, self.angle_2_radians(pos[2])]) + current_pos
        # tar_pos = current_pos + np.array(pos)
        logger.info(f"current_pos:{current_pos}, tar_pos:{tar_pos}")
        self.set_pos(tar_pos, time_dur, sp)
        
    def angle_2_radians(self, angle):
        return angle * math.pi / 180
    
    def radians_2_angle(self, radians):
        return radians * 180 / math.pi

    def odom_update(self):
        self.last_encoders = np.array(self.get_encoders())
        self.last_time = time.time()
        while True:
            if not self.odom_update_flag:
                break
            try:
                # 获取当前编码器值
                current_enc = np.array(self.get_encoders())

                d_enc = current_enc - self.last_encoders
                # print(
                #     f"last_enc: {self.last_encoders}", 
                #     f"current_enc: {current_enc}", 
                #     f"d_enc: {d_enc}"
                # )
                
                dt = time.time() - self.last_time
                if dt == 0:
                    dt = 0.05
                self.last_time = time.time()
                self.last_encoders = current_enc
                # 计算编码器值变化
                d_dis = self.wheel_convert.dis2true(d_enc)
                # print(f"d_dis: {d_dis}")

                # 计算这个瞬间的位移并叠加
                self.dis_traveled += np.average(d_dis)
                # print(f"dis_travel: {self.dis_traveled}")

                # 计算当前位移角度变化
                car_dx, car_dy, car_domega = self.mecanum_forward(d_dis)
                self.dis_now += car_dx
                # print(f"car do: {car_dx, car_dy, car_domega}")

                # 速度 = 距离 / 花费时间
                self.speed_now = np.array([car_dx, car_dy, car_domega]) / dt
                # print(f"speed_now: {self.speed_now}, dis_now: {self.dis_now}")

                # 计算小车当前移动的距离
                dx = car_dx * math.cos(self.odom_theta) - car_dy * math.sin(self.odom_theta)
                dy = car_dx * math.sin(self.odom_theta) + car_dy * math.cos(self.odom_theta)
                domega = car_domega
                # domega = self.speed_now[2]*3

                self.odom_x += dx
                self.odom_y += dy
                self.odom_theta += domega
                # print(
                #     f"car do: {car_dx}, {car_dy}, {car_domega}\n"
                #     f"do: {dx}, {dy}, {domega}\n"
                #     f"speed_now: {self.speed_now}\n"
                #     f"odom: {self.odom_x}, {self.odom_y}, {self.odom_theta}\n"
                #     f"real angle: {self.radians_2_angle(self.odom_theta)}"
                #     f"======================================================="
                # )
                                
            except Exception as e:
                print(f"vehicle_base2.py error: {e}")
            
            time.sleep(0.05)

    def set_pos_relative(self, pos_src=None):
        """
        设置相对位置
        """
        if pos_src is None:
            pos_src = self.get_odom()
        self.pos_start = pos_src

    def get_pos_relative(self, pos_dst=None, pos_src=None):
        """
        获取相对移动距离
        """
        if pos_dst is None:
            pos_dst = self.get_odom()
        if pos_src is None:
            pos_src = self.pos_start
        pos = np.array(pos_dst) - np.array(pos_src)
        dis_x = pos[0] * math.cos(pos_src[2]) - pos[1] * math.sin(pos_src[2])
        dis_y = pos[1] * math.cos(pos_src[2]) + pos[0] * math.sin(pos_src[2])
        dis_angle = pos[2]
        return dis_x, dis_y, dis_angle

    def get_odom(self):
        return self.odom_x, self.odom_y, self.odom_theta

    def get_dis_traveled(self):
        return self.dis_traveled

    def get_speed_closed(self):
        return self.speed_now

    def close(self):
        self.odom_update_flag = False


def ori_test():
    # ser = SerialWrap()
    car = MecanumBase()
    # car = MecanumBase()
    arm = ArmBase()
    # car.arm.set_by_yourself()

    # car.pull_init()

    # car = Car()
    # car.beep()
    while True:
        arm.set(0, 0.05, 1)
        time.sleep(1)
        arm.set(0, 0.1, 1)
        time.sleep(1)
        # car.pull_down()
        # time.sleep(5)
        # for i in range(10):
        #     car.run2(0.1, 0.1)
        #     time.sleep(0.1)
        # car.run2(0, 0)

        # time.sleep(2)
        # car.servo_rotate.set_rotate(100)
        # time.sleep(1)
        # car.servo_rotate.set_rotate(0)
        # time.sleep(2)
        # pass
        # car.mecanum_wheel_closed(0.0, 0.1, 0.0)
        # car.mecanum_wheel(0, 0.1, 0)
        # time.sleep(0.05)
    # car.pull_down()
    # arm  = ArmBase(ser)
    # arm.reset()
    # arm.horiz_moto.set(-20)
    # arm.set(-0.15, 0.1, 2)
    # arm.set_by_yourself()
    # vehicle = VehicleBase(ser)
    # vehicle.beep()
    # chasiss = MecanumBase(ser)
    # chasiss.encoders_motors.reset()
    # chasiss.run2(0.1, 0.1)
    # time.sleep(1)
    # last_time =time.time()
    # count = 0
    # chasiss.move_closed([0.25, -0.25, 0], 2.5)
    # chasiss.process_encoder()
    # while True:
    #     chasiss.mecanum_wheel(0.0, -0.3, 0.0)
    #     time.sleep(0.2)
    # time.sleep(5)
    # chasiss.close()
    # print("time cost:", time.time() - last_time)
    # print(count)


if __name__ == '__main__':
    mecanume = MecanumBase()
    # # # mecanume.set_pos_offset(pos=[0, 10, 0], time_dur=1)
    # # # print(mecanume.radians_2_angle(-0.6))
    # # mecanume.set_pos_offset(pos=[0, 0, 90], sp=[0.2, 0.2, 90])
    # mecanume.beep()
    # mecanume.beep(time_dur=4)
    # mecanume.set_pos_offset(pos=[0.465, -0.27, -36], sp=[0.4, 0.4, 160])
    # mecanume.set_pos_offset(pos=[0, -0.43, 0], sp=[0.4, 0.4, 160])
    pos_start = np.array(mecanume.get_odom())
    mecanume.set_pos_offset(pos=[0, 0.3, 0])
    mecanume.set_pos_offset(pos=[0.1, 0, 0])
    mecanume.set_pos_offset(pos=[-0.1, 0, 0])
    mecanume.set_pos(pos_start)
    
    # arm = ArmBase()
    # arm.reset()
    # while True:
    #     print(f"to 130")
    #     arm.set_grap_angle(angle=130)
    #     time.sleep(0.2)
    #     print(f"to 90")
    #     arm.set_grap_angle(angle=90)
    #     time.sleep(0.2)
    # arm.switch_side(-1)
    
    # arm.set(horiz_pos=-0.12, vert_pos=0.08)
