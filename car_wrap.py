#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import time
import math
import copy
import signal
import platform
import threading

import numpy as np

from collections import defaultdict
from filterpy.kalman import KalmanFilter

from simple_pid import PID

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import ryzquo_tools.path_tools as rpt
from ryzquo_tools.log_tools import logger

from camera import Camera
from vehicle import MecanumBase, SensorAi, SerialWrap, ArmBase, ScreenShow, Key4Btn, Infrared, LedLight
from task_func import MyTask
from infer_cs import ClintInterface, Bbox
from tools import CountRecord, get_yaml, IndexWrap, ThreadAction
from ernie_bot import ErnieBotWrap, ActionPrompt, HumAttrPrompt
from bd_face_sdk import BdAgeSdk


def select_program(programs, order, win_order):
    """
    从给定的程序列表中，根据指定的顺序和胜利顺序，选择并格式化显示程序信息
    Args:
        programs: 程序列表
        order: 关注的程序在列表中的位置
        win_order: 需要关注的程序是第一个win的
            与 order 结合使用, 确定要显示的程序范围

    Returns:
        经过格式化处理的程序信息
    """
    dis_str = ""

    # 计算起始索引
    start_index = order - win_order
    for i, program in enumerate(programs):
        # 跳过起始索引之前的程序
        if i < start_index:
            continue

        # 将当前程序转为字符串
        now = str(program)
        # 如果是关注的程序，则添加>>>标记
        if i == order:
            now = ">>> " + now
        # 否则添加其在程序列表中的序号与.
        else:
            now = str(i + 1) + "." + now
        # 如果程序长度大于19, 则截断
        if len(now) >= 19:
            now = now[:19]
        # 否则换行
        else:
            now = now + "\n"
        # 将当前程序添加到结果字符串中
        dis_str += now

        # 如果已经添加了5个程序，则停止
        if i - start_index == 4:
            break
    return dis_str


def kill_other_python():
    """
    终止除当前Python进程外的所有Python进程。
    该函数遍历所有进程，找到名称中包含'python'的进程，并排除当前进程，然后终止这些进程。
    Args:

    Returns:

    """
    import psutil
    import os
    import time

    # 获取当前进程的PID
    pid_me = os.getpid()

    python_processes = []
    # 遍历所有进程，筛选出Python进程
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # 筛选条件：进程名包含'python'，且命令行参数长度小于30，避免终止一些非目标Python进程
            if 'python' in proc.info['name'].lower() and len(proc.info['cmdline']) > 1 and len(
                    proc.info['cmdline'][1]) < 30:
                python_processes.append(proc.info)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            # 忽略不存在、权限不足或僵尸进程的错误
            pass

    # 终止筛选出的Python进程
    for process in python_processes:
        # 如果进程PID不等于当前进程的PID，则终止该进程
        if int(process['pid']) != pid_me:
            # 使用SIGKILL信号强制终止进程
            os.kill(int(process['pid']), signal.SIGKILL)
            # 等待0.3秒，确保进程终止
            time.sleep(0.3)
            

def get_python_processes():
    import psutil
    # print("----------")
    python_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if 'python' in proc.info['name'].lower() \
                    and proc.info['cmdline'] is not None \
                    and len(proc.info['cmdline']) > 1 \
                    and len(proc.info['cmdline'][1]) < 100:
                info = [proc.info['pid'], proc.info['cmdline'][1]]
                python_processes.append(info)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return python_processes


def stop_process(py_str):
    import psutil
    py_lists = get_python_processes()
    # print(py_lists)
    for py_procees in py_lists:
        pid_id, py_name = py_procees[0], py_procees[1]
        # print(pid_id, py_name)
        if py_str in py_procees[1]:
            psutil.Process(pid_id).terminate()
            print("stop", py_name)
            return


def limit(value, value_range):
    """
    将给定值限制在指定范围内
    Args:
        value: 需要限制的原始值
        value_range: 允许的值的范围

    Returns:
        返回限制后的值，确保该值不会超出指定范围
    """
    return max(min(value, value_range), 0 - value_range)


class PidCal2():
    """
    y方向和转向角两个pid集合成一个
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)

    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class LanePidCal():
    """
    初始化车道保持PID控制器
    Args:
        cfg_pid_y: Y轴方向的PID配置字典，包括比例、积分和微分系数。
        cfg_pid_angle: 角度方向的PID配置字典，包括比例、积分和微分系数。
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        print(cfg_pid_y)
        print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class DetPidCal():
    """
    DetPidCal 类用于初始化两个 PID 控制器，一个用于 Y 轴控制，另一个用于角度控制。
    Args:
        cfg_pid_y: Y 轴 PID 控制器的配置参数，包括比例、积分和微分系数。默认为 None。
        cfg_pid_angle: 角度 PID 控制器的配置参数，包括比例、积分和微分系数。默认为 None。
    """
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class LocatePidCal():
    """
    定位PID校准类，用于设置和获取PID控制器的输出，以实现对目标位置的控制
    """
    def __init__(self):
        """
        创建并配置两个PID控制器，分别用于X和Y方向的定位
        """
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)

    def set_target(self, x, y):
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out


class MyCar(MecanumBase):
    STOP_PARAM = True

    def __init__(self):
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        # 获取自己文件所在的目录路径
        self.path_dir = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))

        logger.info("my car init ok {}".format(time.time() - start_time))
        
        # 任务
        self.task = MyTask()
        # 显示
        self.display = ScreenShow()

        # 获取配置
        self.car_cfg = get_yaml(os.path.join(self.path_dir, "config_car.yml"))
        # 根据配置设置sensor
        self.sensor_init(self.car_cfg)

        self.car_pid_init(self.car_cfg)

        self.camera_init(self.car_cfg)
        
        # 巡航滤波
        self.last_smoothed_angle_speed = 0
        # 卡尔曼滤波器
        self.kf = self.init_kalman_filter()

        # paddle推理初始化
        self.display.show("infer init ...")
        self.paddle_infer_init()
        self.display.show("done")
        
        # 文心一言分析初始化
        self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 按键线程结束标志
        self._end_flag = False
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.setDaemon(True)
        self.thread_key.start()
        
    def init_kalman_filter(self):
        """初始化卡尔曼滤波器"""
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.array([0., 0.])  # 初始状态 [位置, 速度]
        kf.F = np.array([[1., 0.1], [0., 1.]])  # 状态转移矩阵
        kf.H = np.array([[1., 0.]])  # 测量函数
        kf.P *= 100.  # 初始协方差矩阵
        kf.R = 5  # 测量噪声
        kf.Q = np.array([[0.01, 0.1], [0.1, 0.5]])  # 过程噪声
        return kf

    def sensor_init(self, cfg):
        cfg_sensor = cfg['io']
        # print(cfg_sensor)
        self.key = Key4Btn(cfg_sensor['key'])
        self.light = LedLight(cfg_sensor['light'])
        self.left_sensor = Infrared(cfg_sensor['left_sensor'])
        self.right_sensor = Infrared(cfg_sensor['right_sensor'])

    def car_pid_init(self, cfg):
        # lane_pid_cfg = cfg['lane_pid']
        # self.pid_y = PID(lane_pid_cfg['y'], 0, 0)
        # self.lane_pid = LanePidCal(**cfg['lane_pid'])
        # self.det_pid = DetPidCal(**cfg['det_pid'])
        self.lane_pid = PidCal2(**cfg['lane_pid'])
        self.lane_alpha = cfg['alpha']
        self.lane_pid_level_dict = {
            '1': (PidCal2(**cfg['lane_pid_1']), cfg['alpha_1']), 
            '2': (PidCal2(**cfg['lane_pid_2']), cfg['alpha_2']), 
            '3': (PidCal2(**cfg['lane_pid_3']), cfg['alpha_3']), 
            '4': (PidCal2(**cfg['lane_pid_4']), cfg['alpha_4']), 
            '5': (PidCal2(**cfg['lane_pid_5']), cfg['alpha_5']), 
            '6': (PidCal2(**cfg['lane_pid_6']), cfg['alpha_6']), 
            '7': (PidCal2(**cfg['lane_pid_7']), cfg['alpha_7']), 
            '8': (PidCal2(**cfg['lane_pid_8']), cfg['alpha_8']), 
            '9': (PidCal2(**cfg['lane_pid_9']), cfg['alpha_9']), 
            # 十字和直角 危险
            '10': (PidCal2(**cfg['lane_pid_10']), cfg['alpha_10']), 
        }
        self.det_pid = PidCal2(**cfg['det_pid'])

    def camera_init(self, cfg):
        cfg_camera = cfg['camera']
        # 初始化前后摄像头设置
        self.cap_front = Camera(cfg_camera['front'])
        # 侧面摄像头
        self.cap_side = Camera(cfg_camera['side'])

    def paddle_infer_init(self):
        # 全局巡航
        self.crusie = ClintInterface('lane')
        # 前置左右方向识别
        self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface('task')
        # 人体跟踪
        self.mot_hum = ClintInterface('mot')
        # 人体属性
        self.attr_hum = ClintInterface('humattr')
        # 人物年龄
        self.age_hum = BdAgeSdk()
        # ocr识别
        self.ocr_rec = ClintInterface('ocr')
        # 识别为None
        self.last_det = None

    def ernie_bot_init(self):
        self.hum_analysis = ErnieBotWrap()
        self.hum_analysis.set_prompt(str(HumAttrPrompt()))

        self.action_bot = ErnieBotWrap()
        self.action_bot.set_prompt(str(ActionPrompt()))

    @staticmethod
    def get_cfg(path):
        from yaml import load, Loader
        # 把配置文件读取到内存
        with open(path, 'r') as stream:
            yaml_dict = load(stream, Loader=Loader)
        port_list = yaml_dict['port_io']
        # 转化为int
        for port in port_list:
            port['port'] = int(port['port'])
        print(yaml_dict)

    # 延时函数
    def delay(self, time_hold):
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            if time.time() - start_time > time_hold:
                break

    # 按键检测线程
    def key_thread_func(self):
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_btn()
                # print(key_val)
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)

    # 根据某个值获取列表中匹配的结果
    @staticmethod
    def get_list_by_val(list, index, val):
        for det in list:
            if det[index] == val:
                return det
        return None

    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        self.mecanum_wheel(sp[0], sp[1], sp[2])
        while True:
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.mecanum_wheel(sp[0], sp[1], sp[2])
        self.mecanum_wheel(0, 0, 0)

    #  高级移动，按着给定速度进行移动，直到满足条件
    def move_advance(self, sp, value_h=None, value_l=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False

        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.get() / 1000
            print("val:", val_sensor)
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False

        for i in range(times):
            self.move_base(sp, end_fuction, stop=False)
        if stop:
            self.stop()

    def do_action_list(self, acitons):
        def move_func(x, y, angle):
            logger.info(f"move start - x: {x}, y: {y}, angle: {angle}")
            self.set_pos_offset(
                pos=[x, y, angle], 
                sp=[0.3, 0.3, 130]
            )
            logger.info(f"move end")
            
        def reverse_move_func(x, y, angle):
            logger.info(f"reverse move start - x: {-x}, y: {-y}, angle: {-angle}")
            self.set_pos_offset(
                pos=[-x, -y, -angle], 
                sp=[0.3, 0.3, 130]
            )
            logger.info(f"reverse move end")

        def light_func(time_dur=None, count=None, time_interval=None):
            if time_dur == None:
                time_dur = 0.4
            if count == None:
                count = 1
            if time_interval == None:
                time_interval = 0.3
                
            logger.info(f"light start - time_dur: {time_dur}, count: {count}, time_interval: {time_interval}")
            for c in range(count):
                # 亮
                self.light.set_light(1, 255, 0, 0)
                self.light.set_light(2, 0, 255, 0)
                self.light.set_light(3, 0, 0, 255)
                self.light.set_light(4, 255, 0, 255)
                # 等待
                time.sleep(time_dur)
                # 熄灭
                for i in range(1, 5):
                    self.light.set_light(i, 0, 0, 0)
                # 如果多次执行，则等待一段时间
                if count != 1:
                    time.sleep(time_interval)
            logger.info(f"light end")

        def beep_func(time_dur=None, count=None, time_interval=None):
            if time_dur is None:
                time_dur = 0.25
            if count == None:
                count = 1
            if time_interval == None:
                time_interval = 0.1
                
            logger.info(f"beep start - time_dur: {time_dur}, count: {count}, time_interval: {time_interval}")
            for c in range(count):
                self.beep(time_dur=time_dur)
                time.sleep(time_dur)
                # 如果多次执行，则等待一段时间
                if count != 1:
                    time.sleep(time_interval)
            logger.info(f"beep end")
                    
        def wait_func(time_dur=None):
            if time_dur == None:
                time_dur = 0.5
            
            logger.info(f"wait start - time_dur: {time_dur}")
            self.delay(time_dur)
            logger.info(f"wait end")
            
        def actions_run(actions):
            f_actions = copy.deepcopy(actions)
            for action in f_actions:
                func = action_map[action['func']]
                # 删除func, 剩下的都是params
                action.pop('func')
                # 执行
                func(**action)
                time.sleep(0.1)
            
            # # 返回营地
            # s_actions = copy.deepcopy(actions)
            # for action in s_actions[::-1]:
            #     print(action)
            #     if action['func'] == "move":
            #         func = action_map[action['func']]
            #         # 删除func, 剩下的都是params
            #         action.pop('func')
            #         # 执行
            #         reverse_move_func(**action)
            #         time.sleep(0.1)

        action_map = {
            'move': move_func, 
            'light': light_func, 
            'beep': beep_func, 
            'car_wait': wait_func
        }
        if acitons is None or len(acitons)==0:
            acitons = {
                'primary_items': [], 
                'secondary_items': []
            }
        primary_action = acitons['primary_items']
        second_action = acitons['secondary_items']
        
        # 记录起点
        start_pos = np.array(self.get_odom())
        
        # 执行次要动作
        t_second_action_run = ThreadAction(
            actions_run, 
            args=(second_action, ), 
        )
        t_second_action_run.start()
        # 执行主要动作
        actions_run(primary_action)
        # 等待次要动作完成
        t_second_action_run.join()
        
        # 回到原来位置
        self.set_pos(start_pos, sp=[0.3, 0.3, 130])

    # 计算两个坐标的距离
    def calculation_dis(self, pos_dst, pos_src):
        return math.sqrt((pos_dst[0] - pos_src[0]) ** 2 + (pos_dst[1] - pos_src[1]) ** 2)

    def lane_det_location(
        self, 
        speed, 
        pt_tar=None, 
        dis_out=0.22, 
        side=1, 
        det='task', 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行位置定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if pt_tar is None:
            pt_tar = [0, 1, 'pedestrian', 0, -0.15, -0.48, 0.24, 0.82]

        # 设置推理模型
        if det == "task":
            infer = self.task_det
        else:
            infer = self.mot_hum

        # 坐标位置error转换相对位置
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
        else:
            error_adjust = np.array([-1, 1, 1, 1])

        # 初始化PID
        # TODO: 需要调一下
        # pid_x = PID(0.5, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))

        # 计数次数
        cr = 4
        # 用于相同记录结果的计数类
        x_count = CountRecord(cr)
        y_count = CountRecord(cr)
        w_count = CountRecord(cr)

        # 输出速度
        out_x = speed
        out_y = 0

        # 此时设置相对初始位置
        self.set_pos_relative()

        # 目标位置: 类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = \
            pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]

        # 是否找到目标
        find_tar = False
        # 是否定位成功
        flag_location = False
        while True:
            if self._stop_flag:
                return
            # 计算已经移动的距离
            _pos_x, _pos_y, _pos_omega = self.get_pos_relative()

            # 如果已经移动的距离大于越界距离并且没有找到目标, 则退出
            if abs(_pos_x) > dis_out or abs(_pos_y) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    break
                
            # 目标检测
            img_side = self.cap_side.read()
            det_rets = infer(img_side)
            
            # 进行排序，此处排列按照由近及远的顺序
            det_rets.sort(
                key=lambda x: (x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2
            )
            # print(det_rets)
            # 找到最近对应的类别，类别存在第一个位置
            det = self.get_list_by_val(
                list=det_rets, index=2, val=tar_label
            )
            # 如果存在对应内容, 则定位, 否则就重新获取
            if det:
                find_tar = True
                # 获取 bbox
                det_bbox = det[4:]

                if show:
                    # 将 bbox 转为 rect 形式
                    rect = Bbox(box=det_bbox, size=img_side.shape[:2][::-1]).get_rect()
                    # 绘制 bbox
                    cv2.rectangle(
                        img=img_side,
                        pt1=rect[:2], pt2=rect[2:],
                        color=(0, 255, 0), thickness=2
                    )
                
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = (
                    (np.array(det_bbox) - np.array(tar_bbox)) * error_adjust
                ).tolist()

                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                if abs(bbox_error[0]) > 0.1:
                    bbox_error[1] = 0
                    bbox_error[2] = 0
                    bbox_error[3] = 0

                # 检测偏差值连续小于阈值时，跳出循环
                # flag_x = x_count(abs(bbox_error[0]) < 0.02)
                # flag_y = y_count(abs(bbox_error[2]) < 0.025)
                flag_x = x_count(abs(bbox_error[0]) < 0.035)
                flag_y = y_count(abs(bbox_error[2]) < 0.035)

                out_x = pid_x(bbox_error[0])
                # out_y = pid_y(bbox_error[1])
                out_y = pid_w(bbox_error[2])
                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if not is_location and flag_x and flag_y:
                    print("location ok")
                    flag_location = True
                    break
                
                if is_location:
                    print(
                        f"det_bbox: {det_bbox}, tar_bbox: {tar_bbox}", 
                        f"error_x:{bbox_error[0]:.2f}, error_y:{bbox_error[2]:.2f}, ", 
                        f"out_x:{out_x:.2f}, out_y:{out_y:.2f}"
                    )

                if show:
                    cv2.imshow("side", img_side)
                    cv2.waitKey(1)
                
            else:
                x_count(False)
                y_count(False)
                
            if not is_location:
                self.mecanum_wheel(out_x, out_y, 0)
            
        # 停止
        self.mecanum_wheel(0, 0, 0)

        return flag_location
    
    def lane_det_location_base(
        self, 
        speed, 
        infer, 
        pt_tar, 
        pid_x, 
        pid_y, 
        error=0.035, 
        count_times=3, 
        dis_out=0.22, 
        side=1, 
        need_x=True, 
        need_y=True, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行位置定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 坐标位置error转换相对位置
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
        else:
            error_adjust = np.array([-1, 1, 1, 1])

        # 用于相同记录结果的计数类
        x_count = CountRecord(count_times)
        y_count = CountRecord(count_times)

        # 输出速度
        out_x = speed
        out_y = 0

        # 此时设置相对初始位置
        self.set_pos_relative()

        # 目标位置: 类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = \
            pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]

        # 是否找到目标
        find_tar = False
        # 是否定位成功
        flag_location = False
        while True:
            if self._stop_flag:
                return
            # 计算已经移动的距离
            _pos_x, _pos_y, _pos_omega = self.get_pos_relative()

            # 如果已经移动的距离大于越界距离并且没有找到目标, 则退出
            if abs(_pos_x) > dis_out or abs(_pos_y) > dis_out:
                if not find_tar:
                    logger.info("超出任务区域")
                    break
                
            # 目标检测
            img_side = self.cap_side.read()
            det_rets = infer(img_side)
            
            # 进行排序，此处排列按照由近及远的顺序
            det_rets.sort(
                key=lambda x: (x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2
            )
            # print(det_rets)
            # 找到最近对应的类别，类别存在第一个位置
            det = self.get_list_by_val(
                list=det_rets, index=2, val=tar_label
            )
            # print(det)
            # 如果存在对应内容, 则定位, 否则就重新获取
            if det:
                find_tar = True
                # 获取 bbox
                det_bbox = det[4:]

                if show:
                    # 将 bbox 转为 rect 形式
                    rect = Bbox(box=det_bbox, size=img_side.shape[:2][::-1]).get_rect()
                    # 绘制 bbox
                    cv2.rectangle(
                        img=img_side,
                        pt1=rect[:2], pt2=rect[2:],
                        color=(0, 255, 0), thickness=2
                    )
                
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = (
                    (np.array(det_bbox) - np.array(tar_bbox)) * error_adjust
                ).tolist()

                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                if abs(bbox_error[0]) > 0.1:
                    bbox_error[1] = 0
                    bbox_error[2] = 0
                    bbox_error[3] = 0

                # 检测偏差值连续小于阈值时，跳出循环
                flag_x = x_count(abs(bbox_error[0]) < error) if need_x else True
                flag_y = y_count(abs(bbox_error[2]) < error) if need_y else True

                out_x = pid_x(bbox_error[0])
                out_y = pid_y(bbox_error[2])
                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if not is_location and flag_x and flag_y:
                    print("定位完成")
                    flag_location = True
                    break
                
                if is_location:
                    print(
                        f"det_bbox: {det_bbox}, tar_bbox: {tar_bbox}", 
                        f"error_x:{bbox_error[0]:.2f}, error_y:{bbox_error[2]:.2f}, ", 
                        f"out_x:{out_x:.2f}, out_y:{out_y:.2f}"
                    )
                
            else:
                x_count(False)
                y_count(False)
                
            if not is_location:
                self.mecanum_wheel(out_x, out_y, 0)
                
            if show:
                cv2.imshow("side", img_side)
                cv2.waitKey(1)
        
        # 停止
        self.mecanum_wheel(0, 0, 0)

        return flag_location
    
    def lane_det_location_block(
        self, 
        speed, 
        pt_tar=None, 
        dis_out=0.22, 
        side=-1, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行方块定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if pt_tar is None:
            pt_tar = [
                0, 0, "block", 0, 
                -0.12259615384615384, 0.3605769230769231, 0.8894230769230769, 1.2692307692307692
            ]

        # 设置推理模型
        infer = self.task_det

        # 初始化PID
        # --- 原来的
        # pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        # pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))
        # --- 调参1
        # pid_x = PID(0.346, 0, 0.017, setpoint=0, output_limits=(-speed, speed))
        # pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))
        # --- 调参2
        pid_x = PID(0.346, 0, 0.017, setpoint=0, output_limits=(-speed, speed))
        pid_w = PID(0.855, 0, 0.017, setpoint=0, output_limits=(-0.15, 0.15))
        
        return self.lane_det_location_base(
            speed, infer, pt_tar, 
            pid_x=pid_x, pid_y=pid_w, 
            error=0.05, side=side, dis_out=dis_out, 
            is_location=is_location, show=show
        )
    
    def lane_det_location_ball(
        self, 
        speed, 
        pt_tar=None, 
        dis_out=0.22, 
        side=1, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行小球定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if pt_tar is None:
            pt_tar = [0, 1, 'pedestrian', 0, -0.15, -0.48, 0.24, 0.82]

        # 设置推理模型
        infer = self.task_det

        # 初始化PID
        pid_x = PID(0.38, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_w = PID(0.98, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))

        return self.lane_det_location_base(
            speed, infer, pt_tar, 
            pid_x=pid_x, pid_y=pid_w, 
            error=0.05, side=side, dis_out=dis_out, 
            is_location=is_location, show=show
        )
    
    def lane_det_location_cylinder(
        self, 
        speed, 
        cylinder_id, 
        cylinder_tar_list=None, 
        found_tars=None, 
        dis_out=0.22, 
        side=1, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行圆柱定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if cylinder_tar_list is None:
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
        
        # 已找到的目标
        if found_tars is None:
            found_tars = []
        
        # 当前需要查找的目标
        pt_tar = cylinder_tar_list[cylinder_id-1]
        # 需要查找的其他目标
        tar_need = [1, 2, 3]
        for i in found_tars + [cylinder_id]:
            tar_need.remove(i)

        # 设置推理模型
        infer = self.task_det

        # 初始化PID
        # pid_x = PID(0.4, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_x = PID(0.35, 0, 0, setpoint=0, output_limits=(-speed, speed))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))
        
        # 坐标位置error转换相对位置
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
        else:
            error_adjust = np.array([-1, 1, 1, 1])
            
        # 用于相同记录结果的计数类
        count_times = 3
        x_count = CountRecord(count_times)
        y_count = CountRecord(count_times)

        # 输出速度
        out_x = speed
        out_y = 0
        
        error = 0.035
        
        # 此时设置相对初始位置
        self.set_pos_relative()

        # 目标位置: 类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_label, tar_bbox = pt_tar[2], pt_tar[4:]
            
        # 是否找到目标
        find_tar = False
        # 是否定位成功
        flag_location = False
        # 找到的所有目标位置
        found_tar_list = {
            1: None,
            2: None,
            3: None
        }
        four_tar_error_list = {
            1: 100, 
            2: 100, 
            3: 100
        }
        # print(f"tar_needs: {tar_need}, ori found_tar_list: {found_tar_list}")
        while True:
            if self._stop_flag:
                return
            # 计算已经移动的距离
            _pos_x, _pos_y, _pos_omega = self.get_pos_relative()

            # 如果已经移动的距离大于越界距离并且没有找到目标, 则退出
            if abs(_pos_x) > dis_out or abs(_pos_y) > dis_out:
                if not find_tar:
                    logger.info("超出任务区域")
                    break
                
            # 目标检测
            img_side = self.cap_side.read()
            det_rets = infer(img_side)
            # print(f"ori det_rets: {det_rets}")
            
            # 进行排序，此处排列按照由近及远的顺序
            det_rets.sort(
                key=lambda x: (x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2
            )
            # print(f"sort det_rets: {det_rets}")
            
            # 找到最近对应的类别，类别存在索引2处
            det = self.get_list_by_val(
                list=det_rets, index=2, val=tar_label
            )
            # 找到的其他目标
            for tar_need_id in tar_need:
                # 目标
                tar_need_pt = cylinder_tar_list[tar_need_id-1]
                # 目标标签
                tar_need_label = tar_need_pt[2]
                # 获取指定目标
                tar_cnt = self.get_list_by_val(
                    list=det_rets, index=2, val=tar_need_label
                )
                # 如果目标存在
                if tar_cnt:
                    # 计算误差
                    o_bbox_error = (
                        (np.array(tar_cnt[4:]) - np.array(tar_need_pt[4:])) \
                        * error_adjust
                    ).tolist()
                    # 如果误差小于当前最大, 则认为找到, 将当前小车位置存入列表
                    if abs(o_bbox_error[0]) < min(1, four_tar_error_list[tar_need_id]):
                        # print(f"f- need_label: {tar_need_label}, cnt: {tar_cnt}")
                        # found_tar_list[tar_need_id] = tar_cnt
                        found_tar_list[tar_need_id] = self.get_odom()
                        four_tar_error_list[tar_need_id] = abs(o_bbox_error[0])
            # print(det)
            
            # 如果存在对应内容, 则定位, 否则就重新获取
            if det:
                find_tar = True
                # 获取 bbox
                det_bbox = det[4:]

                if show:
                    # 将 bbox 转为 rect 形式
                    rect = Bbox(box=det_bbox, size=img_side.shape[:2][::-1]).get_rect()
                    # 绘制 bbox
                    cv2.rectangle(
                        img=img_side,
                        pt1=rect[:2], pt2=rect[2:],
                        color=(0, 255, 0), thickness=2
                    )
                
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = (
                    (np.array(det_bbox) - np.array(tar_bbox)) * error_adjust
                ).tolist()

                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                if abs(bbox_error[0]) > 0.1:
                    bbox_error[1] = 0
                    bbox_error[2] = 0
                    bbox_error[3] = 0
                
                # 检测偏差值连续小于阈值时，跳出循环
                flag_x = x_count(abs(bbox_error[0]) < error)
                flag_y = y_count(abs(bbox_error[2]) < error)

                out_x = pid_x(bbox_error[0])
                out_y = pid_w(bbox_error[2])
                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if not is_location and flag_x and flag_y:
                    print("定位完成")
                    found_tar_list[cylinder_id] = self.get_odom()
                    flag_location = True
                    break
                
                if is_location:
                    print(
                        f"det_bbox: {det_bbox}, tar_bbox: {tar_bbox}", 
                        f"error_x:{bbox_error[0]:.2f}, error_y:{bbox_error[2]:.2f}, ", 
                        f"out_x:{out_x:.2f}, out_y:{out_y:.2f}"
                    )
                
            else:
                x_count(False)
                y_count(False)
                
            if not is_location:
                self.mecanum_wheel(out_x, out_y, 0)
                
            if show:
                cv2.imshow("side", img_side)
                cv2.waitKey(1)
        
        # 停止
        self.mecanum_wheel(0, 0, 0)

        return flag_location, found_tar_list
    
    def lane_det_location_camp(
        self, 
        speed, 
        pt_tar=None, 
        dis_out=0.22, 
        side=-1, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行圆柱定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if pt_tar is None:
            pt_tar = [
                0, 1, "camp", 0, 
                -0.13701923076923078, 0.2956730769230769, 1.6009615384615385, 1.3990384615384615
            ]

        # 设置推理模型
        infer = self.task_det

        # 初始化PID
        pid_x = PID(0.346, 0, 0.017, setpoint=0, output_limits=(-speed, speed))
        pid_w = PID(0.855, 0, 0.017, setpoint=0, output_limits=(-0.15, 0.15))

        return self.lane_det_location_base(
            speed, infer, pt_tar, 
            pid_x=pid_x, pid_y=pid_w, 
            error=0.05, side=side, dis_out=dis_out, 
            is_location=is_location, show=show
        )
    
    def lane_det_location_mot(
        self, 
        speed, 
        pt_tar=None, 
        dis_out=0.22, 
        side=-1, 
        is_location=False, 
        show=False
    ):
        """
        侧面摄像头进行人物定位
        pt_tar: 定位目标位置
        dis_out: 退出距离
        side: 左1 右-1
        det: 检测任务类型
        """
        # 设置默认定位目标(类别, id, 标签, 置信度, 归一化bbox[x_c, y_c, w, h])
        if pt_tar is None:
            pt_tar = [
                0, 1, 'pedestrian', 0, 
                0.021875, 0.06458333333333334, 0.45, 1.6125
            ]

        infer = self.mot_hum

        # 初始化PID
        pid_x = PID(0.29, 0, 0.017, setpoint=0, output_limits=(-speed, speed))
        pid_w = PID(0.8, 0, 0.017, setpoint=0, output_limits=(-0.15, 0.15))
        
        return self.lane_det_location_base(
            speed, infer, pt_tar, 
            pid_x=pid_x, pid_y=pid_w, 
            error=0.05, side=side, dis_out=dis_out, 
            need_x=True, need_y=False, 
            is_location=is_location, show=show
        )
        
    def exponential_moving_average(self, current_value, last_average, alpha=0.2):
        """指数移动平均滤波"""
        return alpha * current_value + (1 - alpha) * last_average

    def lane_base(
        self, speed, end_function, stop=STOP_PARAM, 
        use_model='base', 
        pid_level='default'
    ):
        """
        基本巡航函数, 根据结束函数 end_function 确定移动距离
        Args:
            speed: x方向速度
            end_function: 结束条件函数
            stop: 小车停止标志

        Returns:

        """
        if use_model == 'base':
            infer_lane = self.crusie
            
        if pid_level == 'default':
            l_pid = self.lane_pid
            l_alpha = self.lane_alpha
        else:
            l_pid, l_alpha = self.lane_pid_level_dict[pid_level]
        
        while True:
            # 判断程序是否结束
            if self._stop_flag:
                return

            # 获取前相机当前图像
            image = self.cap_front.read()
            
            # 巡航识别, 计算y方向误差和角度误差
            error_y, error_angle = infer_lane(image)
            
            # # 使用卡尔曼滤波器
            # self.kf.predict()
            # self.kf.update(error_angle)
            # # 滤波后的误差估计
            # error_angle = self.kf.x[0]  

            # 使用PID控制器计算角度，输入为平滑后的误差
            y_speed, angle_speed = l_pid.get_out(-error_y, -error_angle)

            # # 对PID输出的角度进行EMA滤波
            # smoothed_angle_speed = self.exponential_moving_average(
            #     angle_speed, self.last_smoothed_angle_speed, 
            #     alpha=l_alpha
            # )
            # self.last_smoothed_angle_speed = smoothed_angle_speed
            # angle_speed = smoothed_angle_speed
            
            # # 把 erroy_y 和 error_angle 保存到文件中, 用于调参
            # with open(rpt.join_path(rpt.get_root(__file__), "lane_error.txt"), "a+") as f:
            #     f.write(f"{error_y},{error_angle}\n")  

            # debug
            # print(
            #     f"error_y:{error_y}, error_angle:{error_angle}, "
            #     f"y_speed:{y_speed}, angle_speed:{angle_speed}"
            # )

            # 通过x, y和角速度omega控制麦轮移动
            self.mecanum_wheel(speed, y_speed, angle_speed)
            # 判断是否满足结束条件
            if end_function():
                break

        # 根据stop参数决定在满足条件处是否需要停止程序
        if stop:
            self.stop()

    def lane_det_base(self, speed, end_function, stop=STOP_PARAM):
        """
        根据前方相机识别到的目标移动
        Args:
            speed: x方向速度
            end_function: 结束条件函数
            stop: 小车停止标识

        Returns:

        """
        y_speed = 0
        angle_speed = 0

        while True:
            # 读取前方相机图像
            image = self.cap_front.read()
            # 前置识别
            dets_ret = self.front_det(image)
            # 此处检测简单不需要排序
            # dets_ret.sort(key=lambda x: x[4]**2 + (x[5])**2)

            # 判断是否检测到内容
            if len(dets_ret) > 0:
                # 获取检测到的第一个目标信息
                det = dets_ret[0]
                det_bbox = det[4:]
                
                # 计算y方向误差
                error_y = det_bbox[0]
                # 计算x方向误差
                dis_x = 1 - det_bbox[1]
                # print(f"dis_x: {dis_x}")
                # 判断是否满足结束条件
                if end_function(dis_x):
                    break
                
                error_angle = error_y / dis_x
                # pid计算y方向速度和角速度
                y_speed, angle_speed = self.det_pid.get_out(error_y, error_angle)

            # 通过x, y和角速度omega控制麦轮移动
            self.mecanum_wheel(speed, y_speed, angle_speed)
            
            # 判断是否满足结束条件
            if end_function(0):
                break

        # 判断在到达满足结束条件处是否需要停下来
        if stop:
            self.stop()

    def lane_det_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        根据前方相机识别到的目标移动一段时间
        Args:
            speed: x方向速度
            time_dur: 移动时间
            stop: 小车停止标识

        Returns:

        """
        # 计算结束时间
        time_end = time.time() + time_dur

        def end_function(x):
            """
            结束条件函数, 当前时间大于结束时间
            """
            return time.time() > time_end

        self.lane_det_base(speed, end_function, stop=stop)

    def lane_det_dis2pt(self, speed, dis_end, stop=STOP_PARAM):
        """
        根据前方相机识别到的目标移动一段路程
        Args:
            speed: x方向速度
            dis_end: 移动的路程终点, 越小越近
            stop: 小车结束标识
        """
        def end_function(x):
            """
            结束条件函数, 传入的值小于路程终点且不等0
            """
            return x != 0 and x < dis_end
        self.lane_det_base(speed, end_function, stop=stop)

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        巡航一段时间
        Args:
            speed: x方向速度
            time_dur: 移动消耗的时间
            stop: 小车停止标志

        Returns:

        """
        # 计算结束时间
        time_end = time.time() + time_dur

        def end_function(x):
            """
            结束条件函数, 当前系统时间大于结束时间
            """
            return time.time() > time_end
        self.lane_base(speed, end_function, stop=stop)

    def lane_dis(
        self, speed, dis_end, stop=STOP_PARAM, 
        use_model='base', 
        pid_level='default'
    ):
        """
        巡航一段路程
        Args:
            speed: x方向速度
            dis_end: 移动的路程终点
            stop: 小车停止标志

        Returns:

        """
        def end_function():
            """
            结束条件函数，当前的路程变量值大于路程终点
            """
            return self.get_dis_traveled() > dis_end
        self.lane_base(
            speed, end_function, stop=stop, 
            use_model=use_model, 
            pid_level=pid_level
        )

    def lane_dis_offset(
        self, speed, dis_hold, stop=STOP_PARAM, 
        use_model='base', 
        pid_level='default'
    ):
        """
        以当前位置为起点，移动一定距离
        Args:
            speed: x方向速度
            dis_hold: 需要移动的路程
            stop: 小车停止标志

        Returns:

        """
        # 获取当前路程变量值
        dis_start = self.get_dis_traveled()
        # 计算目标路程, 起点路径+需要移动的路程
        dis_stop = dis_start + dis_hold
        self.lane_dis(
            speed, dis_stop, stop=stop, 
            use_model=use_model, 
            pid_level=pid_level
        )

    def lane_sensor(
            self, 
            speed, 
            value_h=None, value_l=None,
            dis_offset=0.0, 
            times=1, sides=1,
            stop=STOP_PARAM, 
            use_model='base', 
            pid_level='default'
    ):
        """
        使用左右传感器巡航
        Args:
            speed: 速度
            value_h: 传感器高阈值
            value_l: 传感器低阈值
            dis_offset: 路程偏移量, 用于调整行驶路径
            times: 检测次数
            sides: 使用的传感器位置(1:左, -1:右)
            stop: 小车停止标识

        Returns:

        """
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        # 默认使用左侧传感器
        _sensor_usr = self.left_sensor
        # 如果 sides 为 -1, 则使用右侧传感器
        if sides == -1:
            _sensor_usr = self.right_sensor

        # 用于检测开始过渡部分的标记
        flag_start = False

        def end_function():
            """
            结束巡航判断函数
            """
            nonlocal flag_start
            # 将传感器计数转换为毫伏
            val_sensor = _sensor_usr.get() / 1000
            print(f"val_sensor: {val_sensor}, flag_start: {flag_start}")
            # 如果传感器值在范围内就返回True
            if value_l < val_sensor < value_h:
                return flag_start
            else:
                flag_start = True
                return False

        # 进行 timers 次基本车道线巡航
        for i in range(times):
            self.lane_base(
                speed, end_function, stop=False, 
                use_model=use_model, pid_level=pid_level
            )
        # 根据需要进行路程偏移巡航
        self.lane_dis_offset(
            speed, dis_offset, stop=stop, 
            use_model=use_model, pid_level=pid_level
        )

    def get_card_side(self):
        """
        使用前方相机识别卡片, 判断是左转还是右转
        Returns:
            左转返回1, 右转返回-1, 否则不返回任何内容
        """
        # 初始化转向卡片计数器
        count_side = CountRecord(3)
        while True:
            # 检测是否停止
            if self._stop_flag:
                return

            # 读取前方相机图像
            image = self.cap_front.read()
            # 目标检测
            dets_ret = self.front_det(image)
            # 如果没有检测到转向卡片, 记录为-1, 结束本次循环
            if len(dets_ret) == 0:
                count_side(-1)
                continue

            # 检测到转向卡片, 取检测到的每一个目标
            det = dets_ret[0]
            det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
            # 连续检测到相同目标结果超过3次
            if count_side(det_label):
                if det_label == 'turn_left':
                    return 1
                elif det_label == 'turn_right':
                    return -1

    def get_hum_attr(self, pt_tar, show=False, cr=3, age_limit=[24, 50]):
        """
        获取目标人物的属性
        Args:
            pt_tar: 检测到的目标人物信息
            show: 是否在图像上显示检测结果
            age_limit 年龄段临界值 [中年最小, 老年最小]
                        

        Returns:
            字典, 包含目标人物的属性信息
        """
        # 类别, id, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]

        # 初始化目标人物属性计数器
        tar_count = CountRecord(cr)
        while True:
            # 判断是否停止
            if self._stop_flag:
                return

            # 读取侧面相机图像
            image = self.cap_side.read()
            # 人体跟踪
            dets_ret = self.mot_hum(image)
            # 按照距离的远近排序
            # 根据每个检测结果(dets_ret)到目标边界框(tar_bbox)中心点的距离进行排序
            # 距离的计算方式为: 欧氏距离, 即(x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2
            dets_ret.sort(key=lambda x: (x[4] - tar_bbox[0]) ** 2 + (x[5] - tar_bbox[1]) ** 2)
            # print(f"dets_ret: {dets_ret}")
    
            # 如果检测到了目标
            if len(dets_ret) != 0:
                # 获取 bbox
                det_rect_normalise = dets_ret[0][4:]
                # print(det_rect_normalise)

                # 将 bbox 转为 rect 形式
                rect = Bbox(box=det_rect_normalise, size=image.shape[:2][::-1]).get_rect()
                # print(rect)
                if show:
                    cv2.rectangle(image, rect[:2], rect[2:], (0, 255, 0), 2)

                # 检测到的人物图像
                image_hum = image[rect[1]:rect[3], rect[0]:rect[2]]
                if not (min(rect) < 0) and image_hum is not None:
                    # 将检测到的人物传入人物属性检测器进行检测
                    res = self.attr_hum(image_hum)

                    # 对检测到的人物属性进行计数
                    if tar_count(res):
                        # # 稳定后处理年龄
                        # age = self.age_hum(image_hum)
                        # print(f"age: {age}")
                        # if 0 <= age < age_limit[0]:
                        #     res['young'] = True
                        #     res['middle'] = False
                        #     res['old'] = False
                        # elif age_limit[0] < age <= age_limit[1]:
                        #     res['young'] = False
                        #     res['middle'] = True
                        #     res['old'] = False
                        # elif age > age_limit[1]:
                        #     res['young'] = True
                        #     res['middle'] = False
                        #     res['old'] = True
                        # else:
                        #     res['young'] = False
                        #     res['middle'] = False
                        #     res['old'] = True
                        
                        return res

                    if show:
                        print(f"人物属性: {res}")

            # 展示检测到的人物图像
            if show:
                cv2.imshow("hum", image)
                cv2.waitKey(1)
            # logger.info(res)

    def compare_humattr(self, criminal_attr, hum_attr):
        """
        比较两个人物属性的匹配度
        Args:
            criminal_attr: 罪犯属性
            hum_attr: 要比较的人物属性

        Returns:
            匹配的属性数量
        """
        # 匹配的属性数量
        count = 0
        # 匹配的属性
        match_keys = []
        
        # 检测罪犯属性是否存在
        if criminal_attr is None:
            return count, match_keys
        
        # 遍历罪犯属性
        for key, val in criminal_attr.items():
            # 如果是bool类型的属性, 直接比较是否相等
            if type(val) is bool:
                # 如果人物属性中不存在当前罪犯属性, 就返回False
                if val == hum_attr[key]:
                    match_keys.append(key)
                    count += 1
            # 如果是背包、肩包或手提包, 均视为带着包
            elif key == 'bag' and val in ['Backpack', 'ShoulderBag', 'HandBag']:
                if hum_attr[key] in ['Backpack', 'ShoulderBag', 'HandBag']:
                    match_keys.append(key)
                    count += 1
            # 忽略大小写比较
            elif val.lower() == hum_attr[key].lower():
                match_keys.append(key)
                count += 1

        return count, match_keys

    def get_ocr(self, is_test=False, timeout=10):
        # 定义一个三次的计数器
        text_count = CountRecord(3)
        start_time = time.time()
        # 每次识别到的文字
        all_text = []
        while True:
            if self._stop_flag:
                return
            # 读取侧面相机图像
            img = self.cap_side.read()
            # 文本识别
            # text = self.ocr_rec(img[180:, 200:440])
            text = self.ocr_rec(img)
            all_text.append(text)
            print(f"ocr: {text}")
            
            if is_test:
                cv2.imshow("ocr", img)
                if cv2.waitKey(1) == 27:
                    return 
                continue
            
            if (time.time() - start_time) > timeout:
                text_freq = defaultdict(int)

                # 统计每个文本段落的出现次数
                for line in all_text:
                    if len(line) <= 3:
                        continue
                    text_freq[line] += 1

                # 找出出现次数最多的文本段落
                if text_freq:
                    max_freq = max(text_freq.values())
                    most_frequent_texts = [
                        text for text, freq in text_freq.items() if freq == max_freq
                    ]

                    return most_frequent_texts[0] if most_frequent_texts else None
                else:
                    return None
            
            # 简单滤波, 如果三次检测到相同的值就认为稳定, 返回结果
            if text and text_count(text):
                return text
            # 否则获取所有文本中出现率最高的文字

    def yiyan_get_humattr(self, text):
        """
        文心一言获取人物属性
        """
        # 根据 text 获取大模型生成结果, 获取解析后的json字符串
        return self.hum_analysis.get_res_json(text)

    def yiyan_get_actions(self, text, request_timeout=10):
        """
        大模型获取任务
        """
        text = text.replace("，", ", ").replace("。", "")
        return self.action_bot.get_res_json(
            text, request_timeout=request_timeout
        )

    def debug(self):
        # self.arm.arm_init()
        # self.set_xyz_relative(0, 100, 60, 0.5)
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            res = self.crusie(image)
            det_front = self.front_det(image)
            error = res[0]
            angle = res[1]
            image = self.cap_side.read()
            det_task = self.task_det(image)
            det_hum = self.mot_hum(image)

            logger.info("")
            logger.info("--------------")
            logger.info("error:{} angle{}".format(error, angle))
            logger.info("front:{}".format(det_front))
            logger.info("task:{}".format(det_task))
            logger.info("hum_det:{}".format(det_hum))
            logger.info("left:{} right:{}".format(self.left_sensor.get(), self.right_sensor.get()))
            self.delay(0.5)

    def walk_lane_test(self):
        end_function = lambda: True
        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    def close(self):
        self._stop_flag = False
        self._end_flag = True
        self.thread_key.join()
        self.cap_front.close()
        self.cap_side.close()
        # self.grap_cam.close()

    def manage(
            self, programs_list: list, order_index=0,
            py_filename="car_start.py", is_test=False
    ):
        def all_task():
            """
            所有任务
            """
            st = time.time()
            for func in programs_list:
                func()
            logger.info(f"全元素 - 用时: {time.time() - st} s")

        def lane_test():
            """
            单巡航
            """
            st = time.time()
            self.lane_dis_offset(1, 17, pid_level='10')
            logger.info(f"全图巡航 - 用时: {time.time() - st} s")
            
        def m_reset():
            # 重置旋转信号塔舵机
            self.task.servo_rotate.set_angle(speed=90, angle=0)
            self.task.servo_rotate.set_rotate(speed=0)
            # 重置放球平台
            self.task.servo_ball.set_rotate(speed=0)
            self.task.servo_ball.set_angle(
                speed=80, angle=self.task.reset_cfg["servo_ball_init_angle"]
            )
            self.task.servo_ball.set_rotate(speed=0)
            self.task.arm.reset()

        if is_test:
            programs_suffix = [lane_test, self.task.arm.reset]
        else:
            programs_suffix = [all_task, m_reset]

        programs = programs_suffix.copy()
        # 当前选中的序号
        win_num = 5
        win_order = 0
        # 把programs的函数名转字符串
        logger.info(order_index)
        programs_str = [str(i.__name__) for i in programs]
        logger.info(programs_str)
        dis_str = select_program(programs_str, order_index, win_order)
        self.display.show(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # self.button_all.event()
            btn = self.key.get_btn()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            # button_num = car.button_all.clicked()

            if btn != 0:
                # logger.info(btn)
                # 长按1按键，退出
                if btn == 5:
                    self.beep()
                    logger.info("退出程序")
                    # run_flag = True
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 2:
                        self.beep()
                        # 序号减1
                        if order_index == 0:
                            order_index = len(programs) - 1
                            win_order = win_num - 1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = select_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 4:
                        self.beep()
                        # 序号加1
                        if order_index == len(programs) - 1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num - 1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = select_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行
                        # 调用别的程序
                        dis_str = "\n{} running......\n".format(str(programs_str[order_index]))
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = select_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()

                        # 自动跳转下一条
                        if order_index == len(programs) - 1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num - 1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = select_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 1:
                        # 重启程序
                        print('程序重启...')
                        self.display.show("restart")
                        self.beep()
                        stop_process(py_filename)

                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)

            time.sleep(0.02)

        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()


def main_ori():
    # kill_other_python()
    my_car = MyCar()
    my_car.lane_time(0.3, 5)

    # my_car.lane_dis_offset(0.3, 1.2)
    # my_car.lane_sensor(0.3, 0.5)
    # my_car.debug()

    # text = "犯人没有带着眼镜，穿着短袖"
    # criminal_attr = my_car.hum_analysis.get_res_json(text)
    # print(criminal_attr)
    # my_car.task.reset()
    # pt_tar = my_car.task.punish_crimall(arm_set=True)
    # hum_attr = my_car.get_hum_attr(pt_tar)
    # print(hum_attr)
    # res_bool = my_car.compare_humattr(criminal_attr, hum_attr)
    # print(res_bool)
    # pt_tar = [0, 1, 'pedestrian',  0, 0.02, 0.4, 0.22, 0.82]
    # for i in range(4):
    #     my_car.set_pos_offset([0.07, 0, 0])
    #     my_car.lane_det_location(0.1, pt_tar, det="mot", side=-1)
    # my_car.close()
    # text = my_car.get_ocr()
    # print(text)
    # pt_tar = my_car.task.pick_up_ball(arm_set=True)
    # my_car.lane_det_location(0.1, pt_tar)

    my_car.close()
    # my_car.debug()
    # while True:
    #     text = my_car.get_ocr()
    #     print(text)

    # my_car.task.reset()
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.lane_time(0.3, 5)
    # my_car.debug()

    # my_car.debug()

    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # logger.info(time.time())
    # my_car.lane_task_location(0.3, 2)

    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # logger.info("test:", test_ord)
    # car_test(test_ord)


if __name__ == "__main__":
    my_car = MyCar()
    
    # 测试动作执行
    # v1.0
    # my_car.do_action_list([
    #     # {'func': 'light', 'time_dur': 1, 'count': 3, 'time_interval': 0.5}, 
    #     # {'func': 'light', 'time_dur': 5}, 
    #     # {'func': 'light', 'time_dur': 3, 'count': 5, 'time_interval': 1}, 
    #     # {'func': 'beep', 'time_dur': 1, 'count': 3, 'time_interval': 0.5}, 
    #     # {'func': 'beep', 'count': 3}, 
    #     # {'func': 'move', 'x': 0, 'y': 0, 'angle': 0}
    # ])
    # v2.0
    # my_car.do_action_list(
    #     {
    #         'primary_items': [
    #             {'func': 'car_wait', 'time_dur': 3}, 
    #             {'func': 'move', 'x': -0.1, 'y': 0, 'angle': 0}
    #         ], 
    #         'secondary_items': []
    #     }
    # )
    # print(my_car.get_ocr())
    
    # 测试大模型
    # text = "前面有动物，需要鸣警示3次，之后常亮3向右移0.1米，在向左格0.3米"
    # "车子故障，需紧急停车，请发出灯光闪烁3次，并伴有蜂鸣器。", 
    # "后移0.2m, 右移0.1m, 左移0.3m, 旋转90度, 伴有灯光长亮5秒", 
    # text = "鸣笛2秒共3次, 间隔1秒"
    # text = "此处危险，需要前移0.1m,然后返回。"
    # text = "后移0.2m, 右移0.1m, 左移0.3m, 右转90度, 伴有灯光长亮5秒"
    # text = "车子故障，需紧急停车，请发出3次灯光闪烁，等待3秒后出发"
    # text = "此处危险，需要发出3次声音，然后出发。"
    # text = "车子出发前，需要连续发声3秒，然后才能离开。"
    text = "此处停留需要发出3声警报，然后再闪烁三次灯光。"
    print(f"text: {text}")
    actions_map = my_car.yiyan_get_actions(
        text, 
        request_timeout=20
    )
    print(f"actions_map: {actions_map}")
    my_car.do_action_list(actions_map)
    
    # # 测试鸣笛
    # my_car.beep()
    # time.sleep(1)
    
    # # 测试人物属性识别
    # my_car.task.criminal_arm_reset(reset=True)
    # while True:
    #     attr = my_car.get_hum_attr(
    #         pt_tar=[
    #             0, 1, 'pedestrian', 0, 
    #             -0.1015625, 0.12291666666666666, 0.453125, 1.6125
    #         ], 
    #         show=True
    #     )
    #     print(attr)
    #     # time.sleep(1)
    