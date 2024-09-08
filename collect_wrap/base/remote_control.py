#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import json
import time
import threading
import subprocess

from glob import glob

# 添加根目录
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))

from ryzquo_tools.log_tools import logger

from vehicle import MecanumBase, BluetoothPad, ScreenShow
from camera import Camera


class RemoteControlCar:
    def __init__(
            self,
            cap_front_id=None, cap_side_id=None,
            det_set=None,
            cruise_set=None,
            json_name=None,
            det_start_num=0
    ) -> None:
        # 获取当前存储目录
        path_dir = os.path.abspath(os.path.dirname(__file__))
        
        self.det_start_num = det_start_num

        # 退出标志
        self.exit_flag = False

        # ====== 巡航 ======
        if cruise_set is None:
            cruise_set = "image_set"
        if not json_name:
            json_name = "data.json"
        # 拼接巡航数据存储目录
        self.cruise_dir = os.path.join(path_dir, cruise_set)
        self.data_json_path = os.path.join(self.cruise_dir, json_name)
        # 如果存在巡航数据目录
        if os.path.exists(self.cruise_dir):
            self.json_data = []
            # 从文件中读取巡航数据
            if os.path.exists(self.data_json_path):
                with open(self.data_json_path, "r") as f:
                    self.json_data = json.load(f)
            # 如果data.json存在有内容
            if self.json_data:
                # 把索引指向最大图像编号
                self.cruise_index = max(
                    [int(d['img_path'].split(".")[0]) for d in self.json_data]
                )
            # data.json不存在内容
            else:
                # 清空巡航数据目录
                subprocess.run(["find", self.cruise_dir, "-name", "*.jpg", "-delete"])
                # 清空数据
                self.json_data = []
                # 重置巡航数据索引
                self.cruise_index = 0
        # 不存在巡航数据目录
        else:
            # 创建巡航数据目录
            os.mkdir(self.cruise_dir)
            # 创建巡航数据目录
            self.json_data = []
            # 巡航数据索引
            self.cruise_index = 0
        # 初始化巡航数据
        self.speed_x = 0.30  # m/s
        self.speed_y = 0.15  # m/s
        self.ration_omega = 0.4
        self.state_base = [self.speed_x, self.speed_y, self.ration_omega]
        self.state_start = [0.3, 0.3, 0.5]
        self.car_state = [0.0, 0.0, 0.0]
        self.run_flag = False
        # 记录开始点
        self.cruise_start_point = 0

        # ====== 目录检测 ======
        if det_set is None:
            det_set = "car_data_coco"
        # 获取目标检测数据存储目录
        self.det_dir = os.path.join(path_dir, det_set)
        # 如果存在目标检测数据目录
        if os.path.exists(self.det_dir):
            # 获取目录内所有文件
            imgs_list = glob(os.path.join(self.det_dir, "*.jpg"))
            # 如果目录内有内容, 则延续编号并继续录制数据
            if len(imgs_list) > 0:
                # 目标检测数据索引指向最大图像编号
                self.det_index = self.det_start_num if self.det_start_num!=0 else max(
                    [int(os.path.basename(img_name).split(".")[0]) for img_name in imgs_list]
                )
                # 获取当前所有数据
                self.det_imglist = [os.path.basename(img_name) for img_name in imgs_list]
            else:
                if not os.path.exists(self.det_dir):
                    os.mkdir(self.det_dir)
                self.det_index = self.det_start_num if self.det_start_num!=0 else 0
                self.det_imglist = []
        else:
            if not os.path.exists(self.det_dir):
                os.mkdir(self.det_dir)
            self.det_index = self.det_start_num if self.det_start_num!=0 else 0
            self.det_imglist = []
        # 是否录制目标检测数据
        self.det_flag = False

        self.img_size = (640, 480)
        if cap_front_id is None:
            self.cap_front = Camera(2, *self.img_size)
        else:
            self.cap_front = Camera(cap_front_id, *self.img_size)

        if cap_side_id is None:
            self.cap_side = Camera(0, *self.img_size)
        else:
            self.cap_side = Camera(cap_side_id, *self.img_size)

        self.car = MecanumBase()
        self.car.beep()
        self.display = ScreenShow()

        self.blue_pad = BluetoothPad()

        # self.car_thread = threading.Thread(target=self.car_process, args=())
        # self.car_thread.daemon = True
        # self.car_thread.start()
        logger.info("remote control start!!")
        # self.display.show("press btn control\n 3 start\n 4+2 stop\n 4+v del 30pic\n 4+o del all\n")
        self.img_thread = threading.Thread(target=self.image_process, args=())
        self.img_thread.daemon = True
        self.img_thread.start()

        self.car_process()

    def car_process(self):
        pad_exit_flag = False
        while not self.exit_flag:
            keys_val = self.blue_pad.get_stick()
            if keys_val == [-1, -1, -1, -1, 0]:
                pad_exit_flag = False
                self.car_state = [0.0, 0.0, 0.0]
                logger.error("no bluepad")
                self.display.show("no bluepad\n")
                self.car.beep()
                time.sleep(1)

                continue
            else:
                if not pad_exit_flag:
                    self.display.show(
                        "press btn control\n 3 pressing record\n 4+2 stop\n 4+v del 30pic\n 4+o del all\n"
                    )
                pad_exit_flag = True

            # print(keys_val)

            # 采集巡航数据
            # 左前
            if keys_val[4] == 1024:
                self.run_flag = True
            else:
                self.run_flag = False

            # 采集目标检测数据
            # 右前
            if keys_val[4] == 2048:
                self.det_flag = True
            else:
                self.det_flag = False

            # 结束记录
            # 1 + 2
            if keys_val[4] == 49152:
                self.close()
            # 删除 3s 巡航数据
            # 左下
            elif keys_val[4] == 4:
                self.del_last3s_cruise()
            # 删除 全部 巡航数据
            # 左中
            elif keys_val[4] == 256:
                self.restart_cruise()
            # 删除 3s 检测数据
            # 右下
            elif keys_val[4] == 64:
                self.del_last3s_det()
            # 删除 全部 检测数据
            # 右中
            elif keys_val[4] == 512:
                self.restart_det()
            
            # 记录开始点
            if keys_val[4] == 2:
                self.cruise_start_point = self.cruise_index
            # 删除开始点到现在的数据
            elif keys_val[4] == 8:
                self.cruise_del_start_2_now()

            if self.run_flag:
                self.car_state[0] = self.state_base[0]
                self.car_state[1] = -1 * self.state_base[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_base[2] * keys_val[2]
            else:
                self.car_state[0] = self.state_start[0] * keys_val[1]
                self.car_state[1] = -1 * self.state_start[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_start[2] * keys_val[2]
            # print(*self.car_state)
            self.car.mecanum_wheel(*self.car_state)
            time.sleep(0.05)

    def image_process(self):
        cruise_name_length = 4
        det_name_length = 4

        while not self.exit_flag:
            image_front = self.cap_front.read()
            image_side = self.cap_side.read()

            if self.run_flag:
                data_dict = dict()

                img_name = (cruise_name_length - len(str(self.cruise_index))) * '0' + str(self.cruise_index) + '.jpg'
                data_dict["img_path"] = img_name
                image_path = os.path.join(self.cruise_dir, img_name)
                cv2.imwrite(image_path, image_front)
                data_dict["state"] = self.car_state.copy()
                self.json_data.append(data_dict)
                # print(img_name)
                logger.info("cruise_image:{}".format(img_name))
                self.cruise_index += 1
                if self.cruise_index % 10 == 0:
                    self.save_json(self.json_data, self.data_json_path)
                if self.cruise_index % 20 == 0:
                    self.display.show("image:{}\n".format(self.cruise_index))
                time.sleep(0.05)

            if self.det_flag:
                img_name = (det_name_length - len(str(self.det_index))) * '0' + str(self.det_index) + '.jpg'
                img_path = os.path.join(self.det_dir, img_name)
                cv2.imwrite(img_path, image_side)
                self.det_imglist.append(img_name)
                logger.info("det_image:{}".format(img_name))
                self.det_index += 1
                if self.det_index % 20 == 0:
                    self.display.show("image:{}\n".format(self.det_index))
                time.sleep(0.05)

            # cv2.imshow("front", image_front)
            # cv2.imshow("side", image_side)

            if cv2.waitKey(1) == 27:
                self.close()

    def del_last3s_cruise(self):
        self.car.beep()
        for i in range(30):
            # 尝试使用 pop() 方法移除最后一个元素
            try:
                data = self.json_data.pop()
                path = os.path.join(self.cruise_dir, data['img_path'])
                os.remove(path)
                self.cruise_index -= 1

            except IndexError:
                # 输出: 列表为空，无法使用 pop() 方法
                logger.info("image data zero now")
                return
        self.display.show("image:{}\n".format(self.cruise_index))
        
    def cruise_del_start_2_now(self):
        self.car.beep()
        del_num = self.cruise_index - self.cruise_start_point - 1
        del_num = del_num if del_num>0 else 0
        for i in range(del_num):
            # 尝试使用 pop() 方法移除最后一个元素
            try:
                data = self.json_data.pop()
                path = os.path.join(self.cruise_dir, data['img_path'])
                os.remove(path)
                self.cruise_index -= 1

            except IndexError:
                # 输出: 列表为空，无法使用 pop() 方法
                logger.info("image data zero now")
                return
        self.display.show("image:{}\n".format(self.cruise_index))

    def restart_cruise(self):
        self.car.beep()
        time.sleep(0.4)
        self.car.beep()

        # 使用rm命令删除*.jpg
        # subprocess.run(["rm", "-rf", self.dir + "/*.jpg"])
        # 删除文件
        subprocess.run(["find", self.cruise_dir, "-name", "*.jpg", "-delete"])
        self.json_data = []
        self.cruise_index = 0
        self.display.show("image:{}\n".format(self.cruise_index))

    def del_last3s_det(self):
        self.car.beep()
        for i in range(30):
            # 尝试使用 pop() 方法移除最后一个元素
            try:
                img_name = self.det_imglist.pop()
                path = os.path.join(self.det_dir, img_name)
                os.remove(path)
                self.det_index -= 1

            except IndexError:
                # 输出: 列表为空，无法使用 pop() 方法
                logger.info("image data zero now")
                return
        self.display.show("image:{}\n".format(self.det_index))

    def restart_det(self):
        self.car.beep()
        time.sleep(0.4)
        self.car.beep()

        # 使用rm命令删除*.jpg
        # subprocess.run(["rm", "-rf", self.dir + "/*.jpg"])
        # 删除文件
        subprocess.run(["find", self.det_dir, "-name", "*.jpg", "-delete"])
        self.det_index = self.det_start_num if self.det_start_num!=0 else 0
        self.display.show("image:{}\n".format(self.det_index))

    # 写一个网页，具有遥控控件的界面
    def controller_html(self):
        # 具体实现
        pass

    @staticmethod
    def save_json(json_data, path):
        with open(path, 'w') as fp:
            json.dump(json_data, fp)

    def get_state(self):
        return self.car_state

    def close(self):
        self.save_json(self.json_data, self.data_json_path)
        self.display.show("control end!\nimage:{}\n".format(self.cruise_index))
        self.exit_flag = True
        self.img_thread.join()
        self.cap_front.close()
        self.cap_side.close()

        for i in range(3):
            self.car.beep()
            time.sleep(0.4)


if __name__ == "__main__":
    remote_car = RemoteControlCar(
        cap_front_id=2, 
        cap_side_id=0, 
        det_start_num=0 
    )

    # while True:
    # time.sleep(0.5)
