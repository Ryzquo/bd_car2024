#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import psutil
import subprocess

sys.path.append(os.path.dirname(os.path.realpath(__file__)))

from ryzquo_tools.log_tools import logger


def check_back_python(file_name):
    dir_file = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_file, file_name)
    print(file_path)
    if not os.path.exists(file_path):
        raise Exception("后台脚本文件不存在")
    # 获取正在运行的python脚本
    py_lists = get_python_processes()
    for py_iter in py_lists:
        # 检测是否存在后台运行的脚本
        if file_name in py_iter[1]:
            return
    else:
        # 开启后台脚本，后台运行, 忽略输入输出
        # 使用subprocess调用脚本
        logger.info("开启{}脚本, 后台运行中, 请等待".format(file_name))
        # TODO: 在linux上运行时需要改成 python3
        # cmd_str = 'C:\\develop\\Anaconda3\\envs\\pp\\python.exe ' + file_path + ' &'
        cmd_str = 'python3 ' + file_path + ' &'
        # shell=True告诉subprocess模块在运行命令时使用系统的默认shell。这使得可以像在命令行中一样执行命令，包括使用通配符和其他shell特性
        subprocess.Popen(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1)
        # 这里的> /dev/null 2>&1将标准输出和标准错误都重定向到/dev/null，实现与之前subprocess.Popen相同的效果
        # os.system(cmd_str + " > /dev/null 2>&1")
            
def get_python_processes():
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
        
    # for process in python_processes:
    #     print(process)
    # print("    ")
    
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
        
        
def start(py_name):
    python_processes = get_python_processes()
    for process in python_processes:
        if "car_start_back_as.py" in process[1] and os.getpid() != process[0]:
            logger.info("程序已启动...")
            exit()
    
    while True:
        python_processes = get_python_processes()
        have_car = False
        for process in python_processes:
            if py_name in process[1]:
                have_car = True
        
        if not have_car:
            logger.info(f"启动{py_name}...")
            check_back_python(py_name)
        else:
            logger.info(f"{py_name}正在运行中...")
            
        time.sleep(2)


if __name__ == "__main__":
    import argparse
    # 后台运行
    # nohup python3 ~/bd19/vehicle_wbt/car_start_back_as.py --op start > output.log 2>&1 < /dev/null &
    # 清理运行的程序
    # python3 ~/bd19/vehicle_wbt/car_start_back_as.py --op stop

    py_filename = "car_start.py"

    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="start")
    args = args.parse_args()
    print(args)
    if args.op == "start":
        start(py_name=py_filename)
    if args.op == "stop":
        stop_process("infer_back_end.py")
        stop_process(py_filename)
        stop_process("car_start_back_as.py")
