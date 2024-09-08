#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob

from paddle.inference import Config, create_predictor

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from paddle_jetson.pp.seg.paddleseg.deploy.infer import DeployConfig

import ryzquo_tools.path_tools as rpt
from ryzquo_tools.log_tools import logger


class SegInterface:
    def __init__(self, model_dir: str) -> None:
        self.model_dir = model_dir
        
        self.load_cfg()
        
        self.predictor = create_predictor(self.pred_cfg)

    def load_cfg(self):
        """
        加载模型配置
        """
        yml_path = os.path.join(self.model_dir, "deploy.yaml")
        self.cfg = DeployConfig(yml_path)
        
        self._init_base_config()
        self._init_gpu_config()
        
    def _init_base_config(self):
        self.pred_cfg = Config(self.cfg.model, self.cfg.params)
        # 打印飞桨推理信息
        # self.pred_cfg.disable_glog_info()
        # 开启内存优化
        self.pred_cfg.enable_memory_optim()
        # 开启ir优化
        self.pred_cfg.switch_ir_optim(True)
        
    def _init_gpu_config(self):
        """
        Init the config for nvidia gpu.
        """
        logger.info("Use GPU")
        self.pred_cfg.enable_use_gpu(100, 0)

    def __call__(self, *args, **kwds):
        return self.predict(*args, **kwds)

    def predict(self, image, visualize=False):
        pass

    def close(self):
        pass
