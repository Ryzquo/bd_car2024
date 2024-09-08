#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import glob


sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import ryzquo_tools.path_tools as rpt



class InferInterface:
    def __init__(self, model_dir: str):
        self.model_dir = model_dir
        self.attr_predictor = None

    def get_model_path(self):
        """
        获取模型文件与参数文件路径
        Returns:

        """
        model_path = glob.glob(self.model_dir + "/*.pdmodel")[0]
        params_path = glob.glob(self.model_dir + "/*.pdiparams")[0]
        return model_path, params_path

    def get_path_abs(self, path_relative, p_dir=None):
        """
        获取绝对路径
        Args:
            path_relative: 相对路径

        Returns:

        """
        if p_dir is None:
            p_dir = __file__
        
        return rpt.join_path(rpt.get_root(p_dir), path_relative)

    def load_cfg(self):
        """
        加载模型配置
        Returns:

        """
        yml_path = os.path.join(self.model_dir, "infer_cfg.yml")
        with open(yml_path) as f:
            import yaml
            yml_conf = yaml.safe_load(f)
            self.threshold = yml_conf['draw_threshold']
            self.label_list = yml_conf['label_list']

    def __call__(self, *args, **kwds):
        return self.predict(*args, **kwds)

    def predict(self, image, normalize_out=False, visualize=False):
        pass

    def close(self):
        pass
