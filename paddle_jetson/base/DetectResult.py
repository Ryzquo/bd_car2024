#! /usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np



class DetectResult:
    def __init__(self, category_id, label, score, bbox, object_id=0) -> None:
        self.class_id = int(category_id)
        self.object_id = int(object_id)
        self.label_name = label
        self.score = float(score)  # score
        self.bbox = bbox.astype(np.int32)

        # 当前bbox中心
        self.center = [self.bbox[0] + self.bbox[2] / 2, self.bbox[1] + self.bbox[3] / 2]
        # 整个图片中心
        self.middle = [320, 240]

    def pos_from_center(self):
        """
        计算当前目标与图像中心点的偏差
        Returns:
            [x轴的偏移量, y轴的偏移量]
        """
        error_x = self.center[0] - self.middle[0]
        error_y = self.center[1] - self.middle[1]
        return [error_x, error_y]

    def get_pos(self):
        """
        获取当前目标与图像中心点的偏差
        Returns:

        """
        error_x = (self.bbox[0] + self.bbox[2]) / 2 - self.middle[0]
        error_y = (self.bbox[1] + self.bbox[3]) / 2 - self.middle[1]
        return [error_x, error_y]

    def tolist(self):
        """
        将当前目标信息转化为列表
        Returns:

        """
        return [self.class_id, self.object_id, self.label_name, self.score] + self.bbox.tolist()

    def tolist_normalize(self, size):
        """
        将当前目标信息归一化后转换为列表
        Args:
            size: 图像大小

        Returns:

        """
        mid_x = size[0] / 2
        mid_y = size[1] / 2
        pt_mid = [mid_x, mid_y]
        # 归一化中心值
        normalized_x = float((self.bbox[0] + self.bbox[2]) / 2 - pt_mid[0]) / pt_mid[0]
        normalized_y = float((self.bbox[1] + self.bbox[3]) / 2 - pt_mid[1]) / pt_mid[1]
        normalized_w = float(self.bbox[2] - self.bbox[0]) / pt_mid[0]
        normalized_h = float(self.bbox[3] - self.bbox[1]) / pt_mid[1]
        return [self.class_id, self.object_id, self.label_name, self.score] + [normalized_x, normalized_y, normalized_w,
                                                                               normalized_h]

    def pos_from_pos(self, pos):
        """
        计算当前目标与指定位置的偏差
        Args:
            pos: 指定位置

        Returns:

        """
        error_x = self.center[0] - pos[0]
        error_y = self.center[1] - pos[1]
        return [error_x, error_y]

    def __repr__(self) -> str:
        return self.__str__()

    def __str__(self) -> str:
        return "cls_id:{} obj_id:{} label:{} score:{:.3f} bbox:{}".format(
            self.class_id, self.object_id, self.label_name, self.score, self.bbox
        )
