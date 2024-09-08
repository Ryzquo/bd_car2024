#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import glob
import time
import os
import sys
import argparse

import numpy as np

from paddle.inference import Config, create_predictor
from typing import List

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from paddle_jetson.pp.det.deploy.pipeline.cfg_utils import ArgsParser
from paddle_jetson.pp.det.deploy.python.infer import Detector
from paddle_jetson.pp.det.deploy.pptracking.python.mot_sde_infer import SDE_Detector
from paddle_jetson.pp.det.deploy.pipeline.pphuman.attr_infer import AttrDetector
from paddle_jetson.pp.det.deploy.pipeline.ppvehicle.vehicle_plate import PlateRecognizer, PlateDetector, TextRecognizer
from paddle_jetson.pp.det.deploy.pipeline.pipe_utils import parse_mot_res

from paddle_jetson.base.DetectResult import DetectResult
from paddle_jetson.base.InferInterface import InferInterface

from paddle_jetson.base.SegInterface import SegInterface

from ernie_bot import HumAttrPrompt
from camera import Camera

import ryzquo_tools.path_tools as rpt
from ryzquo_tools.log_tools import logger


class MotHuman(InferInterface):
    """
    人物追踪
    # 多目标跟踪(MOT): 单目标跟踪(VOT/SOT)、目标检测(detection)、行人重识别(Re-ID)
    # ①检测 ②特征提取、运动预测 ③相似度计算 ④数据关联。
    # SORT作为一个粗略的框架, 核心就是两个算法: 卡尔曼滤波和匈牙利匹配。
    # DeepSORT的优化主要就是基于匈牙利算法里的这个代价矩阵。它在IOU Match之前做了一次额外的级联匹配, 利用了外观特征和马氏距离。
    """
    def __init__(
            self, model_dir=None, run_mode=None, device=None,
            skip_frame_num=None
    ) -> None:
        if model_dir is None:
            model_dir = 'hum_models/mot_ppyoloe_s_36e_pipeline'
        if run_mode is None:
            run_mode = 'paddle'
        if device is None:
            device = 'GPU'
        if skip_frame_num is None:
            skip_frame_num = 2

        # 加载模型文件夹
        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        print(model_dir, run_mode)
        config_path = rpt.join_path(rpt.get_root(__file__), 'pp/det/deploy/pipeline/config/tracker_config.yml')
        # 加载模型
        self.mot_predictor = SDE_Detector(
            model_dir=self.model_dir,
            tracker_config=config_path,
            # run_mode="trt_fp16",
            run_mode=run_mode,
            device=device,
            skip_frame_num=skip_frame_num
        )
        # 2帧检测一次
        self.skip_frame = skip_frame_num
        # 加载模型配置
        super().load_cfg()
        self.frame_id = 0

    def predict(
            self, image,
            normalize_out=False, visualize=False
    ) -> List[DetectResult]:
        """
        检测输入图像中的人物
        Args:
            image: 输入的图像
            normalize_out: 是否对输出结果进行归一化
            visualize: 是否绘制检测结果

        Returns:
            list[DetectResult]:
            检测结果列表，每个结果包含物体类别ID、类别名称、置信度、物体边界框和可选的物体ID。
        """
        # BGR -> RGB
        frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # if self.frame_id > 5 and self.frame_id % self.skip_frame > 0:
        #     reuse_det_result = True
        # else:
        #     reuse_det_result = False
        # 获取预测结果
        res = self.mot_predictor.predict_image(
            [frame_rgb],
            visual=False,
            reuse_det_result=False,
            frame_count=self.frame_id
        )
        # 预测结果再处理, 获取特征id
        res = self.mot_predictor.predict_image(
            [frame_rgb],
            visual=False,
            reuse_det_result=True,
            frame_count=self.frame_id
        )
        # 进行解析
        mot_res = parse_mot_res(res)

        ret = []
        for bbox in mot_res["boxes"]:
            # 解析每个检测到的人物信息
            obj_id, cls_id, score, rect = int(bbox[0]), int(bbox[1]), bbox[2], bbox[3:].astype(np.int32)
            # 初始化结果类
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect, object_id=obj_id)

            # 是否初始化
            if normalize_out:
                res = res.tolist_normalize(image.shape[:2][::-1])

            # 添加结果
            ret.append(res)

        # 是否可视化
        if visualize:
            visualize_res(image, ret)
        # 更新帧
        self.frame_id += 1
        return ret


class YoloeInfer(InferInterface):
    """
    目标检测(方块, 球, 转向卡片)
    """
    def __init__(self, model_dir=None, run_mode=None, device=None) -> None:
        if model_dir is None:
            model_dir = "det_models/task_model3"
        if run_mode is None:
            run_mode = "paddle"
        if device is None:
            device = "GPU"

        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        print(model_dir, run_mode)
        self.yolo_predictor = Detector(
            model_dir=self.model_dir,
            device=device,
            run_mode=run_mode
        )
        super().load_cfg()

    def predict(
            self, image, normalize_out=False, visualize=False
    ) -> List[DetectResult]:
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 推理
        det_res = self.yolo_predictor.predict_image([image_rgb], visual=False)
        # 过滤分数低于阈值的
        det_res = self.yolo_predictor.filter_box(det_res, self.threshold)
        # 返回值处理
        ret = []
        for bbox in det_res["boxes"]:
            cls_id, score, rect = int(bbox[0]), bbox[1], bbox[2:].astype(np.int32)
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect)
            if normalize_out:
                res = res.tolist_normalize(image.shape[:2][::-1])
            ret.append(res)

        if visualize:
            visualize_res(image, ret)

        return ret

    def close(self):
        super().close()


class HumanAttr(InferInterface):
    """
    检测人物属性
    """
    def __init__(self, model_dir=None, run_mode=None, device=None) -> None:
        if model_dir is None:
            # model_dir = "hum_models/PPLCNet_x1_0_person_attribute_945_infer"
            model_dir = "hum_models/PPLCNet_x1_0_person_attribute_infer_car2024"
        if run_mode is None:
            run_mode = "paddle"
        if device is None:
            device = "GPU"

        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        print(model_dir, run_mode)
        self.attr_predictor = AttrDetector(
            model_dir=self.model_dir,
            device=device,
            run_mode=run_mode
        )
        super().load_cfg()

        # 属性配置，和erniebot获取的设置一致
        self.attr_json = HumAttrPrompt().json_obj()['properties']
        # print(self.label_list)

    def predict(
            self, image, normalize_out=False, visualize=False
    ):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # 推理并获取结果, 推理结果里只有 'output'
        scores = self.attr_predictor.predict_image([image_rgb], visual=False)['output'][0]
        return self.to_attr(scores)

    def to_attr(self, score_arr, threshold_attr=None):
        """
        见 erniebot 的 prompt
        """
        if threshold_attr is None:
            threshold_attr = 0.5

        # 属性配置字典
        dict_attr = self.attr_json

        # 指定必须的属性
        require_arr = {
            'hat': True, 
            'glasses': True, 
            'sleeve': True, 
            'color_upper': True, 
            'color_lower': True,
            'clothes_upper': True, 
            'clothes_lower': True, 
            'boots': True, 
            'bag': True, 
            'holding': True,
            'old': True,
            'middle': True,
            'young': True,
            # 'age': True,
            'sex': True, 
            'direction': True
        }

        # score_arr = np.arange(0.9, 0, -0.1)
        index_s = 0
        index_e = 0
        ret_dict = {}
        for key, val in dict_attr.items():
            # print(f"key: {key}, val: {val}")
            # 获取阈值
            threshold = val['threshold'] if 'threshold' in val else threshold_attr

            # 处理性别
            if key == 'sex':
                index_e = index_s + 1
                # 如果分数大于阈值是女性, 否则是男性
                ret_dict[key] = val['enum'][0] \
                    if score_arr[index_s] > threshold \
                    else val['enum'][1]
            # 处理年龄
            elif key == 'old':
                # 年龄不由这里处理, 交由年龄处理器
                index_e = index_s + 3
                max_idx = np.argmax(np.array(score_arr[index_s:index_e]))
                # print(f"age: {score_arr[index_s:index_e]}, idx: {max_idx}")
                if max_idx == 0:
                    ret_dict['old'] = True
                    ret_dict['middle'] = False
                    ret_dict['young'] = False
                elif max_idx == 1:
                    ret_dict['middle'] = True
                    ret_dict['young'] = False
                    ret_dict['old'] = False
                elif max_idx == 2:
                    ret_dict['young'] = True
                    ret_dict['old'] = False
                    ret_dict['middle'] = False
                # ret_dict['young'] = False
                # ret_dict['old'] = False
                # ret_dict['middle'] = False
            elif key == 'middle':
                continue
            elif key == 'young':
                continue
            # 值里有 'type' 且类型为 boolean, 进行处理
            elif 'type' in val and val['type'] == 'boolean':
                index_e = index_s + 1
                # 分数大于阈值是True, 否则是False
                ret_dict[key] = bool(score_arr[index_s] > threshold)
            # 值里有 'enum', 进行处理
            elif 'enum' in val:
                # 当前位置加上枚举值的长度
                index_e = index_s + len(val['enum'])
                # 获取分数最高的索引
                args_index = np.argmax(np.array(score_arr[index_s:index_e]))
                # 如果是必须的属性, 根据args_index索引取枚举值
                if require_arr[key]:
                    ret_dict[key] = val['enum'][args_index]
                # 否则如果分数大于阈值, 根据args_index索引取枚举值
                elif score_arr[index_s] > threshold:
                    ret_dict[key] = val['enum'][args_index]

            # 把起始索引更新成结尾索引的值
            index_s = index_e
        
        return ret_dict

    def close(self):
        super().close()


class OCRReco(InferInterface):
    def __init__(self, model_det_dir=None, model_rec_dir=None, run_mode=None, device=None,
                 rec_word_dict_path=None) -> None:
        if model_det_dir is None:
            model_det_dir = "ocr_models/ch_PP-OCRv3_det_infer"
        if model_rec_dir is None:
            model_rec_dir = "ocr_models/ch_PP-OCRv3_rec_infer"
        if run_mode is None:
            run_mode = 'paddle'
        if device is None:
            device = "GPU"
        if rec_word_dict_path is None:
            rec_word_dict_path = "pp/det/deploy/pipeline/ppvehicle/rec_word_dict.txt"

        parser = argparse.ArgumentParser()
        parser.add_argument('--device', default=device, help='foo help')
        parser.add_argument('--run_mode', default=run_mode, help='foo help')
        args = parser.parse_args()
        use_gpu = True

        cfg = {
            "det_model_dir": self.get_path_abs(model_det_dir, p_dir=__file__),
            "rec_model_dir": self.get_path_abs(model_rec_dir, p_dir=__file__),
            "det_limit_side_len": 736,
            "det_limit_type": "min",
            "rec_image_shape": [3, 48, 320],
            "rec_batch_num": 6,
            "word_dict_path": self.get_path_abs(rec_word_dict_path, p_dir=__file__)
        }

        args.run_mode = run_mode

        # 文本检测器
        self.plate_detector = PlateDetector(args, cfg=cfg)
        # 文本识别器
        self.text_recognizer = TextRecognizer(args, cfg, use_gpu=use_gpu)
        # 最后过滤用到的阈值
        self.threshold = 0.5
        # super().load_cfg()

    def predict(self, image, normalize_out=False, visualize=False):
        # BGR -> RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 检测图像中的文本, 获得检测到的所有文本信息, 检测耗时
        plate_boxes, det_time = self.plate_detector.predict_image([image_rgb])
        plate_boxes = plate_boxes[0]

        # 存放所有文本图像
        img_text_list = [get_rotate_crop_image(image_rgb, box) for box in plate_boxes]

        text_res = ""
        if len(img_text_list) > 0:
            h_max = max([i.shape[0] for i in img_text_list])

            new_img_text = cv2.hconcat([cv2.resize(img, (img.shape[1], h_max)) for img in img_text_list[::-1]])
            # cv2.imshow("img_text", new_img_text)

            # 文本识别, 获取文本内容和识别耗时
            plate_texts, rec_time = self.text_recognizer.predict_text([new_img_text])
            # 提取文本和置信度
            plate_texts = list(plate_texts[0])

            if plate_texts[1] > self.threshold:
                text_res = plate_texts[0]

        return text_res
    
    
class LaneInferBase(InferInterface):
    def __init__(self, model_dir=None) -> None:
        if model_dir is None:
            model_dir = "lane_models/cruise_cnn"
        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        model_path, params_path = self.get_model_path()
        self.config = Config(model_path, params_path)

        self.config.enable_use_gpu(100, 0)
        self.config.switch_ir_optim()
        self.config.enable_memory_optim()
        self.predictor = create_predictor(self.config)

        self.img_size = (128, 128)
        # self.img_size = (256, 128)
        self.mean = np.array([1.0, 1.0, 1.0])

        self.std = None

    # 归一化处理
    def normalize(self, img):
        img = img.astype(np.float32) / 127.5
        if self.mean is not None:
            img -= self.mean
        if self.std is not None:
            img /= self.std
        # img = (img - self.mean) / self.std
        # 为什么上面的前两个可以，后面的那个不行, 
        # 下面这个是操作后赋值了新的变量，前面的是变量没有更改
        return img

    def preprocess(self, img):
        # 更改分辨率
        img = cv2.resize(img, self.img_size)
        img = self.normalize(img)
        # img = self.resize(img, self.img_size)
        # bgr-> rgb
        img = img[:, :, ::-1].astype('float32')
        # hwc -> chw
        img = img.transpose((2, 0, 1))
        return img[np.newaxis, :]
    
    def extract_lane(self, img_ori):
        """
        提取图像中的车道线
        :param img_ori:
        :return:
        """
        ih, iw = img_ori.shape[:2]
        img = img_ori.copy()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lane_mask = cv2.inRange(
            hsv,
            (14, 31, 127),
            (38, 255, 255)
        )

        # 获取掩码
        img_lane = cv2.bitwise_and(img, img, mask=lane_mask)

        return img_lane
    
    def predict(self, img, normalize_out=False):
        # 提取车道线
        img = self.extract_lane(img)
        # copy img data to input tensor
        img = self.preprocess(img)
        input_names = self.predictor.get_input_names()

        input_tensor = self.predictor.get_input_handle(input_names[0])
        # input_tensor.reshape(img.shape)
        input_tensor.copy_from_cpu(img)
        # do the inference
        self.predictor.run()
        # get out data from output tensor
        output_names = self.predictor.get_output_names()

        output_tensor = self.predictor.get_output_handle(output_names[0])
        output_data = output_tensor.copy_to_cpu()[0]
        if normalize_out:
            output_data = output_data.tolist()
        return output_data


class LaneSeg(SegInterface):
    """
    道路线分割
    """
    def __init__(self, model_dir=None):
        if model_dir is None:
            model_dir = "lane_models/lane_seg"
        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        
        # 大小
        self.img_size = (256, 128)
        
        self.input_names = self.predictor.get_input_names()
        self.input_handle = self.predictor.get_input_handle(self.input_names[0])
        self.output_names = self.predictor.get_output_names()
        self.output_handle = self.predictor.get_output_handle(self.output_names[0])
        
    def _preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.img_size)
        # 将uint8转换为float32，并归一化到0-1范围
        img = img.astype(np.float32) / 255.0
        # HWC to CHW
        img = img.transpose((2, 0, 1))
        return img[np.newaxis, ...]
    
    def _postprocess(self, results):
        # results = np.argmax(results, axis=1)
        return results
    
    def predict(self, image, is_show=False):
        ih, iw = image.shape[:2]
        
        img = self._preprocess(image)
        
        self.input_handle.reshape(img.shape)
        self.input_handle.copy_from_cpu(img)

        self.predictor.run()
        pred = self.output_handle.copy_to_cpu()

        # pred = self._postprocess(pred)
        
        # 二值化
        result_img = (pred[0] == 1).astype(np.uint8) * 255
        
        # 将二值掩码调整回原始图像大小
        result_img = cv2.resize(
            result_img, 
            (iw, ih), 
            interpolation=cv2.INTER_NEAREST
        )
        
        if is_show:
            # 结果叠加到原图
            show_img = cv2.addWeighted(
                image, 0.5, 
                cv2.cvtColor(result_img, cv2.COLOR_GRAY2BGR), 0.5, 
                0
            )
            cv2.imshow("show", show_img)

        return result_img


class LaneInfer(LaneInferBase):
    """
    全程巡航
    """
    def __init__(
        self, 
        model_dir=None,
        seg_model_dir=None
    ):
        if model_dir is None:
            model_dir = "lane_models/cruise_cnn"
        if seg_model_dir is None:
            seg_model_dir = "lane_models/lane_seg"
            
        # 加载分割模型
        self.lane_seg = LaneSeg(
            model_dir=seg_model_dir
        )
        
        super().__init__(rpt.join_path(rpt.get_root(__file__), model_dir))
        
    def extract_lane(self, img):
        """
        提取车道线
        """
        bin_lane_img = self.lane_seg(img)
        
        bgr_lane_img = cv2.cvtColor(bin_lane_img, cv2.COLOR_GRAY2BGR)
        
        return bgr_lane_img


def visualize_res(image, res):
    """
    可视化, 在图像上绘制检测结果
    Args:
        image: 图像
        res: 检测结果

    Returns:

    """
    for obj_box in res:
        obj_id, cls, label, score, bbox = \
            obj_box.object_id, obj_box.class_id, obj_box.label_name, obj_box.score, obj_box.bbox
        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        cv2.putText(image, label, (int(bbox[0]) + 10, int(bbox[1]) + 15),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        cv2.putText(image, str(obj_id), (int(bbox[0]) + 10, int(bbox[1]) + 35),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        cv2.putText(image, "{:.2f}".format(score), (int(bbox[0]) + 10, int(bbox[1]) + 55),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)


def get_rotate_crop_image(img, points):
    """
    从图像中按照指定的四个点进行变换
    Args:
        img: 待处理的图像
        points: 裁剪区域四个顶点的坐标
    Returns:
        变换后的图像
    img_height, img_width = img.shape[0:2]
    left = int(np.min(points[:, 0]))
    right = int(np.max(points[:, 0]))
    top = int(np.min(points[:, 1]))
    bottom = int(np.max(points[:, 1]))
    img_crop = img[top:bottom, left:right, :].copy()
    points[:, 0] = points[:, 0] - left
    points[:, 1] = points[:, 1] - top
    """
    assert len(points) == 4, "shape of points must be 4*2"
    img_crop_width = int(max(
        np.linalg.norm(points[0] - points[1]),
        np.linalg.norm(points[2] - points[3])
    ))
    img_crop_height = int(max(
        np.linalg.norm(points[0] - points[3]),
        np.linalg.norm(points[1] - points[2])
    ))
    pts_std = np.float32([
        [0, 0], [img_crop_width, 0],
        [img_crop_width, img_crop_height],
        [0, img_crop_height]
    ])
    M = cv2.getPerspectiveTransform(points, pts_std)
    dst_img = cv2.warpPerspective(
        img,
        M, (img_crop_width, img_crop_height),
        borderMode=cv2.BORDER_REPLICATE,
        flags=cv2.INTER_CUBIC
    )
    dst_img_height, dst_img_width = dst_img.shape[0:2]
    if dst_img_height * 1.0 / dst_img_width >= 1.5:
        dst_img = np.rot90(dst_img)
    return dst_img


def cam_infer_test():
    cap_front = Camera(2)

    print("model loading...")
    infer = LaneSeg(
        model_dir="lane_models/lane_seg"
    )
    
    print("start...")
    while True:
        img = cap_front.read()
        if cap_front.stop_flag:
            break
        
        st = time.time()
        result_img = infer(img)
        print(f"推理耗时: {time.time() - st} s")
        
        cv2.imshow("img", img)
        cv2.imshow("result", result_img)
        if cv2.waitKey(1) == 27:
            break
    
    print("end...")
    cap_front.close()
    cv2.destroyAllWindows()
    
    
def cam_infer_test_win():
    cap = cv2.VideoCapture(0)
    
    # infer = LaneSeg(
    #     model_dir="lane_models/lane_seg"
    # )
    infer = LaneInfer(
        model_dir="lane_models/cruise_cnn"
    )
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        st = time.time()
        result_img = infer(frame)
        print(f"推理耗时: {time.time() - st} s")
        
        cv2.imshow("frame", frame)
        cv2.imshow("result", result_img)
        if cv2.waitKey(1) == 27:
            break
    
    cap.release()
    cv2.destroyAllWindows()


def img_dir_infer_test():
    # img_dir = r"E:\Dev\baidu19\offline\dataset\seg_lane\lane_ds\images"
    img_dir = r"E:\Dev\baidu19\offline\vehicle_wbt_car2024\demo_data"
    
    img_list = rpt.get_filepaths_without_extension(img_dir, file_format="png")
    
    # infer = LaneSeg(
    #     model_dir="lane_models/lane_seg"
    # )
    infer = YoloeInfer()
    
    for img_path in img_list:
        from infer_cs import Bbox
        img = cv2.imread(img_path)
        
        st = time.time()
        result = infer(img, True)
        print(f"result: {result}")
        print(f"推理耗时: {time.time() - st} s")
        # cv2.putText(img, f"y: {result[0]:.5f}, angle: {result[1]:.5f}", 
        #             (10, 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        for r in result:
            det_bbox = r[4:]
            rect = Bbox(box=det_bbox, size=img.shape[:2][::-1]).get_rect()
            # 绘制 bbox
            cv2.rectangle(
                img=img,
                pt1=rect[:2], pt2=rect[2:],
                color=(0, 255, 0), thickness=2
            )
            cv2.putText(
                img=img,
                text=f"{r[2]}", 
                org=(rect[0], rect[1]-5),
                fontFace=cv2.FONT_HERSHEY_PLAIN,
                fontScale=1,
                color=(255, 0, 0) if 'blue' in r[2] else (0, 255, 255),
                thickness=1
            )
        
        cv2.imshow("img", img)
        # cv2.imshow("result", result_img)
        if cv2.waitKey(0) == 27:
            break
        
    cv2.destroyAllWindows()



if __name__ == '__main__':
    # cam_infer_test()
    # cam_infer_test_win()
    img_dir_infer_test()
    # print(test)
