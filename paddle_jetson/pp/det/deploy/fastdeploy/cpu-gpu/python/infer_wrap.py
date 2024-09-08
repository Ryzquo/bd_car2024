import cv2
import os

import fastdeploy as fd
import time


def parse_arguments():
    import argparse
    import ast
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model_dir",
        default=None,
        help="Path of PaddleDetection model directory")
    parser.add_argument(
        "--image", default=None, help="Path of test image file.")
    parser.add_argument(
        "--device",
        type=str,
        default='cpu',
        help="Type of inference device, support 'kunlunxin', 'cpu' or 'gpu'.")
    parser.add_argument(
        "--use_trt",
        type=ast.literal_eval,
        default=False,
        help="Wether to use tensorrt.")
    return parser.parse_args()


def build_option(args):
    option = fd.RuntimeOption()

    if args.device.lower() == "kunlunxin":
        option.use_kunlunxin()

    if args.device.lower() == "ascend":
        option.use_ascend()

    if args.device.lower() == "gpu":
        option.use_gpu()

    if args.use_trt:
        option.use_trt_backend()
    return option


model_dir = "ppyoloe_plus_crn_s_80e"
model_dir = "ppyoloe_crn_l_300e_coco"
device = "gpu"
use_trt = False

model_file = os.path.join(model_dir, "model.pdmodel")
params_file = os.path.join(model_dir, "model.pdiparams")
config_file = os.path.join(model_dir, "infer_cfg.yml")

# 配置runtime，加载模型
option = fd.RuntimeOption()
runtime_option = option.use_gpu()
runtime_option = option.use_trt_backend()
model = fd.vision.detection.PPYOLOE(
    model_file, params_file, config_file, runtime_option=runtime_option)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
# cv2.CAP_DSHOW什么含义

last_time = time.time()
while True:
    fps = 1/ (time.time() - last_time)
    print("fps:", fps)
    last_time = time.time()
    ret, im = cap.read()
    if not ret:
        break
    result = model.predict(im)
    # print(result)

    # 预测结果可视化
    # vis_im = fd.vision.vis_detection(im, result, score_threshold=0.5)
    # cv2.imshow("image", vis_im)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    # print("Visualized result save in ./visualized_result.jpg")
