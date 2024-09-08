import sys, os
parent_path = os.path.abspath(os.path.join(__file__, ".."))
deploy_path = os.path.abspath(os.path.join(parent_path, "deploy"))
# print(deploy_path)
sys.path.insert(0, deploy_path)
# sys.path.insert(0, parent_path)
from .infer_wrap import MotHuman, YoloeInfer, HumanAttr, OCRReco, LaneInfer