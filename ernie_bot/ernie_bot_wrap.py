#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import erniebot

# 添加根目录
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from ernie_bot.ActionPrompt import ActionPrompt
from ernie_bot.HumAttrPrompt import HumAttrPrompt


class ErnieBotWrap():
    def __init__(self, api_type=None, access_token=None, model=None):
        """
        Args:
            api_type:
            access_token:
            model: [
                'ernie-3.5', 'ernie-4.0', 'ernie-turbo',
                'ernie-text-embedding', 'ernie-vilg-v2'
            ]
        """
        if api_type is None:
            api_type = 'aistudio'
        if access_token is None:
            access_token = ''
        if model is None:
            model = 'ernie-3.5'

        erniebot.api_type = api_type
        erniebot.access_token = access_token
        self.msgs = []
        self.model = model
        self.prompt_str = '请根据下面的描述生成给定格式json'

    @staticmethod
    def get_message(role, dialog):
        data = {}
        if role == 0:
            data['role'] = 'user'
        elif role == 1:
            data['role'] = 'assistant'
        data['content'] = dialog
        return data

    def set_prompt(self, prompt_str):
        """
        设置提示信息
        Args:
            prompt_str: 要设置的提示信息

        Returns:

        """
        # str_input = prompt_str
        # self.msgs.append(self.get_mes(0, str_input))
        # response = erniebot.ChatCompletion.create(model=self.model, messages=self.msgs, system=prompt_str)
        # str_res = response.get_result()
        # self.msgs.append(self.get_mes(1, str_res))
        # print(str_res)
        # print("设置成功")
        self.prompt_str = prompt_str

    # print(self.prompt_str)

    def get_result(self, str_input, record=False, request_timeout=5):
        """
        根据输入的文本调用大模型生成结果
        Args:
            str_input: 输入的文本字符串, 用于生成结果的描述
            record: 是否记录对话历史。如果为True, 则会将此次的输入和输出记录到历史消息中
            request_timeout: 请求超时时间

        Returns:
            返回一个元组, 
            第一个元素是布尔值, 表示请求是否成功;
            第二个元素是生成的结果字符串, 如果请求失败则为None
        """
        # 描述过短则返回false
        if len(str_input) < 1:
            return False, None

        # 拼接描述
        start_str = " ```"
        end_str = " ```, 根据这段描述生成给定格式json"
        str_input = start_str + str_input + end_str
        # 生成描述
        msg_tmp = self.get_message(0, str_input)

        # 记录对话历史
        if record:
            self.msgs.append(msg_tmp)
            msgs = self.msgs
        else:
            msgs = [msg_tmp]
        # Create a chat completion

        # 调用大模型生成结果
        try:
            response = erniebot.ChatCompletion.create(
                model=self.model, messages=msgs,
                system=self.prompt_str,
                top_p=0,
                _config_=dict(api_type="aistudio", ),
                request_timeout=request_timeout
            )
        except Exception as e:
            print(e)
            return False, None
        # _config_=dict(api_type="QIANFAN",)
        # _config_=dict(api_type="AISTUDIO",)
        # print(response)

        # 获取结果
        str_res = response.get_result()
        # 记录对话历史
        if record:
            self.msgs.append(self.get_message(1, str_res))

        return True, str_res

    @staticmethod
    def get_json_str(json_str: str):
        """
        从包含JSON数据的字符串中提取并解析JSON部分
        Args:
            json_str: 包含JSON数据的原始字符串
                被```json和```包围

        Returns:
            解析后的JSON对象
        """
        try:
            # 查找json字符串开始索引
            index_s = json_str.find("```json") + 7
        except Exception as e:
            # 找不到就从0开始
            index_s = 0

        try:
            # 从开始位置起查找字符串结束位置
            index_e = json_str[index_s:].find("```") + index_s
        except Exception as e:
            # 找不到就到最后一个字符
            index_e = len(json_str)

        import json
        # 解析json字符串
        msg_json = json.loads(json_str[index_s:index_e])
        return msg_json

    def get_res_json(self, str_input, record=False, request_timeout=10):
        # 获取大模型生成结果
        state, str_res = self.get_result(str_input, record, request_timeout)
        print(f"ernire - str_res: {str_res}")

        # 生成成功就解析
        if state:
            # 解析json字符串
            try:
                obj_json = self.get_json_str(str_res)
            except Exception as e:
                print(f"JSON decode error: {e}")
                print(f"Failed JSON string: {str_res}")
                obj_json = None
            return obj_json
        else:
            return None


if __name__ == "__main__":
    action_bot = ErnieBotWrap()
    action_bot.set_prompt(str(ActionPrompt()))
    # text = "前面有动物，需要鸣警示3次，之后常亮3向右移0.1米，在向左格0.1米"
    # text = "前面有动物，需要鸣笛警示3次，之后常亮3秒并向右移0.1米，在向左格0.米"
    # text = "车子故障，需紧急停车，请鸣叫3声，并伴有发出灯光闪烁3次。"
    # text = "车子故障，需紧急停车，请发出灯光闪烁3次，并伴有蜂鸣器。"
    # text = "等待3秒，叫3声，亮3秒"
    # text = "此处危险，需要前移0.1m,然后返回。"
    text = "后移0.2m, 右移0.1m, 左移0.3m, 右转90度, 伴有灯光长亮5秒"
    print(f"text: {text}")
    print(action_bot.get_res_json(
        text, 
        request_timeout=20
    ))
    
    # humattr_bot = ErnieBotWrap()
    # humattr_bot.set_prompt(str(HumAttrPrompt()))
    # # text = "犯人是背对我的中年人"
    # text = "犯人不是一个老年人，戴着眼镜。"
    # # text = "犯人不是一个男人，戴着眼镜。"
    # # text = "刚才有个带着帽子背着包的女性抢了我的东西。"
    # print(f"text: {text}")
    # print(f"attr: {humattr_bot.get_res_json(text, request_timeout=20)}")
