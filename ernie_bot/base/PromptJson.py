#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys


class PromptJson:
    def __init__(self, rulers) -> None:
        self.rulers_str = "请根据下面的scheme描述生成给定格式json,只返回json数据,不要其他内容。"
        self.scheme_str = ""
        self.example_str = ""

        self.set_rulers(rulers)
        self.set_scheme(self.json_obj())
        self.set_example(self.example())

    def json_obj(self):
        return """```{'type':'string'}```"""

    def example(self):
        return "正确的示例如下："

    def __call__(self, *args, **kwargs):
        pass

    def set_scheme(self, json_obj):
        # json转字符串去空格,换行, 制表符
        json_str = str(json_obj).replace(' ', '').replace('\n', '').replace('\t', '')
        # 加上三个引号
        json_str = '```' + json_str + '```'
        self.scheme_str = json_str

    def set_example(self, example_str: str):
        # 去空格,换行, 制表符
        example_str = example_str.replace(' ', '').replace('\n', '').replace('\t', '')
        self.example_str = example_str

    def set_rulers(self, rulers):
        self.rulers_str = rulers.replace(' ', '').replace('\n', '').replace('\t', '')

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return self.rulers_str + self.scheme_str + self.example_str


if __name__ == '__main__':
    pass
