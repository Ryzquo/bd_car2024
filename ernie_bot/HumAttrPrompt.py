#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

# 添加根目录
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from ernie_bot.base.PromptJson import PromptJson


class HumAttrPrompt(PromptJson):
    def __init__(self) -> None:
        rulers = """
        你是一个人特征总结程序, 
        需要根据描述把人的特征生成对应的json结果, 如果有对应的描述就写入对应位置。
        严格按照下面的scheme描述生成给定格式json, 只返回json数据:
        请在最短时间内给出你的回复, 
        如果存在无法理解的内容, 只对可以理解的内容生成结果, 不要返回多余的内容, 
        不需要在返回的结果中添加注释(不要在``````之间有任何注释和多余内容)
        严格按照下面的scheme描述生成给定格式json, 只返回json数据:
        """
        super().__init__(rulers)

    def json_obj(self) -> dict:
        """
        0 = Hat - 帽子:0无1有
        1 = Glasses - 眼镜:0无1有
        2 = ShortSleeve - 短袖
        3 = LongSleeve - 长袖
        4 = UpperStride - 有条纹
        5 = UpperLogo - 印有logo/图案
        6 = UpperPlaid - 撞色衣服(多种颜色)
        7 = UpperSplice - 格子衫
        8 = LowerStripe - 有条纹
        9 = LowerPattern - 印有图像
        10 = LongCoat - 长款大衣
        11 = Trousers - 长裤
        12 = Shorts - 短裤
        13 = Skirt&Dress - 裙子/连衣裙
        14 = boots - 鞋子
        15 = HandBag - 手提包
        16 = ShoulderBag - 单肩包
        17 = Backpack - 背包
        18 = HoldObjectsInFront - 手持物品
        19 = AgeOver60 - 大于60
        20 = Age18-60 - =18~60
        21 = AgeLess18 - 小于18
        22 = Female - 0:男性; 1:女性
        23 = Front - 人体朝前
        24 = Side - 人体朝侧
        25 = Back - 人体朝后
        """
        schema_attr = {
            'type': 'object',
            'properties': {
                'hat': {
                    'type': 'boolean',
                    'description': '戴帽子真,没戴帽子假',
                },
                'glasses': {
                    "type": 'boolean',
                    'description': '戴眼镜真,没戴眼镜假',
                    'threshold': 0.15,
                },
                'sleeve': {
                    'enum': ['Short', 'Long'],
                    'description': '衣袖长短, 没有为空字符串("")',
                },
                # 有条纹 'Stride',
                # 印有logo/图案 'Logo',
                # 撞色衣服(多种颜色) 'Plaid',
                # 格子衫'Splice'
                'color_upper': {
                    'enum': ['Stride', 'Logo', 'Plaid', 'Splice'],
                    'description': '上衣衣服颜色, 没有为空字符串("")',
                },
                # 有条纹, 'Stripe',
                # 印有图像 'Pattern'
                'color_lower': {
                    'enum': ['Stripe', 'Pattern'],
                    'description': '下衣衣服长短, 没有为空字符串("")',
                },
                # 长款大衣 'LongCoat'
                'clothes_upper': {
                    'enum': ['LongCoat'],
                    'description': '上衣衣服类型, 没有为空字符串("")',
                    'threshold': 0.8,
                },
                # 长裤 'Trousers',
                # 短裤 'Shorts',
                # 裙子/连衣裙 'Skirt_dress'
                'clothes_lower': {
                    'enum': ['Trousers', 'Shorts', 'Skirt_dress'],
                    'description': '下衣衣服类型, 没有为空字符串("")',
                },
                'boots': {
                    'type': 'boolean',
                    'description': '穿着鞋子真,没穿鞋子假',
                },
                'bag': {
                    'enum': ['HandBag', 'ShoulderBag', 'Backpack'],
                    'description': '带着包的类型, 没有为空字符串("")',
                },
                'holding': {
                    'type': 'boolean',
                    'description': '持有物品(拿在手上)为真, 如果描述中没有提到持有物品, 手中有物品等, 则返回False, 偷了抢了我的东西并不说明持有(拿在手上)',
                    'threshold': 0.5,
                },
                'old': {
                    'type': 'boolean',
                    'description': '是否为老年人',
                    'threshold': 0.1,
                },
                'middle': {
                    'type': 'boolean',
                    'description': '是否为中年人',
                },
                'young': {
                    'type': 'boolean',
                    'description': '是否为年轻人',
                },
                # 'age': {
                #     'enum': ['Old', 'Middle', 'Young'],
                #     'description': '年龄,小于18岁为young, 18到60为middle, 大于60为old',
                # },
                'sex': {
                    'enum': ['Female', 'Male'],
                    'description': '性别',
                    'threshold': 0.6,
                },
                'direction': {
                    'enum': ['Front', 'Side', 'Back'],
                    'description': '人体朝向',
                },
            },
            "additionalProperties": False
        }
        return schema_attr

    def example(self) -> str:
        example = """
        正确的示例如下：
        一个带着眼镜的老人: ```{'glasses': True, 'old': True}```,
        犯人不是一个中年, 戴着帽子: ```{'hat': True, 'middle': False}```,
        犯人不是一个老年: ```{'old': False}```,
        一个带着帽子的中年人: ```{'hat': True, 'middle': True}``` ,
        穿着短袖的带着眼镜的人: ```{'glasses': True, 'clothes': 'short'}``` ,
        穿着蓝色连衣裙的长发女性，拿着鲜花，穿着高跟鞋: ```{'clothes_lower': 'Skirt_dress', 'holding': True, 'boots': True}``` ,
        那位穿着大衣，戴着眼镜，背着包的人偷了我的东西: ```{'glasses': True, 'clothes_upper': 'LongCoat', 'bag': 'Backpack'}```
        """
        return example


if __name__ == '__main__':
    pass
