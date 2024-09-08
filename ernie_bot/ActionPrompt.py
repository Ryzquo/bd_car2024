#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import erniebot

# 添加根目录
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from ernie_bot.base.PromptJson import PromptJson


class ActionPrompt(PromptJson):
    """
    行动提示
    """
    def __init__(self) -> None:
        rulers = """
        你是一个机器人动作规划者, 
        需要把我的话翻译成机器人动作规划并生成对应的json结果, 机器人工作空间参考右手坐标系。
        不要在我的描述的外自己创造新的方法。
        如果描述比较宽泛, 请使用合适的方法, 
        请在最短时间内给出你的回复, 
        如果存在无法理解的内容, 只对可以理解的内容生成结果, 不要返回多余的内容, 
        返回的json数据中不要有"//"开头的注释, 
        严格按照下面的scheme描述生成给定格式json, 只返回json数据, json数据中不要有注释:
        """
        super().__init__(rulers)

        # self.set_rulers(rulers)
        # self.set_scheme(self.json_obj())
        # self.set_example(self.example())

    def json_obj(self) -> dict:
        schema_move = {
            'type': 'object',
            'required': ['func', 'x', 'y', 'angle'],
            'properties': {
                'func': {
                    'description': '移动, 如果只是说行动, 出发, 可以不使用这个方法',
                    'const': 'move'
                },
                'x': {
                    'description': 'x坐标, 前后移动, 向前移动正值(x>0), 向后移动负值(x<0)',
                    'type': 'number'
                },
                'y': {
                    'description': 'y坐标, 左右移动, 向左移动正值(y>0), 向右移动负值(y<0)',
                    'type': 'number'
                },
                'angle': {
                    'description': '旋转或者转弯角度, 左转向左转逆时针转请使用正值(angle>0), 右转向右转顺时针转请使用负值(angle<0)',
                    'type': 'number'
                }
            }
        }
        schema_beep = {
            'type': 'object',
            'required': ['func', 'time_dur'],
            'properties': {
                'func': {
                    'description': '蜂鸣器,需要发声或需要警示时',
                    'const': 'beep'
                },
                'time_dur': {
                    'description': '蜂鸣器发声持续时间, 这个参数不是必需的, 在出现需要鸣笛发声蜂鸣器鸣叫多长时间等描述时才需要这个参数',
                    'type': 'number'
                },
                'count': {
                    'description': '蜂鸣器发声次数, 这个参数不是必需的, 在出现需要鸣笛发声蜂鸣器鸣叫多少次等描述时才需要这个参数',
                    'type': 'number'
                },
                'time_interval': {
                    'description': '蜂鸣器发声间隔时间, 这个参数不是必需的, 在出现需要鸣笛发声蜂鸣器鸣叫多少次等描述时才需要这个参数, 用来指定多次发声之间的间隔时间',
                    'type': 'number'
                }
            }
        }
        schema_light = {
            'type': 'object',
            'required': ['func', 'time_dur'],
            'properties': {
                'func': {
                    'description': '发光, 需要照明或需要警示时',
                    'const': 'light'
                },
                'time_dur': {
                    'description': '照亮持续时间, 这个参数不是必需的, 在出现需要亮灯发光照亮多长时间等描述时才需要这个参数',
                    'type': 'number'
                },
                'count': {
                    'description': '照亮次数, 这个参数不是必需的, 在出现需要亮灯发光照亮多少次等描述时才需要这个参数',
                    'type': 'number'
                },
                'time_interval': {
                    'description': '灯的间隔时间, 这个参数不是必需的, 在出现需要亮灯发光照亮多少次等描述时才需要这个参数, 用来指定多次亮灯之间的间隔时间',
                    'type': 'number'
                }
            }
        }
        schema_wait = {
            'type': 'object',
            'required': ['func', 'time_dur'],
            'properties': {
                'func': {
                    'description': '等待一段时间',
                    'const': 'car_wait'
                },
                'time_dur': {
                    'description': '等待持续时间, 等待的时间',
                    'type': 'number'
                },
            }
        }

        schema_actions = {
            'type': 'object',
            'description': "动作列表",
            'required': ['primary_items', 'secondary_items'],
            'properties': {
                'primary_items': {
                    'type': 'array',
                    'description': "主要动作列表, 对描述中的主要内容进行规划",
                    'required': ['items'],
                    'items': {
                        'anyOf': [schema_move, schema_beep, schema_light, schema_wait],
                        'minItems': 1
                    }
                },
                'secondary_items': {
                    'description': "次要动作列表, 对描述中的次要内容进行规划, 如出现伴有(伴随、同时)xxx, xxx即为次要动作, 将次要动作放在secondary_items中",
                    'required': ['items'],
                    'items': {
                        'anyOf': [schema_move, schema_beep, schema_light, schema_wait],
                        'minItems': 1
                    }
                }
            }
        }
        return schema_actions

    def example(self) -> str:
        example = """
        正确的示例如下: 
        向左移0.1m, 向左转弯85度: ```{'primary_items': [{'func': 'move', 'x': 0, 'y': 0.1, 'angle': 85}], 'secondary_items': []}```,
        向右移0.2米, 向右转弯90度: ```{'primary_items': [{'func': 'move', 'x': 0, 'y': -0.2, 'angle': -90}], 'secondary_items': []}```,
        向右移0.2m, 左移0.3米: ```{'primary_items': [{'func': 'move', 'x': 0, 'y': -0.2, 'angle': 0}, {'func': 'move', 'x': 0, 'y': 0.3, 'angle': 0}], 'secondary_items': []}```,
        后移0.2m, 左移0.3m: ```{'primary_items': [{'func': 'move', 'x': -0.2, 'y': 0, 'angle': 0}, {'func': 'move', 'x': 0, 'y': 0.3, 'angle': 0}], 'secondary_items': []}```,
        向右移0.2m, 向前0.1m: ```{'primary_items': [{'func': 'move', 'x': 0.1, 'y': -0.2, 'angle': 0}], 'secondary_items': []}```,
        向右移0.2米, 向后0.3m: ```{'primary_items': [{'func': 'move', 'x': -0.3, 'y': -0.2, 'angle': 0}], 'secondary_items': []}```,
        往右边行动0.2m, 前进0.5m: ```{'primary_items': [{'func': 'move', 'x': 0.5, 'y': -0.2, 'angle': 0},], 'secondary_items': []}```,
        向右转85度, 向右移0.1米: ```{'primary_items': [{'func': 'move', 'x': 0, 'y': -0.1, 'angle': -85}], 'secondary_items': []}```,
        蜂鸣器发声5秒: ```{'primary_items': [{'func': 'beep', 'time_dur': 5}], 'secondary_items': []}```,
        鸣笛警示3次: ```{'primary_items': [{'func': 'beep', 'count': 3}], 'secondary_items': []}```
        鸣笛警示3次, 间隔3秒: ```{'primary_items': [{'func': 'beep', 'count': 3, 'time_interval': 3}], 'secondary_items': []}```
        亮灯警示3次, 每次3秒, 间隔2秒: ```{'primary_items': [{'func': 'light', 'count': 3, 'time_interval': 3}], 'secondary_items': []}```
        鸣笛警示3次, 间隔3秒, 然后出发: ```{'primary_items': [{'func': 'beep', 'count': 3, 'time_dur': 3, 'time_interval': 2}], 'secondary_items': []}```
        照亮3次: ```{'primary_items': [{'func': 'light', 'count': 3}], 'secondary_items': []}```,
        常亮4秒: ```{'primary_items': [{'func': 'light', 'time_dur': 4}], 'secondary_items': []}```,
        3次照亮5秒, 间隔1秒: ```{'primary_items': [{'func': 'light', 'time_dur': 5, 'count': 3, 'time_interval': 1}], 'secondary_items': []}```,
        照亮4秒3次, 间隔1秒: ```{'primary_items': [{'func': 'light', 'time_dur': 4, 'count': 3, 'time_interval': 1}], 'secondary_items': []}```,
        发光5秒: ```{'primary_items': [{'func': 'light', 'time_dur': 5}], 'secondary_items': []}```。
        不境, 照亮3秒, 毛3次, 之后鸣笛地向右转弯100度: ```{'primary_items': [{'func': 'light', 'time_dur': 3}, {'func': 'beep', 'count': 1}, {'func': 'move', 'x': 0, 'y': 0, 'angle': -100}, {'func': 'move', 'x': 0, 'y': 0, 'angle': 100}], 'secondary_items': []}```, 
        发出灯光闪烁3次, 并伴有蜂鸣器: ```{'primary_items': [{'func': 'light', 'count': 3}], 'secondary_items': [{'func': 'beep'}]}```, 
        发光5次, 并伴有蜂鸣器鸣叫3秒: ```{'primary_items': [{'func': 'light', 'count': 5}], 'secondary_items': [{'func': 'beep', 'time_dur': 3}]}```, 
        蜂鸣器警报5次, 同时灯光长亮4秒: ```{'primary_items': [{'func': 'beep', 'count': 3}], 'secondary_items': [{'func': 'light', 'time_dur': 4}]}```, 
        鸣叫3声, 并伴有发出灯光闪烁3次: ```{'primary_items': [{'func': 'beep', 'count': 3}], 'secondary_items': [{'func': 'light', 'count': 3}]}```, 
        """
        return example


if __name__ == '__main__':
    pass
