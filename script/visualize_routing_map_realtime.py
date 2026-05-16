#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
import traceback
import threading
import queue
import uuid
from typing import Dict, Iterable, List, Optional, Tuple, Set

import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端以支持实时更新
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, Polygon, Circle, Rectangle
from matplotlib.text import Annotation

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from hric_msgs.msg import ObjectArray
from hric_msgs.msg import PlannedTrajectory
from visualization_msgs.msg import MarkerArray, Marker
from bodyctrl_msgs.msg import MotorStatusMsg1, PowerBatteryStatus, RtkGps
from std_msgs.msg import String


# 远程服务快捷键：./> 加速，,/< 减速（步长由 --speed-delta 运行时指定）
REMOTE_SERVICE_NAME = "/hric/nav/adjust_nav_speed"
REMOTE_SERVICE_TYPE = "hric_msgs/srv/AdjustNavSpeed"
REMOTE_SPEED_DEFAULT_DELTA = 0.5
REMOTE_SPEED_TASK_ID = "test_task"
REMOTE_SPEED_UP_KEYS = {'.', '>'}
REMOTE_SPEED_DOWN_KEYS = {',', '<'}

# 远程服务：发送导航目标点（其余字段与 docs/malasong.sh 保持一致；x/y 由 CLI 指定）
START_NAV_SERVICE_NAME = "/hric/nav/start_nav"
START_NAV_SERVICE_TYPE = "hric_msgs/srv/StartNav"
START_NAV_KEY = "ctrl+g"  # mnemonic: Go
START_NAV_DEFAULT_X = 453000.47
START_NAV_DEFAULT_Y = 4404732.38


def build_start_nav_payload(x: float, y: float) -> dict:
    """按给定目标 x/y 构造 StartNav 请求 payload；其它字段与 malasong.sh 一致。"""
    return {
        'task_id': 'test_task',
        'task_type': 1,
        'xy_goal_tolerance': 0.1,
        'yaw_goal_tolerance': 0.1,
        'swing_arm': False,
        'poses': [
            {
                'header': {
                    'stamp': {'sec': 0, 'nanosec': 0},
                    'frame_id': 'map',
                },
                'pose': {
                    'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
            },
        ],
    }


# 为中文环境准备字体，避免中文标题/标签乱码
matplotlib.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC',
    'SimHei',
    'WenQuanYi Zen Hei',
    'Source Han Sans SC',
    'Arial Unicode MS',
    'sans-serif',
]
matplotlib.rcParams['axes.unicode_minus'] = False


Point2D = Tuple[float, float]
LaneSamples = Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]

# 状态面板：腿/臂关节温度行拆为多段——「左*」标签绿色、「右*」标签蓝色，冒号及之后为黑色
_OVERLAY_NORMAL_COLOR = '#1a1a1a'
_LABEL_GREEN = '#2ca02c'
_LABEL_BLUE = '#1f77b4'
_OVERLAY_SUFFIX_BLACK = '#000000'

# 右上角状态面板样式参数
STATUS_PANEL_FONT_SIZE = 10  # 速度/电量/温度的字号（原逻辑是 8）

# 温度阈值（原先你提到 130/90，这里改为 120/80）
TEMP_MOTOR_TEMP_HIGH = 120.0  # motortemperature
TEMP_MOS_TEMP_HIGH = 80.0     # mostemperature
_TEMP_HIGH_COLOR = '#d62728'   # 高于阈值时温度数值变红
_TEMP_OK_COLOR = _OVERLAY_SUFFIX_BLACK


class LaneNode:
    """存储单条车道的中心线与元信息。"""

    def __init__(self) -> None:
        self.lane_id: Optional[str] = None
        self.road_id: Optional[str] = None
        self.is_virtual: Optional[bool] = None
        self.length_m: Optional[float] = None  # 顶层 length
        self.cost: Optional[float] = None  # 顶层 cost
        self.points: List[Point2D] = []  # central_curve.segment.line_segment.point 序列


class SupplyStation:
    """存储一个补给站的入口线与出口线。"""

    def __init__(self) -> None:
        self.station_id: Optional[str] = None
        self.entry_start: Optional[Point2D] = None
        self.entry_end: Optional[Point2D] = None
        self.exit_start: Optional[Point2D] = None
        self.exit_end: Optional[Point2D] = None


class RaceLine:
    """存储一条赛道线（起跑线 / 终点线）。"""

    def __init__(self) -> None:
        self.line_id: Optional[str] = None
        self.line_type: Optional[str] = None  # RACE_START / RACE_FINISH
        self.start_point: Optional[Point2D] = None
        self.end_point: Optional[Point2D] = None


def _parse_float(token: str) -> Optional[float]:
    try:
        return float(token)
    except Exception:
        return None


def _color_from_string(key: Optional[str]) -> Tuple[float, float, float]:
    """根据字符串生成稳定的颜色（HSV->RGB）。"""
    if not key:
        return (0.4, 0.4, 0.4)
    h = (hash(key) % 360) / 360.0
    s = 0.65
    v = 0.85
    # hsv to rgb
    i = int(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    i = i % 6
    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return (r, g, b)


def parse_routing_map(
    file_path: str,
    bbox: Optional[Tuple[float, float, float, float]] = None,
    max_nodes: Optional[int] = None,
) -> Tuple[Dict[str, LaneNode], List[Tuple[str, str]], List[Tuple[str, str, str]]]:
    """
    解析 Apollo routing_map 文本，提取 node 的中心线点列与 edge 拓扑。

    bbox: (min_x, min_y, max_x, max_y)，仅保留中心线有点落入 bbox 的 node 与相关 edge。
    max_nodes: 限制最多解析保存的 node 数量（按出现顺序）。
    """
    nodes: Dict[str, LaneNode] = {}
    edges: List[Tuple[str, str]] = []
    edges_typed: List[Tuple[str, str, str]] = []

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"未找到文件: {file_path}")

    def point_in_bbox(pt: Point2D) -> bool:
        if bbox is None:
            return True
        min_x, min_y, max_x, max_y = bbox
        x, y = pt
        return (min_x <= x <= max_x) and (min_y <= y <= max_y)

    # 状态机变量
    in_node = False
    in_edge = False
    node_brace_depth = 0
    edge_brace_depth = 0
    current_node: Optional[LaneNode] = None
    top_length_captured = False
    top_cost_captured = False

    # line_segment 内局部状态
    collecting_line_segment_points = False
    line_segment_depth = 0
    pending_x: Optional[float] = None

    # 暂存全部 edge，等完成 nodes 过滤后再筛 edge
    all_edges: List[Tuple[str, str]] = []
    tmp_from: Optional[str] = None
    tmp_to: Optional[str] = None
    tmp_dir: Optional[str] = None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw_line in f:
            line = raw_line.strip()

            # 进入/退出 node 块
            if not in_node and line.startswith('node {'):
                in_node = True
                node_brace_depth = 1
                current_node = LaneNode()
                top_length_captured = False
                top_cost_captured = False
                # 重置几何状态
                collecting_line_segment_points = False
                line_segment_depth = 0
                pending_x = None
                continue

            if not in_edge and line.startswith('edge {'):
                in_edge = True
                edge_brace_depth = 1
                tmp_from = None
                tmp_to = None
                tmp_dir = None
                continue

            # node 内容解析
            if in_node and current_node is not None:
                # 花括号计数（node 作用域）
                if '{' in line:
                    node_brace_depth += line.count('{')
                if '}' in line:
                    node_brace_depth -= line.count('}')

                # 顶层字段（lane_id/road_id/is_virtual/length/cost）
                if current_node.lane_id is None and line.startswith('lane_id:'):
                    # lane_id: "xxx"
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        current_node.lane_id = line[q1 + 1:q2]
                elif current_node.road_id is None and line.startswith('road_id:'):
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        current_node.road_id = line[q1 + 1:q2]
                elif current_node.is_virtual is None and line.startswith('is_virtual:'):
                    if 'true' in line:
                        current_node.is_virtual = True
                    elif 'false' in line:
                        current_node.is_virtual = False
                elif (not top_length_captured) and line.startswith('length:'):
                    value = _parse_float(line.split(':', 1)[1].strip())
                    if value is not None:
                        current_node.length_m = value
                        top_length_captured = True
                elif (not top_cost_captured) and line.startswith('cost:'):
                    value = _parse_float(line.split(':', 1)[1].strip())
                    if value is not None:
                        current_node.cost = value
                        top_cost_captured = True

                # line_segment 点采集（central_curve.segment.line_segment 作用域）
                if 'line_segment {' in line and not collecting_line_segment_points:
                    collecting_line_segment_points = True
                    line_segment_depth = 1
                    pending_x = None
                    continue

                if collecting_line_segment_points:
                    # 维护 line_segment 局部深度
                    if '{' in line:
                        line_segment_depth += line.count('{')
                    if '}' in line:
                        line_segment_depth -= line.count('}')

                    if line.startswith('x:'):
                        pending_x = _parse_float(line.split(':', 1)[1].strip())
                    elif line.startswith('y:'):
                        y_value = _parse_float(line.split(':', 1)[1].strip())
                        if pending_x is not None and y_value is not None:
                            current_node.points.append((pending_x, y_value))
                            pending_x = None

                    if line_segment_depth <= 0:
                        collecting_line_segment_points = False
                        pending_x = None

                # 结束 node 块：保存 node
                if node_brace_depth <= 0:
                    in_node = False
                    node_brace_depth = 0

                    # 过滤 bbox
                    keep_node = True
                    if bbox is not None and current_node.points:
                        keep_node = any((point_in_bbox(p) for p in current_node.points))

                    if keep_node and current_node.lane_id is not None:
                        nodes[current_node.lane_id] = current_node
                    current_node = None

                    # 数量限制
                    if max_nodes is not None and len(nodes) >= max_nodes:
                        # 提前中断节点解析，但仍需继续读完文件以采集 edge？
                        # 此处改为仅停止进一步 node 解析，仍允许 edge 解析。
                        pass

                    continue

            # edge 内容解析
            if in_edge:
                if '{' in line:
                    edge_brace_depth += line.count('{')
                if '}' in line:
                    edge_brace_depth -= line.count('}')

                if tmp_from is None and line.startswith('from_lane_id:'):
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        tmp_from = line[q1 + 1:q2]
                elif tmp_to is None and line.startswith('to_lane_id:'):
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        tmp_to = line[q1 + 1:q2]
                elif tmp_dir is None and line.startswith('direction_type:'):
                    # 形如: direction_type: FORWARD/LEFT/RIGHT
                    val = line.split(':', 1)[1].strip()
                    tmp_dir = val

                if edge_brace_depth <= 0:
                    in_edge = False
                    edge_brace_depth = 0
                    if tmp_from and tmp_to:
                        all_edges.append((tmp_from, tmp_to))
                        if tmp_dir is None:
                            tmp_dir = 'FORWARD'
                        # 暂存带类型的边
                        edges_typed.append((tmp_from, tmp_to, tmp_dir))
                    tmp_from, tmp_to, tmp_dir = None, None, None
                    continue

    # 根据 nodes 过滤 edge，并过滤 bbox（若 edge 两端节点都不在 nodes 中则丢弃）
    node_keys = set(nodes.keys())
    for from_id, to_id in all_edges:
        if from_id in node_keys and to_id in node_keys:
            edges.append((from_id, to_id))
    # 过滤带类型的边
    filtered_edges_typed: List[Tuple[str, str, str]] = []
    for fr, to, dt in edges_typed:
        if fr in node_keys and to in node_keys:
            filtered_edges_typed.append((fr, to, dt))

    return nodes, edges, filtered_edges_typed


def parse_lane_samples_from_base_map(file_path: str) -> Dict[str, LaneSamples]:
    """解析 base_map.txt 中每条 lane 的 left_sample/right_sample。"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"未找到文件: {file_path}")

    lane_samples: Dict[str, LaneSamples] = {}

    in_lane = False
    lane_brace_depth = 0
    current_lane_id: Optional[str] = None
    current_left: List[Tuple[float, float]] = []
    current_right: List[Tuple[float, float]] = []

    in_left_sample = False
    left_sample_depth = 0
    left_s: Optional[float] = None
    left_w: Optional[float] = None

    in_right_sample = False
    right_sample_depth = 0
    right_s: Optional[float] = None
    right_w: Optional[float] = None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw_line in f:
            line = raw_line.strip()

            if not in_lane and line.startswith('lane {'):
                in_lane = True
                lane_brace_depth = 1
                current_lane_id = None
                current_left = []
                current_right = []
                in_left_sample = False
                left_sample_depth = 0
                left_s = None
                left_w = None
                in_right_sample = False
                right_sample_depth = 0
                right_s = None
                right_w = None
                continue

            if not in_lane:
                continue

            if '{' in line:
                lane_brace_depth += line.count('{')
            if '}' in line:
                lane_brace_depth -= line.count('}')

            # lane.id.id 的第一处 id 作为当前 lane id
            if current_lane_id is None and line.startswith('id:'):
                q1 = line.find('"')
                q2 = line.rfind('"')
                if q1 != -1 and q2 != -1 and q2 > q1:
                    current_lane_id = line[q1 + 1:q2]

            if not in_left_sample and line.startswith('left_sample {'):
                in_left_sample = True
                left_sample_depth = 1
                left_s = None
                left_w = None
                continue

            if not in_right_sample and line.startswith('right_sample {'):
                in_right_sample = True
                right_sample_depth = 1
                right_s = None
                right_w = None
                continue

            if in_left_sample:
                if '{' in line:
                    left_sample_depth += line.count('{')
                if '}' in line:
                    left_sample_depth -= line.count('}')
                if line.startswith('s:'):
                    left_s = _parse_float(line.split(':', 1)[1].strip())
                elif line.startswith('width:'):
                    left_w = _parse_float(line.split(':', 1)[1].strip())
                if left_sample_depth <= 0:
                    if left_s is not None and left_w is not None:
                        current_left.append((left_s, left_w))
                    in_left_sample = False
                    left_s = None
                    left_w = None

            if in_right_sample:
                if '{' in line:
                    right_sample_depth += line.count('{')
                if '}' in line:
                    right_sample_depth -= line.count('}')
                if line.startswith('s:'):
                    right_s = _parse_float(line.split(':', 1)[1].strip())
                elif line.startswith('width:'):
                    right_w = _parse_float(line.split(':', 1)[1].strip())
                if right_sample_depth <= 0:
                    if right_s is not None and right_w is not None:
                        current_right.append((right_s, right_w))
                    in_right_sample = False
                    right_s = None
                    right_w = None

            if lane_brace_depth <= 0:
                in_lane = False
                lane_brace_depth = 0
                if current_lane_id:
                    current_left.sort(key=lambda x: x[0])
                    current_right.sort(key=lambda x: x[0])
                    lane_samples[current_lane_id] = (current_left, current_right)
                current_lane_id = None
                current_left = []
                current_right = []

    return lane_samples


def parse_supply_stations_from_base_map(file_path: str) -> List['SupplyStation']:
    """解析 base_map.txt 中的 supply_station 块，返回 SupplyStation 列表。"""
    if not os.path.exists(file_path):
        return []

    stations: List[SupplyStation] = []

    in_station = False
    station_brace_depth = 0
    current: Optional[SupplyStation] = None

    in_entry = False
    entry_brace_depth = 0
    in_exit = False
    exit_brace_depth = 0

    in_point_block = False
    point_brace_depth = 0
    point_target: Optional[str] = None  # 'entry_start' / 'entry_end' / 'exit_start' / 'exit_end'
    pending_x: Optional[float] = None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw_line in f:
            line = raw_line.strip()

            if not in_station and line.startswith('supply_station {'):
                in_station = True
                station_brace_depth = 1
                current = SupplyStation()
                in_entry = False
                in_exit = False
                in_point_block = False
                continue

            if not in_station:
                continue

            if '{' in line:
                station_brace_depth += line.count('{')
            if '}' in line:
                station_brace_depth -= line.count('}')

            if current is not None and current.station_id is None and line.startswith('id:'):
                q1 = line.find('"')
                q2 = line.rfind('"')
                if q1 != -1 and q2 != -1 and q2 > q1:
                    current.station_id = line[q1 + 1:q2]

            if not in_entry and not in_exit and line.startswith('entry_line {'):
                in_entry = True
                entry_brace_depth = 1
                continue
            if not in_exit and not in_entry and line.startswith('exit_line {'):
                in_exit = True
                exit_brace_depth = 1
                continue

            if in_entry:
                if '{' in line:
                    entry_brace_depth += line.count('{')
                if '}' in line:
                    entry_brace_depth -= line.count('}')

                if not in_point_block and line.startswith('start_point {'):
                    in_point_block = True
                    point_brace_depth = 1
                    point_target = 'entry_start'
                    pending_x = None
                    continue
                if not in_point_block and line.startswith('end_point {'):
                    in_point_block = True
                    point_brace_depth = 1
                    point_target = 'entry_end'
                    pending_x = None
                    continue

                if entry_brace_depth <= 0:
                    in_entry = False
                    entry_brace_depth = 0

            if in_exit:
                if '{' in line:
                    exit_brace_depth += line.count('{')
                if '}' in line:
                    exit_brace_depth -= line.count('}')

                if not in_point_block and line.startswith('start_point {'):
                    in_point_block = True
                    point_brace_depth = 1
                    point_target = 'exit_start'
                    pending_x = None
                    continue
                if not in_point_block and line.startswith('end_point {'):
                    in_point_block = True
                    point_brace_depth = 1
                    point_target = 'exit_end'
                    pending_x = None
                    continue

                if exit_brace_depth <= 0:
                    in_exit = False
                    exit_brace_depth = 0

            if in_point_block:
                if '{' in line:
                    point_brace_depth += line.count('{')
                if '}' in line:
                    point_brace_depth -= line.count('}')

                if line.startswith('x:'):
                    pending_x = _parse_float(line.split(':', 1)[1].strip())
                elif line.startswith('y:'):
                    y_val = _parse_float(line.split(':', 1)[1].strip())
                    if pending_x is not None and y_val is not None and current is not None and point_target:
                        setattr(current, point_target, (pending_x, y_val))
                    pending_x = None

                if point_brace_depth <= 0:
                    in_point_block = False
                    point_target = None
                    pending_x = None

            if station_brace_depth <= 0:
                in_station = False
                station_brace_depth = 0
                if current is not None and current.station_id is not None:
                    stations.append(current)
                current = None

    return stations


def parse_race_lines_from_base_map(file_path: str) -> List['RaceLine']:
    """解析 base_map.txt 中的 race_line 块，返回 RaceLine 列表。"""
    if not os.path.exists(file_path):
        return []

    race_lines: List[RaceLine] = []

    in_race = False
    race_brace_depth = 0
    current: Optional[RaceLine] = None

    in_point_block = False
    point_brace_depth = 0
    point_target: Optional[str] = None  # 'start_point' / 'end_point'
    pending_x: Optional[float] = None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw_line in f:
            line = raw_line.strip()

            if not in_race and line.startswith('race_line {'):
                in_race = True
                race_brace_depth = 1
                current = RaceLine()
                in_point_block = False
                continue

            if not in_race:
                continue

            if '{' in line:
                race_brace_depth += line.count('{')
            if '}' in line:
                race_brace_depth -= line.count('}')

            if current is not None and current.line_id is None and line.startswith('id:'):
                q1 = line.find('"')
                q2 = line.rfind('"')
                if q1 != -1 and q2 != -1 and q2 > q1:
                    current.line_id = line[q1 + 1:q2]

            if current is not None and current.line_type is None and line.startswith('type:'):
                current.line_type = line.split(':', 1)[1].strip()

            if not in_point_block and line.startswith('start_point {'):
                in_point_block = True
                point_brace_depth = 1
                point_target = 'start_point'
                pending_x = None
                continue
            if not in_point_block and line.startswith('end_point {'):
                in_point_block = True
                point_brace_depth = 1
                point_target = 'end_point'
                pending_x = None
                continue

            if in_point_block:
                if '{' in line:
                    point_brace_depth += line.count('{')
                if '}' in line:
                    point_brace_depth -= line.count('}')

                if line.startswith('x:'):
                    pending_x = _parse_float(line.split(':', 1)[1].strip())
                elif line.startswith('y:'):
                    y_val = _parse_float(line.split(':', 1)[1].strip())
                    if pending_x is not None and y_val is not None and current is not None:
                        if point_target == 'start_point':
                            current.start_point = (pending_x, y_val)
                        elif point_target == 'end_point':
                            current.end_point = (pending_x, y_val)
                    pending_x = None

                if point_brace_depth <= 0:
                    in_point_block = False
                    point_target = None
                    pending_x = None

            if race_brace_depth <= 0:
                in_race = False
                race_brace_depth = 0
                if current is not None and current.line_id is not None:
                    race_lines.append(current)
                current = None

    return race_lines


def _interpolate_point_heading_on_polyline(
    points: List[Tuple[float, float]], s: float
) -> Optional[Tuple[float, float, float]]:
    """沿折线按弧长插值，返回 (x, y, heading)。"""
    if len(points) < 2:
        return None
    if s <= 0.0:
        x1, y1 = points[0]
        x2, y2 = points[1]
        return (x1, y1, math.atan2(y2 - y1, x2 - x1))

    accumulated = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len <= 1e-10:
            continue
        if accumulated + seg_len >= s:
            t = (s - accumulated) / seg_len
            t = max(0.0, min(1.0, t))
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            heading = math.atan2(y2 - y1, x2 - x1)
            return (x, y, heading)
        accumulated += seg_len

    # 超出总弧长时，使用末端点与末段朝向
    x1, y1 = points[-2]
    x2, y2 = points[-1]
    return (x2, y2, math.atan2(y2 - y1, x2 - x1))


def _build_boundary_points_from_samples(
    center_points: List[Tuple[float, float]],
    samples: List[Tuple[float, float]],
    use_left_samples: bool,
) -> List[Tuple[float, float]]:
    """根据中心线与 sampled width 计算左右边界点。"""
    if len(center_points) < 2 or not samples:
        return []

    boundary_points: List[Tuple[float, float]] = []
    for s, width in samples:
        if width is None:
            continue
        pose = _interpolate_point_heading_on_polyline(center_points, s)
        if pose is None:
            continue
        x, y, heading = pose
        if use_left_samples:
            normal_x = -math.sin(heading)
            normal_y = math.cos(heading)
        else:
            normal_x = math.sin(heading)
            normal_y = -math.cos(heading)
        boundary_points.append((x + normal_x * width, y + normal_y * width))
    return boundary_points


def build_boundary_collections_from_samples(
    nodes: Dict[str, LaneNode],
    lane_ids_in_order: List[str],
    lane_samples: Dict[str, LaneSamples],
    line_width: float = 0.6,
    alpha: float = 0.9,
) -> Tuple[Optional[LineCollection], Optional[LineCollection], int, int]:
    """基于 base_map left/right_sample 构建左右边界虚线图层。"""
    left_segments: List[List[Tuple[float, float]]] = []
    right_segments: List[List[Tuple[float, float]]] = []

    for lane_id in lane_ids_in_order:
        lane = nodes.get(lane_id)
        samples = lane_samples.get(lane_id)
        if lane is None or len(lane.points) < 2 or samples is None:
            continue

        left_samples, right_samples = samples
        left_pts = _build_boundary_points_from_samples(lane.points, left_samples, use_left_samples=True)
        right_pts = _build_boundary_points_from_samples(lane.points, right_samples, use_left_samples=False)

        if len(left_pts) >= 2:
            left_segments.append(left_pts)
        if len(right_pts) >= 2:
            right_segments.append(right_pts)

    left_collection: Optional[LineCollection] = None
    right_collection: Optional[LineCollection] = None

    if left_segments:
        left_collection = LineCollection(
            left_segments,
            colors=[(0.05, 0.65, 1.0, alpha)],
            linewidths=line_width,
            linestyles='dashed',
            zorder=7,
        )
    if right_segments:
        right_collection = LineCollection(
            right_segments,
            colors=[(1.0, 0.5, 0.05, alpha)],
            linewidths=line_width,
            linestyles='dashed',
            zorder=7,
        )

    return left_collection, right_collection, len(left_segments), len(right_segments)


def build_collections(
    nodes: Dict[str, LaneNode],
    edges: List[Tuple[str, str]],
    color_by: str = 'road',
    draw_edges: bool = False,
    max_edges: Optional[int] = None,
) -> Tuple[LineCollection, Optional[LineCollection], List[str]]:
    """构建用于绘制的 LineCollection。"""
    # 车道中心线集合
    line_segments: List[List[Tuple[float, float]]] = []
    colors: List[Tuple[float, float, float]] = []

    lane_ids_in_order: List[str] = []
    for lane_id, lane in nodes.items():
        if len(lane.points) < 2:
            continue
        line_segments.append(lane.points)
        lane_ids_in_order.append(lane_id)
        if color_by == 'road':
            colors.append(_color_from_string(lane.road_id))
        elif color_by == 'cost':
            # 按成本映射为颜色（低->蓝，高->红）
            cost = lane.cost if lane.cost is not None else 0.0
            # 简单归一化：假设 cost 落在 [0, 10]，超界裁剪
            c = max(0.0, min(1.0, cost / 10.0))
            colors.append((c, 0.2, 1.0 - c))
        else:
            colors.append((0.3, 0.3, 0.3))

    lane_collection = LineCollection(line_segments, colors=colors, linewidths=0.8, alpha=0.9)

    # 边集合（可选）
    edge_collection: Optional[LineCollection] = None
    if draw_edges and edges:
        edge_segments: List[List[Tuple[float, float]]] = []
        count = 0
        for from_id, to_id in edges:
            if from_id not in nodes or to_id not in nodes:
                continue
            a = nodes[from_id]
            b = nodes[to_id]
            if not a.points or not b.points:
                continue
            # 使用 from 末端 -> to 起点 作为连线
            p0 = a.points[-1]
            p1 = b.points[0]
            edge_segments.append([p0, p1])
            count += 1
            if max_edges is not None and count >= max_edges:
                break
        edge_collection = LineCollection(edge_segments, colors=(0.1, 0.1, 0.1, 0.15), linewidths=0.5)

    return lane_collection, edge_collection, lane_ids_in_order


def build_neighbor_maps(
    nodes: Dict[str, LaneNode],
    edges_typed: List[Tuple[str, str, str]],
) -> Tuple[Dict[str, Set[str]], Dict[str, Set[str]], Dict[str, Set[str]], Dict[str, Set[str]]]:
    """返回 (successors, predecessors, lefts, rights) 四个邻接映射。"""
    successors: Dict[str, Set[str]] = {k: set() for k in nodes.keys()}
    predecessors: Dict[str, Set[str]] = {k: set() for k in nodes.keys()}
    lefts: Dict[str, Set[str]] = {k: set() for k in nodes.keys()}
    rights: Dict[str, Set[str]] = {k: set() for k in nodes.keys()}

    for fr, to, dt in edges_typed:
        if dt == 'FORWARD':
            successors[fr].add(to)
            predecessors[to].add(fr)
        elif dt == 'LEFT':
            lefts[fr].add(to)
            rights[to].add(fr)
        elif dt == 'RIGHT':
            rights[fr].add(to)
            lefts[to].add(fr)
        else:
            # 其它类型（如 UTURN 等）暂不处理为前后左右
            pass

    return successors, predecessors, lefts, rights


def choose_chinese_font(preferred_name: Optional[str] = None) -> Optional[str]:
    """选择一个可用的中文字体名，用户指定优先。返回字体名或 None。"""
    if preferred_name:
        try:
            prop = font_manager.FontProperties(family=preferred_name)
            font_manager.findfont(prop, fallback_to_default=False)
            return preferred_name
        except Exception:
            pass

    candidates = [
        'Noto Sans CJK SC',
        'Source Han Sans SC',
        'SimHei',
        'Microsoft YaHei',
        'WenQuanYi Zen Hei',
        'PingFang SC',
        'Hiragino Sans GB',
        'Arial Unicode MS',
    ]
    for name in candidates:
        try:
            prop = font_manager.FontProperties(family=name)
            font_manager.findfont(prop, fallback_to_default=False)
            return name
        except Exception:
            continue
    return None


def _bundled_cjk_font_path() -> Optional[str]:
    """
    使用与本脚本同目录下的 fonts/ 中的字体文件，无需系统级安装。
    仓库默认附带 NotoSansCJKsc-Regular.otf（SIL Open Font License 1.1）。
    亦可自行放入其他 .otf/.ttf/.ttc（见 fonts/README.txt）。
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fonts_dir = os.path.join(script_dir, 'fonts')
    preferred_names = [
        'NotoSansCJKsc-Regular.otf',
        'NotoSansSC-Regular.otf',
        'NotoSansSC-Medium.otf',
        'NotoSansSC-Bold.otf',
        'SourceHanSansSC-Regular.otf',
        'SourceHanSansCN-Regular.otf',
        'DroidSansFallback.ttf',
        'SimHei.ttf',
        'msyh.ttc',
        'msyhl.ttc',
    ]
    for name in preferred_names:
        p = os.path.join(fonts_dir, name)
        if os.path.isfile(p):
            return p
    if os.path.isdir(fonts_dir):
        found: List[str] = []
        for fn in sorted(os.listdir(fonts_dir)):
            if fn.startswith('.'):
                continue
            low = fn.lower()
            if low.endswith(('.ttf', '.otf', '.ttc')):
                found.append(os.path.join(fonts_dir, fn))
        if len(found) == 1:
            return found[0]
    return None


def _try_register_matplotlib_font(path: str) -> None:
    try:
        font_manager.fontManager.addfont(path)
    except Exception:
        pass


def _rc_font_size_to_points(*rc_keys: str, default_pt: float = 12.0) -> float:
    """将 rcParams 中的字号转为磅值；兼容数值与 'large' 等相对尺寸字符串。"""
    for key in rc_keys:
        try:
            raw = matplotlib.rcParams[key]
        except KeyError:
            continue
        try:
            return float(raw)
        except (TypeError, ValueError):
            try:
                return float(font_manager.FontProperties(size=raw).get_size_in_points())
            except Exception:
                continue
    return default_pt


def resolve_ui_cjk_font_path(
    preferred_family: Optional[str],
    explicit_file: Optional[str],
) -> Optional[str]:
    """
    解析标题/状态面板用的中文字体文件路径。
    优先级：--cjk-font-file > 环境变量 VISUAL_TOOL_CJK_FONT > 脚本同目录 fonts/（仓库自带）>
    系统字体（--font-name 或默认候选）。
    """
    if explicit_file:
        p = os.path.abspath(os.path.expanduser(explicit_file.strip()))
        if os.path.isfile(p):
            _try_register_matplotlib_font(p)
            return p
    env_p = (os.environ.get('VISUAL_TOOL_CJK_FONT') or '').strip()
    if env_p:
        p = os.path.abspath(os.path.expanduser(env_p))
        if os.path.isfile(p):
            _try_register_matplotlib_font(p)
            return p
    bundled = _bundled_cjk_font_path()
    if bundled:
        _try_register_matplotlib_font(bundled)
        return bundled
    for fp_try in (
        resolve_chinese_fontproperties(preferred_family),
        resolve_chinese_fontproperties(None),
    ):
        if fp_try is None:
            continue
        try:
            path = str(font_manager.findfont(fp_try))
        except Exception:
            continue
        if path and 'dejavu' not in path.lower():
            return path
    return None


def resolve_chinese_fontproperties(preferred_name: Optional[str] = None) -> Optional[font_manager.FontProperties]:
    """
    解析到磁盘上的字体文件并返回 FontProperties。
    部分环境下 ax.text 不会正确继承 rcParams 的 sans-serif CJK 列表，显式 fname 可正常显示中文。
    """
    ordered: List[str] = []
    if preferred_name:
        ordered.append(preferred_name)
    ordered.extend([
        'Noto Sans CJK SC',
        'Noto Sans CJK TC',
        'Noto Sans CJK JP',
        'Noto Sans CJK KR',
        'Source Han Sans SC',
        'Source Han Sans CN',
        'WenQuanYi Zen Hei',
        'WenQuanYi Micro Hei',
        'Droid Sans Fallback',
        'SimHei',
        'Microsoft YaHei',
        'PingFang SC',
        'Hiragino Sans GB',
        'Arial Unicode MS',
    ])
    seen: Set[str] = set()
    for name in ordered:
        if not name or name in seen:
            continue
        seen.add(name)
        try:
            prop = font_manager.FontProperties(family=name)
            path = font_manager.findfont(prop, fallback_to_default=False)
        except Exception:
            continue
        if not path:
            continue
        pl = path.lower()
        if 'dejavu' in pl:
            continue
        try:
            return font_manager.FontProperties(fname=path)
        except Exception:
            continue
    return None


def auto_bbox_from_nodes(nodes: Dict[str, LaneNode]) -> Optional[Tuple[float, float, float, float]]:
    xs: List[float] = []
    ys: List[float] = []
    for lane in nodes.values():
        for x, y in lane.points:
            xs.append(x)
            ys.append(y)
    if not xs or not ys:
        return None
    return (min(xs), min(ys), max(xs), max(ys))


def quaternion_to_yaw(q: Quaternion) -> float:
    """将四元数转换为偏航角（弧度）。"""
    # 四元数转欧拉角 (yaw)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def _marker_rgba(m: Marker, default: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """从 Marker.color 提取 RGBA（0-1），若未设置则回退 default。"""
    try:
        r = float(m.color.r)
        g = float(m.color.g)
        b = float(m.color.b)
        a = float(m.color.a)
        # 常见情况：颜色字段全 0（未设置），此时用默认色
        if (r, g, b, a) == (0.0, 0.0, 0.0, 0.0):
            return default
        # 若 alpha 为 0 但 rgb 非 0，仍给个默认 alpha
        if a <= 1e-6:
            return (r, g, b, default[3])
        return (r, g, b, a)
    except Exception:
        return default


def _transform_local_points_2d(
    pts: Iterable,
    tx: float,
    ty: float,
    yaw: float,
) -> List[Tuple[float, float]]:
    """将 Marker.points（局部坐标）按 pose(平移+yaw) 转换到世界坐标，仅取 x/y。"""
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    out: List[Tuple[float, float]] = []
    for p in pts:
        try:
            x = float(p.x)
            y = float(p.y)
        except Exception:
            continue
        gx = tx + cy * x - sy * y
        gy = ty + sy * x + cy * y
        out.append((gx, gy))
    return out


def _find_routing_response_file(path_or_dir: str) -> Optional[str]:
    """接受文件或目录；若为目录，尝试在其中寻找 RoutingResponse 文本文件."""
    if not path_or_dir:
        return None
    if os.path.isfile(path_or_dir):
        return path_or_dir
    if os.path.isdir(path_or_dir):
        # 优先匹配更具体的命名
        candidates: List[str] = []
        try:
            for name in os.listdir(path_or_dir):
                full = os.path.join(path_or_dir, name)
                if not os.path.isfile(full):
                    continue
                low = name.lower()
                if low.endswith('.pb.txt') and ('routing' in low or 'response' in low):
                    candidates.append(full)
            # 退而求其次：任意 .pb.txt
            if not candidates:
                for name in os.listdir(path_or_dir):
                    full = os.path.join(path_or_dir, name)
                    if os.path.isfile(full) and name.lower().endswith('.pb.txt'):
                        candidates.append(full)
        except Exception:
            return None
        if not candidates:
            return None
        # 按修改时间倒序，取最新
        candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return candidates[0]
    return None


def parse_routing_response_segments(file_path: str) -> List[str]:
    """
    解析 RoutingResponse 文本（proto text），抽取所有 passage.segment.id。
    仅返回 segment 的 id 列表（按出现顺序去重）。
    """
    ids: List[str] = []
    seen: Set[str] = set()
    if not file_path or not os.path.exists(file_path):
        return ids
    in_segment = False
    seg_brace_depth = 0
    current_id: Optional[str] = None
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = raw.strip()
            if not in_segment and line.startswith('segment {'):
                in_segment = True
                seg_brace_depth = 1
                current_id = None
                continue
            if in_segment:
                if '{' in line:
                    seg_brace_depth += line.count('{')
                if '}' in line:
                    seg_brace_depth -= line.count('}')
                if current_id is None and line.startswith('id:'):
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        current_id = line[q1 + 1:q2]
                if seg_brace_depth <= 0:
                    in_segment = False
                    if current_id and current_id not in seen:
                        ids.append(current_id)
                        seen.add(current_id)
                    current_id = None
    return ids


def parse_routing_response_segments_detailed(file_path: str) -> List[Tuple[str, Optional[float], Optional[float]]]:
    """
    解析 RoutingResponse 文本（proto text），抽取所有 passage.segment 的 (id, start_s, end_s)。
    返回按出现顺序的列表。
    """
    result: List[Tuple[str, Optional[float], Optional[float]]] = []
    if not file_path or not os.path.exists(file_path):
        return result
    in_segment = False
    seg_brace_depth = 0
    seg_id: Optional[str] = None
    seg_start: Optional[float] = None
    seg_end: Optional[float] = None
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = raw.strip()
            if not in_segment and line.startswith('segment {'):
                in_segment = True
                seg_brace_depth = 1
                seg_id = None
                seg_start = None
                seg_end = None
                continue
            if in_segment:
                if '{' in line:
                    seg_brace_depth += line.count('{')
                if '}' in line:
                    seg_brace_depth -= line.count('}')
                if line.startswith('id:'):
                    q1 = line.find('"')
                    q2 = line.rfind('"')
                    if q1 != -1 and q2 != -1 and q2 > q1:
                        seg_id = line[q1 + 1:q2]
                elif line.startswith('start_s:'):
                    try:
                        seg_start = float(line.split(':', 1)[1].strip())
                    except Exception:
                        seg_start = None
                elif line.startswith('end_s:'):
                    try:
                        seg_end = float(line.split(':', 1)[1].strip())
                    except Exception:
                        seg_end = None
                if seg_brace_depth <= 0:
                    in_segment = False
                    if seg_id is not None:
                        result.append((seg_id, seg_start, seg_end))
                    seg_id, seg_start, seg_end = None, None, None
    return result


def parse_routing_response_waypoints(file_path: str) -> List[Tuple[Optional[str], Optional[float], Optional[float], Optional[float]]]:
    """
    解析 RoutingResponse 文本，抽取 routing_request.waypoint 列表。
    返回 [(lane_id, s, pose_x, pose_y), ...]，按出现顺序。
    """
    wps: List[Tuple[Optional[str], Optional[float], Optional[float], Optional[float]]] = []
    if not file_path or not os.path.exists(file_path):
        return wps
    in_req = False
    req_depth = 0
    in_wp = False
    wp_depth = 0
    in_pose = False
    pose_depth = 0
    lane_id: Optional[str] = None
    s_val: Optional[float] = None
    px: Optional[float] = None
    py: Optional[float] = None
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = raw.strip()
            # 进入/退出 routing_request
            if not in_req and line.startswith('routing_request {'):
                in_req = True
                req_depth = 1
                continue
            if in_req:
                if '{' in line:
                    req_depth += line.count('{')
                if '}' in line:
                    req_depth -= line.count('}')
                if not in_wp and line.startswith('waypoint {'):
                    in_wp = True
                    wp_depth = 1
                    lane_id = None
                    s_val = None
                    px = None
                    py = None
                    continue
                if in_wp:
                    if '{' in line:
                        wp_depth += line.count('{')
                    if '}' in line:
                        wp_depth -= line.count('}')
                    if not in_pose and line.startswith('pose {'):
                        in_pose = True
                        pose_depth = 1
                        continue
                    if in_pose:
                        if '{' in line:
                            pose_depth += line.count('{')
                        if '}' in line:
                            pose_depth -= line.count('}')
                        if line.startswith('x:'):
                            try:
                                px = float(line.split(':', 1)[1].strip())
                            except Exception:
                                px = None
                        elif line.startswith('y:'):
                            try:
                                py = float(line.split(':', 1)[1].strip())
                            except Exception:
                                py = None
                        if pose_depth <= 0:
                            in_pose = False
                            pose_depth = 0
                            # 继续处理 waypoint 其余字段
                            continue
                    # waypoint 顶层字段
                    if line.startswith('id:'):
                        q1 = line.find('"')
                        q2 = line.rfind('"')
                        if q1 != -1 and q2 != -1 and q2 > q1:
                            lane_id = line[q1 + 1:q2]
                    elif line.startswith('s:'):
                        try:
                            s_val = float(line.split(':', 1)[1].strip())
                        except Exception:
                            s_val = None
                    if wp_depth <= 0:
                        in_wp = False
                        wps.append((lane_id, s_val, px, py))
                        lane_id, s_val, px, py = None, None, None, None
                        continue
                if req_depth <= 0:
                    in_req = False
                    req_depth = 0
                    continue
    return wps


def _parse_color_to_rgba(color_str: Optional[str], default: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """解析 '#RRGGBB(AA)' 或 'r,g,b(,a)' 到 RGBA(0-1)。"""
    if not color_str:
        return default
    s = color_str.strip()
    try:
        if s.startswith('#'):
            hexv = s[1:]
            if len(hexv) == 6:
                r = int(hexv[0:2], 16) / 255.0
                g = int(hexv[2:4], 16) / 255.0
                b = int(hexv[4:6], 16) / 255.0
                return (r, g, b, default[3])
            if len(hexv) == 8:
                r = int(hexv[0:2], 16) / 255.0
                g = int(hexv[2:4], 16) / 255.0
                b = int(hexv[4:6], 16) / 255.0
                a = int(hexv[6:8], 16) / 255.0
                return (r, g, b, a)
        parts = [p.strip() for p in s.split(',')]
        if len(parts) >= 3:
            r = float(parts[0])
            g = float(parts[1])
            b = float(parts[2])
            a = float(parts[3]) if len(parts) >= 4 else default[3]
            return (r, g, b, a)
    except Exception:
        return default
    return default


def _speed_to_rgba(
    speed: float,
    min_speed: float = 0.2,
    max_speed: float = 3.0,
    min_alpha: float = 0.4,
    max_alpha: float = 1.0,
) -> Tuple[float, float, float, float]:
    """速度映射到颜色：低速蓝(半透明) -> 高速红(不透明)。"""
    if max_speed <= min_speed:
        return (1.0, 0.0, 0.0, max_alpha)
    s = max(min_speed, min(max_speed, speed))
    t = (s - min_speed) / (max_speed - min_speed)
    # 蓝 -> 红
    r = t
    g = 0.0
    b = 1.0 - t
    a = min_alpha + (max_alpha - min_alpha) * t
    return (r, g, b, a)


def build_route_overlay_collection(
    nodes: Dict[str, LaneNode],
    route_lane_ids: List[str],
    color_rgba: Tuple[float, float, float, float],
    linewidth: float,
) -> Tuple[Optional[LineCollection], List[str], List[str]]:
    """基于给定 lane_id 列表构建路线覆盖层。返回 (collection, found_ids, missing_ids)。"""
    if not route_lane_ids:
        return None, [], []
    segments: List[List[Tuple[float, float]]] = []
    found: List[str] = []
    missing: List[str] = []
    for lid in route_lane_ids:
        lane = nodes.get(lid)
        if lane is None or len(lane.points) < 2:
            missing.append(lid)
            continue
        segments.append(lane.points)
        found.append(lid)
    if not segments:
        return None, found, missing
    lc = LineCollection(segments, colors=[(color_rgba[0], color_rgba[1], color_rgba[2], color_rgba[3])], linewidths=linewidth, zorder=8)
    try:
        lc.set_capstyle('round')
        lc.set_joinstyle('round')
    except Exception:
        pass
    return lc, found, missing


def _interpolate_point_on_polyline(points: List[Tuple[float, float]], s: float) -> Optional[Tuple[float, float]]:
    """沿折线按弧长定位点坐标。s 为起点累计长度，单位与坐标一致。"""
    if not points or len(points) < 2:
        return None
    if s <= 0.0:
        return points[0]
    # 计算累计长度并定位
    accumulated = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len <= 1e-10:
            continue
        if accumulated + seg_len >= s:
            t = (s - accumulated) / seg_len
            t = max(0.0, min(1.0, t))
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
        accumulated += seg_len
    # 超出总长则返回终点
    return points[-1]


def _leg_joint_label(name: int) -> Optional[str]:
    """51–56：左腿1–6；61–66：右腿1–6（与 bodyctrl_msgs/MotorName 约定一致）。"""
    if 51 <= name <= 56:
        return f"左腿{name - 50}"
    if 61 <= name <= 66:
        return f"右腿{name - 60}"
    return None


def _arm_joint_label(name: int) -> Optional[str]:
    """11–17：左臂1–7；21–27：右臂1–7（与 bodyctrl_msgs/MotorName 约定一致）。"""
    if 11 <= name <= 17:
        return f"左臂{name - 10}"
    if 21 <= name <= 27:
        return f"右臂{name - 20}"
    return None


def _format_leg_motor_temp_rows(msg: MotorStatusMsg1) -> List[List[Tuple[str, str, bool]]]:
    """每行多段 (文案, 颜色, 是否加粗)。标签+冒号加粗；温度数值根据阈值上色。"""
    by_name: Dict[int, Tuple[float, float]] = {}
    for st in msg.status:
        try:
            n = int(st.name)
        except Exception:
            continue
        if _leg_joint_label(n) is None:
            continue
        try:
            by_name[n] = (float(st.motortemperature), float(st.mostemperature))
        except Exception:
            continue
    rows: List[List[Tuple[str, str, bool]]] = []
    for n in list(range(51, 57)) + list(range(61, 67)):
        if n not in by_name:
            continue
        label = _leg_joint_label(n)
        if label is None:
            continue
        mt, mos = by_name[n]
        if label.startswith('左腿'):
            lc = _LABEL_GREEN
        elif label.startswith('右腿'):
            lc = _LABEL_BLUE
        else:
            lc = _OVERLAY_NORMAL_COLOR

        temp_color = _TEMP_OK_COLOR
        if (mt >= TEMP_MOTOR_TEMP_HIGH) or (mos >= TEMP_MOS_TEMP_HIGH):
            temp_color = _TEMP_HIGH_COLOR
        rows.append(
            [
                ('  ', _OVERLAY_SUFFIX_BLACK, False),
                (label + '：', lc, True),
                (f"{mt:.1f}°C mos {mos:.1f}°C", temp_color, False),
            ]
        )
    return rows


def _format_arm_motor_temp_rows(msg: MotorStatusMsg1) -> List[List[Tuple[str, str, bool]]]:
    """手臂关节温度行，格式与腿一致；左右颜色与腿相同（左绿右蓝）；温度数值根据阈值上色。"""
    by_name: Dict[int, Tuple[float, float]] = {}
    for st in msg.status:
        try:
            n = int(st.name)
        except Exception:
            continue
        if _arm_joint_label(n) is None:
            continue
        try:
            by_name[n] = (float(st.motortemperature), float(st.mostemperature))
        except Exception:
            continue
    rows: List[List[Tuple[str, str, bool]]] = []
    for n in list(range(11, 18)) + list(range(21, 28)):
        if n not in by_name:
            continue
        label = _arm_joint_label(n)
        if label is None:
            continue
        mt, mos = by_name[n]
        if label.startswith('左臂'):
            lc = _LABEL_GREEN
        elif label.startswith('右臂'):
            lc = _LABEL_BLUE
        else:
            lc = _OVERLAY_NORMAL_COLOR

        temp_color = _TEMP_OK_COLOR
        if (mt >= TEMP_MOTOR_TEMP_HIGH) or (mos >= TEMP_MOS_TEMP_HIGH):
            temp_color = _TEMP_HIGH_COLOR
        rows.append(
            [
                ('  ', _OVERLAY_SUFFIX_BLACK, False),
                (label + '：', lc, True),
                (f"{mt:.1f}°C mos {mos:.1f}°C", temp_color, False),
            ]
        )
    return rows


def _build_status_overlay_rows(
    speed_mps: Optional[float],
    battery_pct: Optional[float],
    leg_rows: Optional[List[List[Tuple[str, str, bool]]]],
    arm_rows: Optional[List[List[Tuple[str, str, bool]]]],
) -> List[List[Tuple[str, str, bool]]]:
    """组装状态面板：每行内为多段 (文案, 颜色, 加粗)，右对齐拼接。腿/臂关节温度行接在首行（速度、电量）之后，无单独小节标题。"""
    rows: List[List[Tuple[str, str, bool]]] = []
    line1: List[Tuple[str, str, bool]] = [
        ("速度：", _OVERLAY_NORMAL_COLOR, True),
    ]
    if speed_mps is not None:
        line1.append((f"{speed_mps:.3f} m/s", _OVERLAY_SUFFIX_BLACK, False))
    else:
        line1.append(("-- m/s", _OVERLAY_SUFFIX_BLACK, False))
    line1.append(("  ", _OVERLAY_NORMAL_COLOR, False))
    line1.append(("电量：", _OVERLAY_NORMAL_COLOR, True))
    if battery_pct is not None:
        line1.append((f"{battery_pct:.1f}%", _OVERLAY_SUFFIX_BLACK, False))
    else:
        line1.append(("--", _OVERLAY_SUFFIX_BLACK, False))
    rows.append(line1)
    temp_lines: List[List[Tuple[str, str, bool]]] = []
    for segs in leg_rows or []:
        temp_lines.append(segs)
    for segs in arm_rows or []:
        temp_lines.append(segs)
    if temp_lines:
        for segs in temp_lines:
            rows.append(segs)
    elif leg_rows is None and arm_rows is None:
        rows.append([("  （等待数据）", _OVERLAY_NORMAL_COLOR, False)])
    else:
        rows.append([("  （无数据）", _OVERLAY_NORMAL_COLOR, False)])
    return rows


class ROS2SubscriberNode(Node):
    """ROS2订阅节点，用于接收实时数据。"""
    
    def __init__(
        self,
        data_lock: threading.Lock,
        shared_data: Dict,
         enable_dp_path: bool = False,
         enable_qp_path: bool = False,
        enable_plot_tra: bool = False,
        use_origin_obs: bool = True,
        use_converted_obs: bool = False,
    ):
        super().__init__('routing_map_visualizer')
        self.data_lock = data_lock
        self.shared_data = shared_data
        
        # 创建回调组
        callback_group = ReentrantCallbackGroup()
        
        # QoS配置
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅机器人位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual/loc/pose',
            self.odom_callback,
            qos,
            callback_group=callback_group
        )
        
        # 订阅参考线
        self.refline_sub = self.create_subscription(
            Path,
            '/hric/refline/tmp',
            self.refline_callback,
            qos,
            callback_group=callback_group
        )
        
        # 订阅规划路径（默认使用 planning/path）
        self.planning_path_sub = None
        if not enable_plot_tra:
            self.planning_path_sub = self.create_subscription(
                Path,
                '/visual/plan_path',
                self.planning_path_callback,
                qos,
                callback_group=callback_group
            )

        # 订阅规划轨迹（PlannedTrajectory）
        self.planned_trajectory_sub = None
        if enable_plot_tra:
            self.planned_trajectory_sub = self.create_subscription(
                PlannedTrajectory,
                '/hric/nav/plan_trajectory',
                self.planned_trajectory_callback,
                qos,
                callback_group=callback_group
            )

        # 订阅前视点（默认开启）
        self.lookahead_pose_sub = self.create_subscription(
            PoseStamped,
            '/lookahead_pose',
            self.lookahead_pose_callback,
            qos,
            callback_group=callback_group
        )

        # 订阅 DP 路径（可选）
        self.dp_path_sub = None
        if enable_dp_path:
            self.dp_path_sub = self.create_subscription(
                Path,
                'planning/dp_path',
                self.dp_path_callback,
                qos,
                callback_group=callback_group
            )

        # 订阅 QP 路径（可选）
        self.qp_path_sub = None
        if enable_qp_path:
            self.qp_path_sub = self.create_subscription(
                Path,
                'planning/qp_path',
                self.qp_path_callback,
                qos,
                callback_group=callback_group
            )

        # 订阅感知障碍物（原始 ObjectArray）
        self.objects_sub = None
        if use_origin_obs:
            self.objects_sub = self.create_subscription(
                ObjectArray,
                '/perception/objects',
                self.objects_callback,
                qos,
                callback_group=callback_group
            )

        # 订阅转换后的障碍物（MarkerArray）
        self.converted_obs_sub = None
        if use_converted_obs:
            self.converted_obs_sub = self.create_subscription(
                MarkerArray,
                '/visual/perception/objects',
                self.converted_obstacles_callback,
                qos,
                callback_group=callback_group
            )

        # 右上角状态：与 /visual/plan_path 同链路，统一使用 /visual/* 话题
        self.battery_status_sub = self.create_subscription(
            PowerBatteryStatus,
            '/visual/power/battery/status',
            self.battery_status_callback,
            qos,
            callback_group=callback_group,
        )
        self.leg_motor_status_sub = self.create_subscription(
            MotorStatusMsg1,
            '/visual/leg/motor_status',
            self.leg_motor_status_callback,
            qos,
            callback_group=callback_group,
        )
        self.arm_motor_status_sub = self.create_subscription(
            MotorStatusMsg1,
            '/visual/arm/motor_status',
            self.arm_motor_status_callback,
            qos,
            callback_group=callback_group,
        )
        self.rtk_gps_sub = self.create_subscription(
            RtkGps,
            '/visual/rtk_gps',
            self.rtk_gps_callback,
            qos,
            callback_group=callback_group,
        )

        # 远程服务调用：JSON 通过 std_msgs/String 桥到 udp_client
        self.remote_call_pub = self.create_publisher(
            String, '/visual/remote_service_call', 10
        )
        self.remote_result_sub = self.create_subscription(
            String, '/visual/remote_service_result',
            self._remote_service_result_callback, 10,
            callback_group=callback_group,
        )

        self.get_logger().info('ROS2订阅节点已初始化')

    def publish_remote_service_call(self, service_name: str, srv_type: str,
                                    payload: dict, request_id: Optional[str] = None):
        """发起一次远程服务调用（由快捷键触发）。"""
        req = {
            'service': service_name,
            'srv_type': srv_type,
            'payload': payload or {},
        }
        if request_id:
            req['request_id'] = request_id
        out = String()
        out.data = json.dumps(req)
        self.remote_call_pub.publish(out)

    def _remote_service_result_callback(self, msg: String):
        try:
            data = json.loads(msg.data) if msg.data else {}
        except Exception:
            return
        with self.data_lock:
            q = self.shared_data.setdefault('remote_service_result_queue', [])
            q.append(data)
    
    def battery_status_callback(self, msg: PowerBatteryStatus):
        with self.data_lock:
            try:
                self.shared_data['battery_power_pct'] = float(msg.master_battery_power)
            except Exception:
                pass

    def leg_motor_status_callback(self, msg: MotorStatusMsg1):
        rows = _format_leg_motor_temp_rows(msg)
        with self.data_lock:
            self.shared_data['leg_motor_overlay_lines'] = rows

    def arm_motor_status_callback(self, msg: MotorStatusMsg1):
        rows = _format_arm_motor_temp_rows(msg)
        with self.data_lock:
            self.shared_data['arm_motor_overlay_lines'] = rows

    def rtk_gps_callback(self, msg: RtkGps):
        with self.data_lock:
            try:
                self.shared_data['rtk_gps_speed_mps'] = float(msg.gprmc_speed)
            except Exception:
                pass

    def odom_callback(self, msg: Odometry):
        """机器人位置回调。"""
        with self.data_lock:
            pose = msg.pose.pose
            self.shared_data['robot_pose'] = {
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': quaternion_to_yaw(pose.orientation)
            }
    
    def refline_callback(self, msg: Path):
        """参考线回调。"""
        with self.data_lock:
            points = []
            for pose_stamped in msg.poses:
                points.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            self.shared_data['refline'] = points
    
    def planning_path_callback(self, msg: Path):
        """规划路径回调。"""
        with self.data_lock:
            points = []
            for pose_stamped in msg.poses:
                points.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            self.shared_data['planning_path'] = points

    def planned_trajectory_callback(self, msg: PlannedTrajectory):
        """规划轨迹回调：提取点坐标与线速度。"""
        points = []
        for pt in msg.points:
            try:
                x = float(pt.pose.position.x)
                y = float(pt.pose.position.y)
                vx = float(pt.velocity.linear.x)
                vy = float(pt.velocity.linear.y)
                vz = float(pt.velocity.linear.z)
                speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            except Exception:
                continue
            points.append((x, y, speed))
        with self.data_lock:
            self.shared_data['planned_trajectory'] = points

    def dp_path_callback(self, msg: Path):
        """DP路径回调。"""
        with self.data_lock:
            points = []
            for pose_stamped in msg.poses:
                points.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            self.shared_data['dp_path'] = points

    def qp_path_callback(self, msg: Path):
        """QP路径回调。"""
        with self.data_lock:
            points = []
            for pose_stamped in msg.poses:
                points.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            self.shared_data['qp_path'] = points

    def lookahead_pose_callback(self, msg: PoseStamped):
        """前视点回调。"""
        with self.data_lock:
            pose = msg.pose
            self.shared_data['lookahead_pose'] = {
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': quaternion_to_yaw(pose.orientation)
            }

    def objects_callback(self, msg: ObjectArray):
        """感知障碍物回调：提取每个 Object.shape 的顶点坐标（x,y）。"""
        polys: List[List[Tuple[float, float]]] = []
        meta: List[Dict] = []
        for obj in msg.objects:
            try:
                shape = obj.shape
                # 新消息为 geometry_msgs/Point[]；兼容旧的 Polygon.points
                pts = shape.points if hasattr(shape, 'points') else shape
            except Exception:
                continue
            if not pts or len(pts) < 3:
                continue
            poly_xy: List[Tuple[float, float]] = []
            for p in pts:
                poly_xy.append((float(p.x), float(p.y)))
            polys.append(poly_xy)
            meta.append({
                'id': getattr(obj, 'id', ''),
                'type': int(getattr(obj, 'type', 0)),
                'confidence': float(getattr(obj, 'confidence', 0.0)),
            })
        with self.data_lock:
            self.shared_data['perception_objects_polys'] = polys
            self.shared_data['perception_objects_meta'] = meta

    def converted_obstacles_callback(self, msg: MarkerArray):
        """转换后障碍物回调：保存 MarkerArray.markers，绘制时解析。"""
        with self.data_lock:
            self.shared_data['converted_obstacle_markers'] = list(msg.markers)


def main() -> None:
    parser = argparse.ArgumentParser(description='实时可视化地图目录中的 routing_map/base_map（Apollo 路网）及机器人位置、参考线、规划路径')
    parser.add_argument('--map-dir', '-m', default='/colcon_ws/map_data/guoqizhilian_modified2', help='地图目录路径（目录下需包含 routing_map.txt 与 base_map.txt）')
    parser.add_argument('--bbox', nargs=4, type=float, default=None, metavar=('MIN_X', 'MIN_Y', 'MAX_X', 'MAX_Y'), help='可选的绘制边界框')
    parser.add_argument('--max-nodes', type=int, default=None, help='最多保留的节点数量（按出现顺序）')
    parser.add_argument('--draw-edges', action='store_true', help='是否绘制拓扑边')
    parser.add_argument('--max-edges', type=int, default=None, help='最多绘制的边数量（防止过密）')
    parser.add_argument('--color-by', choices=['road', 'cost', 'none'], default='road', help='节点颜色映射方式')
    parser.add_argument('--figsize', nargs=2, type=float, default=(12.0, 12.0), metavar=('W', 'H'), help='画布尺寸（英寸）')
    parser.add_argument('--font-name', type=str, default=None, help='中文字体名称（如 Noto Sans CJK SC）')
    parser.add_argument(
        '--cjk-font-file',
        type=str,
        default=None,
        help='中文字体文件路径（.ttf/.otf/.ttc），无需 apt 安装；也可用环境变量 VISUAL_TOOL_CJK_FONT',
    )
    parser.add_argument('--arrow-scale', type=float, default=12.0, help='箭头尺寸（mutation_scale）')
    parser.add_argument('--two-way-eps', type=float, default=1.5, help='双向启发式阈值（端点匹配距离，单位与坐标一致）')
    parser.add_argument('--lane-width', type=float, default=3.5, help='车道宽度（米），用于绘制车道区域')
    parser.add_argument('--lane-fill-alpha', type=float, default=0.25, help='车道填充区域透明度（0-1）')
    parser.add_argument('--interactive', action='store_true', help='启用交互：点击车道显示 ID 与邻接')
    parser.add_argument('--pick-radius', type=float, default=3.0, help='拾取半径（数据坐标单位）')
    parser.add_argument('--highlight-width', type=float, default=2.5, help='高亮时的线宽')
    parser.add_argument('--hover', action='store_true', help='开启悬停提示（需交互模式）')
    parser.add_argument('--follow-pose-window', type=float, default=20.0, help='锁定跟随窗口边长（米），默认 20m（窗口大小为 N x N）')
    parser.add_argument('--follow-pose-key', type=str, default='f', help='切换 /hric/loc/pose 视窗跟随的按键，默认 f')
    parser.add_argument('--follow-pose-on-start', action='store_true', help='启动后立即开启 /hric/loc/pose 视窗跟随')
    parser.add_argument('--enable-search', action='store_true', help='启用搜索功能：在命令行输入车道ID进行定位')
    # 规划路径话题叠加：默认仅 planning/path；当参数为 dp_path 时额外叠加 planning/dp_path
    parser.add_argument('--planning-path', type=str, default='path', choices=['path', 'dp_path'], help='规划路径叠加：path=仅 planning/path，dp_path=额外叠加 planning/dp_path')
    parser.add_argument('--plot-tra', action='store_true', help='绘制 /hric/nav/plan_trajectory（PlannedTrajectory），并替代 planning/path')
    # QP 路径叠加：开启后额外订阅 planning/qp_path 并叠加可视化
    parser.add_argument('--qp_path', action='store_true', help='叠加可视化 planning/qp_path（QP 路径）')
    # 障碍物可视化源选择（两者都加则同时显示；都不加则默认使用 origin，保持兼容）
    parser.add_argument('--use-origin-obs', action='store_true', help='使用 /perception/objects（ObjectArray）进行障碍物可视化')
    parser.add_argument('--use-converted-obs', action='store_true', help='使用 /perception/converted_obstacles（MarkerArray）进行障碍物可视化')
    # RoutingResponse 相关
    parser.add_argument('--routing-response', type=str, default=None, help='RoutingResponse proto 文本文件路径或包含该文件的目录（*.pb.txt）')
    parser.add_argument('--route-width', type=float, default=5.0, help='路线覆盖线宽')
    parser.add_argument('--route-color', type=str, default='#ff0000', help='路线颜色，格式 #RRGGBB(AA) 或 r,g,b(,a)')
    parser.add_argument('--boundary-line-width', type=float, default=0.6, help='左右边界虚线线宽')
    parser.add_argument('--boundary-alpha', type=float, default=0.85, help='左右边界虚线透明度（0-1）')
    parser.add_argument(
        '--speed-delta', type=float, default=REMOTE_SPEED_DEFAULT_DELTA,
        help=f"远程调速每次步长（m/s），按 '.'/'>' 为 +delta，按 ','/'<' 为 -delta（默认 {REMOTE_SPEED_DEFAULT_DELTA}）",
    )
    parser.add_argument(
        '--nav-target-x', type=float, default=START_NAV_DEFAULT_X,
        help=f"按 '{START_NAV_KEY}' 下发的导航目标点 x（map 坐标系，默认 {START_NAV_DEFAULT_X}）",
    )
    parser.add_argument(
        '--nav-target-y', type=float, default=START_NAV_DEFAULT_Y,
        help=f"按 '{START_NAV_KEY}' 下发的导航目标点 y（map 坐标系，默认 {START_NAV_DEFAULT_Y}）",
    )

    args = parser.parse_args()

    bbox: Optional[Tuple[float, float, float, float]] = None
    if args.bbox is not None:
        bbox = (args.bbox[0], args.bbox[1], args.bbox[2], args.bbox[3])

    # 中文字体：fonts/ 下自带 Noto（可随仓库拷贝到离线机）；显式路径/环境变量仍可覆盖
    _cjk_fpath = resolve_ui_cjk_font_path(args.font_name, args.cjk_font_file)
    if _cjk_fpath:
        print(f"UI 中文字体: {_cjk_fpath}")
        try:
            fp0 = font_manager.FontProperties(fname=_cjk_fpath)
            fam = fp0.get_name()
            matplotlib.rcParams['font.family'] = ['sans-serif']
            base_ss = list(matplotlib.rcParams.get('font.sans-serif', []))
            dedup: List[str] = [fam] + [x for x in base_ss if x != fam]
            matplotlib.rcParams['font.sans-serif'] = dedup
            print(f"matplotlib 默认中文族: {fam}")
        except Exception:
            pass
    else:
        chosen_font = choose_chinese_font(args.font_name)
        if chosen_font:
            matplotlib.rcParams['font.family'] = ['sans-serif']
            matplotlib.rcParams['font.sans-serif'] = [chosen_font]
            print(f"使用中文字体: {chosen_font}")
        else:
            print(
                "警告: 未找到中文字体。请将 .otf/.ttf 放入本脚本同目录 fonts/，或使用 "
                "--cjk-font-file、环境变量 VISUAL_TOOL_CJK_FONT、--font-name。"
            )

    map_dir = os.path.abspath(args.map_dir)
    routing_map_path = os.path.join(map_dir, 'routing_map.txt')
    base_map_path = os.path.join(map_dir, 'base_map.txt')

    if not os.path.isdir(map_dir):
        raise FileNotFoundError(f"地图目录不存在: {map_dir}")
    if not os.path.exists(routing_map_path):
        raise FileNotFoundError(f"未找到 routing_map.txt: {routing_map_path}")
    if not os.path.exists(base_map_path):
        raise FileNotFoundError(f"未找到 base_map.txt: {base_map_path}")

    print(f"地图目录: {map_dir}")
    print(f"读取 routing_map: {routing_map_path}")
    print(f"读取 base_map: {base_map_path}")
    nodes, edges, edges_typed = parse_routing_map(routing_map_path, bbox=bbox, max_nodes=args.max_nodes)
    print(f"节点数: {len(nodes)}，边数: {len(edges)}（已按 bbox 过滤）")

    lane_collection, edge_collection, lane_ids_in_order = build_collections(
        nodes, edges, color_by=args.color_by, draw_edges=args.draw_edges, max_edges=args.max_edges
    )

    fig, ax = plt.subplots(figsize=(args.figsize[0], args.figsize[1]))
    ax.add_collection(lane_collection)
    if edge_collection is not None:
        ax.add_collection(edge_collection)
    # 左右边界（基于 base_map 的 left_sample / right_sample）
    try:
        lane_samples = parse_lane_samples_from_base_map(base_map_path)
        left_boundary_collection, right_boundary_collection, left_count, right_count = build_boundary_collections_from_samples(
            nodes,
            lane_ids_in_order,
            lane_samples,
            line_width=max(0.1, float(args.boundary_line_width)),
            alpha=max(0.0, min(1.0, float(args.boundary_alpha))),
        )
        if left_boundary_collection is not None:
            ax.add_collection(left_boundary_collection)
        if right_boundary_collection is not None:
            ax.add_collection(right_boundary_collection)
        print(
            f"左右边界可视化: 左 {left_count} 条，右 {right_count} 条（源: {base_map_path}）"
        )
    except Exception as e:
        print(f"警告: 左右边界解析失败，已跳过。原因: {e}")

    # supply_station 可视化（黄色，带三角标记）
    try:
        supply_stations = parse_supply_stations_from_base_map(base_map_path)
        _ss_color = '#e6b800'
        for ss in supply_stations:
            if ss.entry_start and ss.entry_end:
                ax.plot(
                    [ss.entry_start[0], ss.entry_end[0]],
                    [ss.entry_start[1], ss.entry_end[1]],
                    color=_ss_color, linewidth=2.5, solid_capstyle='round', zorder=8,
                )
                mx = (ss.entry_start[0] + ss.entry_end[0]) * 0.5
                my = (ss.entry_start[1] + ss.entry_end[1]) * 0.5
                ax.plot(mx, my, marker='>', color=_ss_color, markersize=6, zorder=9)
            if ss.exit_start and ss.exit_end:
                ax.plot(
                    [ss.exit_start[0], ss.exit_end[0]],
                    [ss.exit_start[1], ss.exit_end[1]],
                    color=_ss_color, linewidth=2.5, solid_capstyle='round', zorder=8,
                )
                mx = (ss.exit_start[0] + ss.exit_end[0]) * 0.5
                my = (ss.exit_start[1] + ss.exit_end[1]) * 0.5
                ax.plot(mx, my, marker='<', color=_ss_color, markersize=6, zorder=9)
        if supply_stations:
            print(f"补给站可视化: {len(supply_stations)} 个 supply_station")
    except Exception as e:
        print(f"警告: supply_station 解析失败，已跳过。原因: {e}")

    # race_line 可视化（红色，带三角标记）
    try:
        race_lines = parse_race_lines_from_base_map(base_map_path)
        _rl_color = '#d62728'
        for rl in race_lines:
            if rl.start_point and rl.end_point:
                ax.plot(
                    [rl.start_point[0], rl.end_point[0]],
                    [rl.start_point[1], rl.end_point[1]],
                    color=_rl_color, linewidth=3.0, solid_capstyle='round', zorder=8,
                )
                mx = (rl.start_point[0] + rl.end_point[0]) * 0.5
                my = (rl.start_point[1] + rl.end_point[1]) * 0.5
                ax.plot(mx, my, marker='^', color=_rl_color, markersize=7, zorder=9)
        if race_lines:
            print(f"赛道线可视化: {len(race_lines)} 条 race_line")
    except Exception as e:
        print(f"警告: race_line 解析失败，已跳过。原因: {e}")

    # 设定显示范围
    if bbox is not None:
        ax.set_xlim(bbox[0], bbox[2])
        ax.set_ylim(bbox[1], bbox[3])
    else:
        inferred = auto_bbox_from_nodes(nodes)
        if inferred is not None:
            pad_x = (inferred[2] - inferred[0]) * 0.02
            pad_y = (inferred[3] - inferred[1]) * 0.02
            ax.set_xlim(inferred[0] - pad_x, inferred[2] + pad_x)
            ax.set_ylim(inferred[1] - pad_y, inferred[3] + pad_y)

    ax.set_aspect('equal', adjustable='box')
    base_title = '路网中心线实时可视化'
    _title_fp_main: Optional[font_manager.FontProperties] = None
    if _cjk_fpath:
        _title_fp_main = font_manager.FontProperties(fname=_cjk_fpath)
        _title_fp_main.set_size(_rc_font_size_to_points('axes.titlesize', 'font.size', default_pt=12.0))
    if _title_fp_main is not None:
        ax.set_title(base_title, fontproperties=_title_fp_main)
    else:
        ax.set_title(base_title)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, linewidth=0.3, alpha=0.3)

    # RoutingResponse 覆盖层（若提供）
    route_collection_static: Optional[LineCollection] = None
    route_start_marker = None
    route_end_marker = None
    route_waypoint_markers = []
    
    if args.routing_response:
        rr_path = _find_routing_response_file(args.routing_response)
        if rr_path is None:
            print(f"警告: 未找到 RoutingResponse 文件于: {args.routing_response}")
        else:
            print(f"读取 RoutingResponse: {rr_path}")
            # 详细解析（包含 start_s/end_s），并派生 id 列表用于覆盖层
            route_segments_info = parse_routing_response_segments_detailed(rr_path)
            route_ids = [sid for (sid, _ss, _es) in route_segments_info]
            print(f"RoutingResponse 中提取 segment 数量: {len(route_ids)}")
            # 默认使用较低透明度，避免覆盖底图颜色
            color_rgba = _parse_color_to_rgba(args.route_color, default=(1.0, 0.0, 0.0, 0.4))
            route_collection_static, found_ids, missing_ids = build_route_overlay_collection(
                nodes, route_ids, color_rgba=color_rgba, linewidth=args.route_width
            )
            if route_collection_static is not None:
                ax.add_collection(route_collection_static)
            print(f"可视化路线段: {len(found_ids)}，缺失: {len(missing_ids)}")
            if missing_ids:
                # 仅打印前若干以免过长
                preview = ', '.join(missing_ids[:8])
                more = '' if len(missing_ids) <= 8 else f' 等 {len(missing_ids)} 段'
                print(f"提示: 下列路线段在节点集中未找到（可能被 bbox/max-nodes 过滤）: {preview}{more}")
            # 绘制起终点标记（不裁剪 polyline，仅在 start_s / end_s 位置打标）
            if route_segments_info:
                # 起点
                first_id, first_start_s, _ = route_segments_info[0]
                start_lane = nodes.get(first_id)
                if start_lane and start_lane.points:
                    sx = 0.0 if first_start_s is None else max(0.0, first_start_s)
                    sp = _interpolate_point_on_polyline(start_lane.points, sx)
                    if sp is not None:
                        try:
                            route_start_marker = ax.scatter([sp[0]], [sp[1]], marker='*', s=120.0, c=[(1.0, 0.84, 0.0, 0.95)], edgecolors=(0.9, 0.6, 0.0, 0.95), linewidths=0.6, zorder=14)
                        except Exception:
                            route_start_marker = ax.scatter([sp[0]], [sp[1]], marker='*', s=120.0, c=[(1.0, 0.84, 0.0, 0.95)], zorder=14)
                else:
                    print(f"起点所在车道缺失: {first_id}")
                # 终点
                last_id, _ls, last_end_s = route_segments_info[-1]
                end_lane = nodes.get(last_id)
                if end_lane and end_lane.points:
                    # 若 end_s 缺失，则用该车道末端
                    # 估算折线总长以便 clamp
                    total_len = 0.0
                    for i in range(len(end_lane.points) - 1):
                        x1, y1 = end_lane.points[i]
                        x2, y2 = end_lane.points[i + 1]
                        total_len += math.hypot(x2 - x1, y2 - y1)
                    ex = total_len if last_end_s is None else max(0.0, min(total_len, last_end_s))
                    ep = _interpolate_point_on_polyline(end_lane.points, ex)
                    if ep is not None:
                        try:
                            route_end_marker = ax.scatter([ep[0]], [ep[1]], marker='^', s=120.0, c=[(1.0, 0.15, 0.15, 0.95)], edgecolors=(0.6, 0.0, 0.0, 0.95), linewidths=0.6, zorder=14)
                        except Exception:
                            route_end_marker = ax.scatter([ep[0]], [ep[1]], marker='^', s=120.0, c=[(1.0, 0.15, 0.15, 0.95)], zorder=14)
                else:
                    print(f"终点所在车道缺失: {last_id}")
            # 绘制 routing_request.waypoint（若存在，按首尾与中间不同样式）
            waypoints = parse_routing_response_waypoints(rr_path)
            if waypoints and len(waypoints) >= 2:
                def waypoint_xy(wp: Tuple[Optional[str], Optional[float], Optional[float], Optional[float]]) -> Optional[Tuple[float, float]]:
                    lid, s_val, px, py = wp
                    if lid and lid in nodes and nodes[lid].points:
                        # 若提供 s 则按弧长定位，否则取 lane 起点
                        if s_val is None:
                            return nodes[lid].points[0]
                        # clamp 在 lane 总长内
                        pts = nodes[lid].points
                        total = 0.0
                        for i in range(len(pts) - 1):
                            x1, y1 = pts[i]
                            x2, y2 = pts[i + 1]
                            total += math.hypot(x2 - x1, y2 - y1)
                        s_clamped = max(0.0, min(total, s_val))
                        return _interpolate_point_on_polyline(pts, s_clamped)
                    # 回退使用 pose
                    if px is not None and py is not None:
                        return (px, py)
                    return None
                # 起点（五角星，金色）
                start_xy = waypoint_xy(waypoints[0])
                if start_xy is not None:
                    try:
                        route_waypoint_markers.append(ax.scatter([start_xy[0]], [start_xy[1]], marker='*', s=130.0, c=[(1.0, 0.84, 0.0, 0.95)], edgecolors=(0.9, 0.6, 0.0, 0.95), linewidths=0.6, zorder=15))
                    except Exception:
                        route_waypoint_markers.append(ax.scatter([start_xy[0]], [start_xy[1]], marker='*', s=130.0, c=[(1.0, 0.84, 0.0, 0.95)], zorder=15))
                # 中间途经点（黄色三角）
                if len(waypoints) > 2:
                    mid_xs: List[float] = []
                    mid_ys: List[float] = []
                    for wp in waypoints[1:-1]:
                        p = waypoint_xy(wp)
                        if p is not None:
                            mid_xs.append(p[0])
                            mid_ys.append(p[1])
                    if mid_xs:
                        try:
                            route_waypoint_markers.append(ax.scatter(mid_xs, mid_ys, marker='^', s=110.0, c=[(1.0, 0.9, 0.2, 0.95)], edgecolors=(0.9, 0.7, 0.0, 0.95), linewidths=0.6, zorder=15))
                        except Exception:
                            route_waypoint_markers.append(ax.scatter(mid_xs, mid_ys, marker='^', s=110.0, c=[(1.0, 0.9, 0.2, 0.95)], zorder=15))
                # 终点（红色三角）
                end_xy = waypoint_xy(waypoints[-1])
                if end_xy is not None:
                    try:
                        route_waypoint_markers.append(ax.scatter([end_xy[0]], [end_xy[1]], marker='^', s=130.0, c=[(1.0, 0.15, 0.15, 0.95)], edgecolors=(0.6, 0.0, 0.0, 0.95), linewidths=0.6, zorder=15))
                    except Exception:
                        route_waypoint_markers.append(ax.scatter([end_xy[0]], [end_xy[1]], marker='^', s=130.0, c=[(1.0, 0.15, 0.15, 0.95)], zorder=15))

    # 初始化ROS2
    rclpy.init()
    
    # 共享数据结构和锁
    data_lock = threading.Lock()
    shared_data = {
        'robot_pose': None,
        'lookahead_pose': None,
        'refline': None,
        'planning_path': None,
        'planned_trajectory': None,  # List[(x, y, speed)]
        'dp_path': None,
        'qp_path': None,
        'perception_objects_polys': None,  # List[List[(x,y)]]
        'perception_objects_meta': None,   # List[Dict]
        'converted_obstacle_markers': None,  # List[visualization_msgs/Marker]
        'battery_power_pct': None,  # master_battery_power -> 电量 %
        'rtk_gps_speed_mps': None,  # RtkGps.gprmc_speed (m/s)
        'leg_motor_overlay_lines': None,  # List[List[Tuple[str,str,bool]]] 每行多段
        'arm_motor_overlay_lines': None,  # 同上，手臂关节；与腿一同接在首行之后
        'remote_service_result_queue': [],  # 累积的服务调用结果 dict
        'pending_remote_calls': {},  # request_id -> tag（'speed' / 'start_nav'），区分 toast 文案
    }
    
    # 若用户未指定任何障碍物来源，默认沿用历史行为：使用 /perception/objects
    use_origin_obs = bool(args.use_origin_obs)
    use_converted_obs = bool(args.use_converted_obs)
    if (not use_origin_obs) and (not use_converted_obs):
        use_origin_obs = True

    # 创建ROS2节点
    enable_dp_path = bool(args.planning_path == 'dp_path')
    enable_qp_path = bool(args.qp_path)
    enable_plot_tra = bool(args.plot_tra)
    ros_node = ROS2SubscriberNode(
        data_lock,
        shared_data,
         enable_dp_path=enable_dp_path,
         enable_qp_path=enable_qp_path,
        enable_plot_tra=enable_plot_tra,
        use_origin_obs=use_origin_obs,
        use_converted_obs=use_converted_obs,
    )
    
    # 在单独线程中运行ROS2执行器
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    def ros_spin():
        """在后台线程中运行ROS2执行器。"""
        executor.spin()
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 实时可视化元素
    robot_arrow: Optional[FancyArrowPatch] = None
    lookahead_arrow: Optional[FancyArrowPatch] = None
    refline_collection: Optional[LineCollection] = None
    planning_path_collection: Optional[LineCollection] = None
    planned_trajectory_collection: Optional[LineCollection] = None
    dp_path_collection: Optional[LineCollection] = None
    qp_path_collection: Optional[LineCollection] = None
    perception_object_patches: List[Polygon] = []
    converted_obstacle_artists: List = []
    hover_lane_id: Optional[str] = None
    # 远程服务调用 toast：单槽，后到覆盖前者
    toast_state = {'expire_ts': 0.0}
    toast_artist = fig.text(
        0.5, 0.96, '', ha='center', va='top',
        fontsize=14, color='#1a1a1a',
        bbox=dict(facecolor='#fff9c4', edgecolor='#999999', alpha=0.92,
                  boxstyle='round,pad=0.45'),
        visible=False, zorder=100,
    )

    def _show_toast(text: str, color: str = '#1a1a1a',
                    bg: str = '#fff9c4', duration: float = 2.5) -> None:
        toast_state['expire_ts'] = time.monotonic() + duration
        toast_artist.set_text(text)
        toast_artist.set_color(color)
        try:
            toast_artist.get_bbox_patch().set_facecolor(bg)
        except Exception:
            pass
        toast_artist.set_visible(True)
        fig.canvas.draw_idle()

    follow_window_size = max(1.0, float(args.follow_pose_window))
    follow_pose_key = (args.follow_pose_key or 'f').strip().lower() or 'f'
    follow_pose_enabled = bool(args.follow_pose_on_start)
    follow_pose_last_center: Optional[Tuple[float, float]] = None
    view_before_follow: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None

    def _set_follow_window_center(cx: float, cy: float) -> None:
        half = 0.5 * follow_window_size
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

    def _update_title() -> None:
        title = base_title
        if hover_lane_id:
            title += f' - hover: {hover_lane_id}'
        if follow_pose_enabled:
            title += f' | 跟随 /hric/loc/pose ({follow_window_size:.1f}m x {follow_window_size:.1f}m)'
        if _title_fp_main is not None:
            ax.set_title(title, fontproperties=_title_fp_main)
        else:
            ax.set_title(title)

    _update_title()

    _overlay_font_fp: Optional[font_manager.FontProperties] = None
    _overlay_font_fp_bold: Optional[font_manager.FontProperties] = None
    if _cjk_fpath:
        _overlay_font_fp = font_manager.FontProperties(fname=_cjk_fpath, size=STATUS_PANEL_FONT_SIZE)
        try:
            _overlay_font_fp_bold = font_manager.FontProperties(
                fname=_cjk_fpath,
                size=STATUS_PANEL_FONT_SIZE,
                weight='bold',
            )
        except Exception:
            _overlay_font_fp_bold = font_manager.FontProperties(fname=_cjk_fpath)
            _overlay_font_fp_bold.set_size(STATUS_PANEL_FONT_SIZE)
            try:
                _overlay_font_fp_bold.set_weight('bold')
            except Exception:
                pass
    else:
        print(
            "警告: 未解析到中文字体文件，状态面板与标题可能显示为方框。"
            "请将 Noto Sans SC 等 otf/ttf 放入本脚本同目录的 fonts/，或设置 --cjk-font-file / 环境变量 VISUAL_TOOL_CJK_FONT。"
        )

    status_panel_bg: Optional[Rectangle] = None
    status_panel_texts: List = []
    status_panel_last_sig: Optional[Tuple[Tuple[Tuple[str, str, bool], ...], ...]] = None
    status_panel_row_shape: Optional[Tuple[int, ...]] = None

    def redraw_status_panel(rows: List[List[Tuple[str, str, bool]]]) -> None:
        nonlocal status_panel_bg, status_panel_last_sig, status_panel_row_shape

        # 字号变大时同步加大行距，避免上下文字重叠
        line_dy = 0.021 * (STATUS_PANEL_FONT_SIZE / 8.0)
        pad_r_axes = 0.006
        pad_t_axes = 0.004
        x_right = 1.0 - pad_r_axes
        fig = ax.figure
        y_measure_base = -0.08

        def _rows_sig(r: List[List[Tuple[str, str, bool]]]) -> Tuple[Tuple[Tuple[str, str, bool], ...], ...]:
            return tuple(tuple(segs) for segs in r)

        def _tear_down_status_artists() -> None:
            nonlocal status_panel_bg
            for t in status_panel_texts:
                try:
                    t.remove()
                except Exception:
                    pass
            status_panel_texts.clear()
            if status_panel_bg is not None:
                try:
                    status_panel_bg.remove()
                except Exception:
                    pass
                status_panel_bg = None

        if not rows:
            _tear_down_status_artists()
            status_panel_last_sig = None
            status_panel_row_shape = None
            return

        sig = _rows_sig(rows)
        if status_panel_last_sig is not None and sig == status_panel_last_sig and status_panel_bg is not None:
            return

        row_shape = tuple(len(r) for r in rows)
        n_seg = sum(row_shape)
        use_incremental = (
            status_panel_bg is not None
            and status_panel_row_shape == row_shape
            and len(status_panel_texts) == n_seg
        )

        text_kw_base: Dict = dict(
            transform=ax.transAxes,
            ha='right',
            va='top',
            fontsize=STATUS_PANEL_FONT_SIZE,
            zorder=100,
            clip_on=False,
        )

        def _measure_ws_per_row(rows_m: List[List[Tuple[str, str, bool]]]) -> List[List[float]]:
            all_measure: List = []
            row_seg_counts: List[int] = []
            y_step = 0.035 * (STATUS_PANEL_FONT_SIZE / 8.0)
            for ri, row_segs in enumerate(rows_m):
                seg_list = list(row_segs)
                row_seg_counts.append(len(seg_list))
                x_m = 0.0
                y_m = y_measure_base - ri * y_step
                for seg_text, _c, bold in seg_list:
                    if _overlay_font_fp is not None:
                        mfp = _overlay_font_fp_bold if bold and _overlay_font_fp_bold is not None else _overlay_font_fp
                    else:
                        mfp = None
                    tw: Dict = {
                        'transform': ax.transAxes,
                        'ha': 'left',
                        'va': 'top',
                        'fontsize': STATUS_PANEL_FONT_SIZE,
                        'zorder': 0,
                        'clip_on': False,
                        'alpha': 1.0,
                        'color': '#e8e8e8',
                    }
                    if mfp is not None:
                        tw['fontproperties'] = mfp
                    else:
                        tw['fontweight'] = 'bold' if bold else 'normal'
                    tm = ax.text(x_m, y_m, seg_text, **tw)
                    all_measure.append(tm)
                    x_m += 0.42
            fig.canvas.draw()
            renderer = fig.canvas.get_renderer()
            axbb = ax.get_window_extent(renderer)
            widths = [t.get_window_extent(renderer).width / max(axbb.width, 1e-9) for t in all_measure]
            w_i = 0
            ws_out: List[List[float]] = []
            for cnt in row_seg_counts:
                ws_out.append(widths[w_i : w_i + cnt])
                w_i += cnt
            for tm in all_measure:
                try:
                    tm.remove()
                except Exception:
                    pass
            return ws_out

        def _bg_geom(ws_per_row_inner: List[List[float]], y_top_axes: float) -> Tuple[float, float, float, float]:
            max_bw, max_bh = 0.48, 0.45
            max_row_w_axes = max((sum(ws) for ws in ws_per_row_inner), default=0.06)
            pad_w = 0.014
            pad_h = 0.012
            bg_w = min(max_row_w_axes + pad_w, max_bw)
            bg_h = min(len(rows) * line_dy + pad_h, max_bh)
            bg_x1 = 1.0 - 0.003
            bg_y1 = min(1.0, y_top_axes + 0.014)
            bg_x0 = max(0.0, bg_x1 - bg_w)
            bg_y0 = max(0.0, bg_y1 - bg_h)
            return bg_x0, bg_y0, bg_w, bg_h

        if use_incremental:
            try:
                y = 0.98 - pad_t_axes
                y_start = y
                ws_per_row = _measure_ws_per_row(rows)
                flat_i = 0
                for ri, row_segs in enumerate(rows):
                    seg_list = list(row_segs)
                    x_r = x_right
                    ws = ws_per_row[ri]
                    for (seg_text, color, bold), w in zip(reversed(seg_list), reversed(ws)):
                        t = status_panel_texts[flat_i]
                        t.set_text(seg_text)
                        t.set_color(color)
                        if _overlay_font_fp is not None:
                            t.set_fontproperties(
                                _overlay_font_fp_bold if bold and _overlay_font_fp_bold is not None else _overlay_font_fp
                            )
                        else:
                            t.set_fontweight('bold' if bold else 'normal')
                        t.set_position((x_r, y))
                        flat_i += 1
                        x_r -= w
                    y -= line_dy
                bg_x0, bg_y0, bg_w, bg_h = _bg_geom(ws_per_row, y_start)
                status_panel_bg.set_xy((bg_x0, bg_y0))
                status_panel_bg.set_width(bg_w)
                status_panel_bg.set_height(bg_h)
                status_panel_last_sig = sig
                return
            except Exception:
                traceback.print_exc()
                _tear_down_status_artists()
                status_panel_row_shape = None
                use_incremental = False

        if not use_incremental:
            _tear_down_status_artists()
            y = 0.98 - pad_t_axes
            try:
                ws_per_row = _measure_ws_per_row(rows)
                y_start = y
                for ri, row_segs in enumerate(rows):
                    seg_list = list(row_segs)
                    x_r = x_right
                    ws = ws_per_row[ri]
                    for (seg_text, color, bold), w in zip(reversed(seg_list), reversed(ws)):
                        kw = {**text_kw_base}
                        kw['color'] = color
                        if _overlay_font_fp is not None:
                            kw['fontproperties'] = (
                                _overlay_font_fp_bold if bold and _overlay_font_fp_bold is not None else _overlay_font_fp
                            )
                        else:
                            kw['fontweight'] = 'bold' if bold else 'normal'
                        txt = ax.text(x_r, y, seg_text, **kw)
                        status_panel_texts.append(txt)
                        x_r -= w
                    y -= line_dy

                bg_x0, bg_y0, bg_w, bg_h = _bg_geom(ws_per_row, y_start)
                status_panel_bg = Rectangle(
                    (bg_x0, bg_y0),
                    bg_w,
                    bg_h,
                    transform=ax.transAxes,
                    facecolor='white',
                    edgecolor='0.72',
                    linewidth=0.7,
                    alpha=0.92,
                    zorder=99,
                    clip_on=False,
                )
                ax.add_patch(status_panel_bg)
                status_panel_bg.set_zorder(99)
                for t in status_panel_texts:
                    t.set_zorder(100)
                status_panel_last_sig = sig
                status_panel_row_shape = row_shape
            except Exception:
                traceback.print_exc()
                _tear_down_status_artists()
                status_panel_row_shape = None
                max_chars = max((len(''.join(seg[0] for seg in r)) for r in rows), default=10)
                bg_w = min(0.4, 0.03 + 0.0065 * min(max_chars, 48))
                bg_h = len(rows) * line_dy + 0.016
                bg_x0 = 1.0 - pad_r_axes - bg_w
                bg_y0 = 0.98 - pad_t_axes - bg_h + 0.002
                status_panel_bg = Rectangle(
                    (bg_x0, bg_y0),
                    bg_w,
                    bg_h,
                    transform=ax.transAxes,
                    facecolor='white',
                    edgecolor='0.72',
                    linewidth=0.7,
                    alpha=0.92,
                    zorder=99,
                    clip_on=False,
                )
                ax.add_patch(status_panel_bg)
                y_fb = 0.98 - pad_t_axes
                for row_segs in rows:
                    line = ''.join(s[0] for s in row_segs)
                    fb_kw: Dict = {
                        'transform': ax.transAxes,
                        'ha': 'right',
                        'va': 'top',
                        'fontsize': STATUS_PANEL_FONT_SIZE,
                        'color': _OVERLAY_NORMAL_COLOR,
                        'zorder': 100,
                        'clip_on': False,
                    }
                    if _overlay_font_fp is not None:
                        fb_kw['fontproperties'] = _overlay_font_fp
                    txt_fb = ax.text(x_right, y_fb, line, **fb_kw)
                    status_panel_texts.append(txt_fb)
                    y_fb -= line_dy
                status_panel_last_sig = sig

    def update_visualization():
        """更新可视化（10Hz调用）。"""
        nonlocal robot_arrow, lookahead_arrow, refline_collection, planning_path_collection, planned_trajectory_collection, dp_path_collection, qp_path_collection, perception_object_patches, converted_obstacle_artists, follow_pose_last_center
        
        with data_lock:
            # 更新机器人位置箭头
            if shared_data['robot_pose'] is not None:
                pose = shared_data['robot_pose']
                x, y, yaw = pose['x'], pose['y'], pose['yaw']
                
                # 移除旧的箭头
                if robot_arrow is not None:
                    try:
                        robot_arrow.remove()
                    except Exception:
                        pass
                    robot_arrow = None
                
                # 计算箭头终点（长度约2米）
                arrow_length = 2.0
                end_x = x + arrow_length * math.cos(yaw)
                end_y = y + arrow_length * math.sin(yaw)
                
                # 创建黄色箭头
                robot_arrow = FancyArrowPatch(
                    (x, y), (end_x, end_y),
                    arrowstyle='->',
                    mutation_scale=20.0,
                    color=(1.0, 0.84, 0.0, 0.95),  # 黄色
                    linewidth=3.0,
                    zorder=20
                )
                ax.add_patch(robot_arrow)
            else:
                # 如果没有数据，清除箭头
                if robot_arrow is not None:
                    try:
                        robot_arrow.remove()
                    except Exception:
                        pass
                    robot_arrow = None

            # 更新前视点箭头（绿色）
            if shared_data['lookahead_pose'] is not None:
                pose = shared_data['lookahead_pose']
                x, y, yaw = pose['x'], pose['y'], pose['yaw']

                if lookahead_arrow is not None:
                    try:
                        lookahead_arrow.remove()
                    except Exception:
                        pass
                    lookahead_arrow = None

                arrow_length = 2.0
                end_x = x + arrow_length * math.cos(yaw)
                end_y = y + arrow_length * math.sin(yaw)
                lookahead_arrow = FancyArrowPatch(
                    (x, y), (end_x, end_y),
                    arrowstyle='->',
                    mutation_scale=18.0,
                    color=(0.0, 0.9, 0.0, 0.95),
                    linewidth=2.5,
                    zorder=19
                )
                ax.add_patch(lookahead_arrow)
            else:
                if lookahead_arrow is not None:
                    try:
                        lookahead_arrow.remove()
                    except Exception:
                        pass
                    lookahead_arrow = None
            
            # 更新参考线（紫色，较宽，透明度50%）
            if shared_data['refline'] is not None and len(shared_data['refline']) >= 2:
                refline_points = shared_data['refline']
                # 移除旧的参考线
                if refline_collection is not None:
                    try:
                        refline_collection.remove()
                    except Exception:
                        pass
                    refline_collection = None
                
                # 创建新的参考线集合
                refline_collection = LineCollection(
                    [refline_points],
                    colors=[(0.5, 0.5, 0.5, 0.5)],  # 灰色，透明度50%
                    linewidths=4.0,  # 较宽
                    zorder=15
                )
                ax.add_collection(refline_collection)
            else:
                # 如果没有数据，清除参考线
                if refline_collection is not None:
                    try:
                        refline_collection.remove()
                    except Exception:
                        pass
                    refline_collection = None
            
            # 更新规划路径或规划轨迹
            if enable_plot_tra:
                # 清除 planning/path 的可视化
                if planning_path_collection is not None:
                    try:
                        planning_path_collection.remove()
                    except Exception:
                        pass
                    planning_path_collection = None

                traj_points = shared_data.get('planned_trajectory')
                if traj_points is not None and len(traj_points) >= 2:
                    if planned_trajectory_collection is not None:
                        try:
                            planned_trajectory_collection.remove()
                        except Exception:
                            pass
                        planned_trajectory_collection = None

                    segments: List[List[Tuple[float, float]]] = []
                    colors: List[Tuple[float, float, float, float]] = []
                    for i in range(len(traj_points) - 1):
                        x1, y1, s1 = traj_points[i]
                        x2, y2, s2 = traj_points[i + 1]
                        segments.append([(x1, y1), (x2, y2)])
                        speed = 0.5 * (s1 + s2)
                        r, g, b, a = _speed_to_rgba(speed)
                        colors.append((r, g, b, a))

                    planned_trajectory_collection = LineCollection(
                        segments,
                        colors=colors,
                        linewidths=4.0,
                        zorder=16
                    )
                    ax.add_collection(planned_trajectory_collection)
                else:
                    if planned_trajectory_collection is not None:
                        try:
                            planned_trajectory_collection.remove()
                        except Exception:
                            pass
                        planned_trajectory_collection = None
            else:
                # 使用传统 planning/path
                if planned_trajectory_collection is not None:
                    try:
                        planned_trajectory_collection.remove()
                    except Exception:
                        pass
                    planned_trajectory_collection = None

                if shared_data['planning_path'] is not None and len(shared_data['planning_path']) >= 2:
                    planning_points = shared_data['planning_path']
                    if planning_path_collection is not None:
                        try:
                            planning_path_collection.remove()
                        except Exception:
                            pass
                        planning_path_collection = None

                    planning_path_collection = LineCollection(
                        [planning_points],
                        colors=[(0.0, 0.8, 0.0, 0.9)],  # 绿色
                        linewidths=1.5,  # 细线
                        zorder=16
                    )
                    ax.add_collection(planning_path_collection)
                else:
                    if planning_path_collection is not None:
                        try:
                            planning_path_collection.remove()
                        except Exception:
                            pass
                        planning_path_collection = None

            # 更新 DP 路径（蓝色，细线）
            if shared_data.get('dp_path') is not None and len(shared_data.get('dp_path')) >= 2:
                dp_points = shared_data.get('dp_path')
                if dp_path_collection is not None:
                    try:
                        dp_path_collection.remove()
                    except Exception:
                        pass
                    dp_path_collection = None
                dp_path_collection = LineCollection(
                    [dp_points],
                    colors=[(0.1, 0.35, 1.0, 0.5)],  # 蓝色，50% 透明
                    linewidths=4.0,  # 加粗但不过分遮挡
                    linestyles='dashed',  # 虚线：即使两条路径重合也能区分
                    zorder=19  # 放到更上层，避免被其它线覆盖
                )
                ax.add_collection(dp_path_collection)
            else:
                if dp_path_collection is not None:
                    try:
                        dp_path_collection.remove()
                    except Exception:
                        pass
                    dp_path_collection = None

            # 更新 QP 路径（橙色系，50% 透明，虚线，线宽 4.0）
            if shared_data.get('qp_path') is not None and len(shared_data.get('qp_path')) >= 2:
                qp_points = shared_data.get('qp_path')
                if qp_path_collection is not None:
                    try:
                        qp_path_collection.remove()
                    except Exception:
                        pass
                    qp_path_collection = None
                qp_path_collection = LineCollection(
                    [qp_points],
                    colors=[(1.0, 0.55, 0.0, 0.5)],  # 橙色，50% 透明（与 dp 的蓝色区分）
                    linewidths=4.0,
                    linestyles='dashed',
                    zorder=19
                )
                ax.add_collection(qp_path_collection)
            else:
                if qp_path_collection is not None:
                    try:
                        qp_path_collection.remove()
                    except Exception:
                        pass
                    qp_path_collection = None

            # 更新感知障碍物（多边形轮廓）
            # 先清理旧的 patch，避免累积
            if perception_object_patches:
                for p in perception_object_patches:
                    try:
                        p.remove()
                    except Exception:
                        pass
                perception_object_patches = []

            polys = shared_data.get('perception_objects_polys', None)
            metas = shared_data.get('perception_objects_meta', None)
            if polys is not None and len(polys) > 0:
                # 简单配色：半透明绿色填充 + 绿色边框
                face = (0.10, 0.80, 0.20, 0.18)
                edge = (0.05, 0.65, 0.15, 0.85)
                for i, poly_xy in enumerate(polys):
                    if not poly_xy or len(poly_xy) < 3:
                        continue
                    patch = Polygon(
                        poly_xy,
                        closed=True,
                        facecolor=face,
                        edgecolor=edge,
                        linewidth=1.2,
                        zorder=18
                    )
                    ax.add_patch(patch)
                    perception_object_patches.append(patch)
                    # 可选：绘制一个很轻量的文本（默认关掉，避免太卡）
                    # if metas and i < len(metas):
                    #     mid = poly_xy[len(poly_xy)//2]
                    #     ax.text(mid[0], mid[1], str(metas[i].get('id','')), fontsize=7, color=edge, zorder=19)

            # 更新 converted obstacles（MarkerArray）
            # 统一管理 artists：每次更新都移除旧的，再按最新 markers 重建
            if converted_obstacle_artists:
                for art in converted_obstacle_artists:
                    try:
                        art.remove()
                    except Exception:
                        pass
                converted_obstacle_artists = []

            markers = shared_data.get('converted_obstacle_markers', None)
            if markers:
                # converted obstacles 多边形样式：红色粗线条
                polygon_edge = (1.0, 0.0, 0.0, 0.95)
                polygon_linewidth = 2.8
                default_edge = (0.0, 0.8, 0.8, 0.8)
                for m in markers:
                    # 忽略删除动作
                    try:
                        if int(getattr(m, 'action', 0)) in (2, 3):  # DELETE / DELETEALL
                            continue
                    except Exception:
                        pass

                    try:
                        mx = float(m.pose.position.x)
                        my = float(m.pose.position.y)
                        yaw = quaternion_to_yaw(m.pose.orientation)
                    except Exception:
                        mx, my, yaw = 0.0, 0.0, 0.0

                    rgba = _marker_rgba(m, default=default_edge)
                    edge = (rgba[0], rgba[1], rgba[2], max(0.15, rgba[3]))
                    face = (rgba[0], rgba[1], rgba[2], min(0.22, rgba[3] * 0.35))

                    mtype = int(getattr(m, 'type', -1))

                    # 1) LINE_STRIP / LINE_LIST：绘制为线
                    if mtype in (Marker.LINE_STRIP, Marker.LINE_LIST):
                        pts = _transform_local_points_2d(getattr(m, 'points', []), mx, my, yaw)
                        if len(pts) < 2:
                            continue
                        lw = polygon_linewidth
                        try:
                            sx = float(m.scale.x)
                            if sx > 0.0:
                                lw = max(polygon_linewidth, sx)
                        except Exception:
                            pass

                        if mtype == Marker.LINE_STRIP:
                            lc = LineCollection([pts], colors=[polygon_edge], linewidths=lw, zorder=18)
                            ax.add_collection(lc)
                            converted_obstacle_artists.append(lc)
                        else:
                            segs: List[List[Tuple[float, float]]] = []
                            for i in range(0, len(pts) - 1, 2):
                                segs.append([pts[i], pts[i + 1]])
                            if segs:
                                lc = LineCollection(segs, colors=[polygon_edge], linewidths=lw, zorder=18)
                                ax.add_collection(lc)
                                converted_obstacle_artists.append(lc)
                        continue

                    # 2) POINTS：散点
                    if mtype == Marker.POINTS:
                        pts = _transform_local_points_2d(getattr(m, 'points', []), mx, my, yaw)
                        if not pts:
                            continue
                        xs = [p[0] for p in pts]
                        ys = [p[1] for p in pts]
                        size = 18.0
                        try:
                            size = max(6.0, float(m.scale.x) * 20.0)
                        except Exception:
                            pass
                        sc = ax.scatter(xs, ys, s=size, c=[edge], zorder=19)
                        converted_obstacle_artists.append(sc)
                        continue

                    # 3) CUBE：按 2D 有向矩形绘制（忽略 z）
                    if mtype == Marker.CUBE:
                        try:
                            sx = float(m.scale.x)
                            sy = float(m.scale.y)
                        except Exception:
                            sx, sy = 0.0, 0.0
                        if sx <= 1e-6 or sy <= 1e-6:
                            continue
                        hx = sx * 0.5
                        hy = sy * 0.5
                        local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
                        cy = math.cos(yaw)
                        syaw = math.sin(yaw)
                        world = []
                        for lx, ly in local:
                            gx = mx + cy * lx - syaw * ly
                            gy = my + syaw * lx + cy * ly
                            world.append((gx, gy))
                        patch = Polygon(
                            world,
                            closed=True,
                            facecolor=(1.0, 0.0, 0.0, 0.0),
                            edgecolor=polygon_edge,
                            linewidth=polygon_linewidth,
                            zorder=18
                        )
                        ax.add_patch(patch)
                        converted_obstacle_artists.append(patch)
                        continue

                    # 4) SPHERE / CYLINDER：按 2D 圆绘制
                    if mtype in (Marker.SPHERE, Marker.CYLINDER):
                        try:
                            r = 0.5 * max(float(m.scale.x), float(m.scale.y))
                        except Exception:
                            r = 0.0
                        if r <= 1e-6:
                            continue
                        circ = Circle((mx, my), radius=r, facecolor=face, edgecolor=edge, linewidth=1.2, zorder=18)
                        ax.add_patch(circ)
                        converted_obstacle_artists.append(circ)
                        continue

                    # 5) TRIANGLE_LIST：按三点一组三角形绘制（通常较密，限制数量）
                    if mtype == Marker.TRIANGLE_LIST:
                        pts = _transform_local_points_2d(getattr(m, 'points', []), mx, my, yaw)
                        if len(pts) < 3:
                            continue
                        tri_count = 0
                        for i in range(0, len(pts) - 2, 3):
                            tri = [pts[i], pts[i + 1], pts[i + 2]]
                            patch = Polygon(
                                tri,
                                closed=True,
                                facecolor=(1.0, 0.0, 0.0, 0.0),
                                edgecolor=polygon_edge,
                                linewidth=polygon_linewidth,
                                zorder=18
                            )
                            ax.add_patch(patch)
                            converted_obstacle_artists.append(patch)
                            tri_count += 1
                            if tri_count >= 200:
                                break
                        continue
                    # 其它类型暂不处理（ARROW、TEXT、MESH 等）

            # 视窗锁定：保持在 /hric/loc/pose 周围固定窗口，并随位姿移动
            if follow_pose_enabled:
                pose = shared_data.get('robot_pose')
                if pose is not None:
                    cx = float(pose['x'])
                    cy = float(pose['y'])
                    # 降低无效 set_xlim/set_ylim 调用，减少刷新抖动
                    if (follow_pose_last_center is None) or (abs(cx - follow_pose_last_center[0]) > 1e-4) or (abs(cy - follow_pose_last_center[1]) > 1e-4):
                        _set_follow_window_center(cx, cy)
                        follow_pose_last_center = (cx, cy)
                else:
                    # 跟随开启但暂无定位数据时，允许恢复后重新居中
                    follow_pose_last_center = None

            spd = shared_data.get('rtk_gps_speed_mps')
            bp = shared_data.get('battery_power_pct')
            leg_ol = shared_data.get('leg_motor_overlay_lines')
            arm_ol = shared_data.get('arm_motor_overlay_lines')

            # 取走远程服务结果队列；顺带解析每条的 tag（speed / start_nav / 未知）
            result_queue = shared_data.get('remote_service_result_queue')
            pending_results: List[Tuple[str, dict]] = []
            if result_queue:
                tag_map = shared_data.get('pending_remote_calls', {})
                for r in result_queue:
                    rid = str(r.get('request_id') or '')
                    tag = tag_map.pop(rid, None) or ''
                    pending_results.append((tag, r))
                result_queue.clear()
        redraw_status_panel(_build_status_overlay_rows(spd, bp, leg_ol, arm_ol))

        # 处理服务调用结果：最新一条决定 toast
        if pending_results:
            tag, last = pending_results[-1]
            success = bool(last.get('success'))
            err = last.get('error')
            resp = last.get('response') or {}

            if tag == 'start_nav':
                # StartNav 响应：success / message / error_code
                inner_ok = True
                inner_msg = ''
                error_code = None
                if isinstance(resp, dict):
                    if 'success' in resp:
                        inner_ok = bool(resp.get('success'))
                    inner_msg = str(resp.get('message', '') or '').strip()
                    error_code = resp.get('error_code')
                if success and inner_ok:
                    _show_toast(
                        "目标点已下发",
                        color='#0b6623', bg='#d6f5d6', duration=2.0,
                    )
                elif success and not inner_ok:
                    detail = inner_msg or f"error_code={error_code}"
                    _show_toast(
                        f"目标点被拒绝: {detail}",
                        color='#b35c00', bg='#ffe9c4', duration=2.5,
                    )
                else:
                    _show_toast(
                        f"目标点下发失败: {err or '未知错误'}",
                        color='#b30000', bg='#ffd6d6', duration=3.5,
                    )
            else:
                # 默认按调速渲染（兼容无 tag 的情况）
                if success:
                    inner_ok = True
                    cruise_speed = None
                    if isinstance(resp, dict):
                        if 'success' in resp:
                            inner_ok = bool(resp.get('success'))
                        cruise_speed = resp.get('updated_cruise_speed')
                    if inner_ok:
                        parts = ["调速成功"]
                        if cruise_speed is not None:
                            try:
                                parts.append(f"当前速度={float(cruise_speed):.2f}")
                            except (TypeError, ValueError):
                                parts.append(f"当前速度={cruise_speed}")
                        _show_toast(
                            " ".join(parts),
                            color='#0b6623', bg='#d6f5d6', duration=2.0,
                        )
                    else:
                        inner_msg = str(resp.get('message', '') or '').strip() if isinstance(resp, dict) else ''
                        _show_toast(
                            f"调速被拒绝 {inner_msg}".strip(),
                            color='#b35c00', bg='#ffe9c4', duration=2.0,
                        )
                else:
                    _show_toast(
                        f"调速失败: {err or '未知错误'}",
                        color='#b30000', bg='#ffd6d6', duration=3.5,
                    )

        # toast 过期隐藏
        if toast_artist.get_visible() and time.monotonic() >= toast_state['expire_ts']:
            toast_artist.set_visible(False)

        # 刷新画布
        fig.canvas.draw_idle()
    
    # 设置10Hz更新定时器（100ms间隔）
    update_timer = fig.canvas.new_timer(interval=100)
    update_timer.add_callback(update_visualization)
    update_timer.start()

    print(f"提示: 按 '{follow_pose_key}' 可切换 /hric/loc/pose 跟随窗口（默认 {follow_window_size:.1f}m x {follow_window_size:.1f}m）")
    print(f"提示: 跟随模式开启后，按 '-' 增大窗口 10m，按 '+' 减小窗口 10m（最小 10m）")
    speed_delta = abs(float(args.speed_delta))
    if speed_delta <= 0.0:
        print(f"警告: --speed-delta={args.speed_delta} 非正，已回退到默认 {REMOTE_SPEED_DEFAULT_DELTA}")
        speed_delta = REMOTE_SPEED_DEFAULT_DELTA
    print(f"提示: 按 '.' 或 '>' 远程加速 +{speed_delta}；按 ',' 或 '<' 减速 -{speed_delta}（调用 {REMOTE_SERVICE_NAME}）")
    nav_target_x = float(args.nav_target_x)
    nav_target_y = float(args.nav_target_y)
    print(f"提示: 按 '{START_NAV_KEY}' 发送导航目标点 ({nav_target_x:.2f}, {nav_target_y:.2f})（调用 {START_NAV_SERVICE_NAME}）")
    if follow_pose_enabled:
        print("提示: 已按参数开启启动即跟随模式。")

    def on_key_press(event):
        nonlocal follow_pose_enabled, follow_pose_last_center, view_before_follow, follow_window_size
        raw_key = event.key or ''
        key = raw_key.strip().lower()
        if not key:
            return

        # 远程服务快捷键：./> 加速，,/< 减速
        # 注意用原始键（不 lower）匹配，避免 '<' '>' 被小写改写（小写是幂等的但显式更清楚）
        if raw_key in REMOTE_SPEED_UP_KEYS or raw_key in REMOTE_SPEED_DOWN_KEYS:
            delta = speed_delta if raw_key in REMOTE_SPEED_UP_KEYS else -speed_delta
            payload = {'task_id': REMOTE_SPEED_TASK_ID,
                       'delta_speed': float(delta)}
            request_id = uuid.uuid4().hex
            with data_lock:
                shared_data['pending_remote_calls'][request_id] = 'speed'
            try:
                ros_node.publish_remote_service_call(
                    REMOTE_SERVICE_NAME, REMOTE_SERVICE_TYPE, payload,
                    request_id=request_id,
                )
            except Exception as e:
                with data_lock:
                    shared_data['pending_remote_calls'].pop(request_id, None)
                _show_toast(f"调速请求发送失败: {e}",
                            color='#b30000', bg='#ffd6d6', duration=3.0)
                return
            action = '加速' if delta > 0 else '减速'
            _show_toast(
                f"{action} {delta:+.2f} m/s 请求中…",
                color='#1f3b7a', bg='#e3ecff', duration=2.0,
            )
            print(f"远程服务: 已发送 {REMOTE_SERVICE_NAME} delta_speed={delta:+.2f}")
            return

        # 远程服务快捷键：ctrl+g 发送导航目标点（x/y 来自 --nav-target-x/y，其余字段同 docs/malasong.sh）
        if raw_key == START_NAV_KEY:
            request_id = uuid.uuid4().hex
            with data_lock:
                shared_data['pending_remote_calls'][request_id] = 'start_nav'
            payload = build_start_nav_payload(nav_target_x, nav_target_y)
            try:
                ros_node.publish_remote_service_call(
                    START_NAV_SERVICE_NAME, START_NAV_SERVICE_TYPE,
                    payload, request_id=request_id,
                )
            except Exception as e:
                with data_lock:
                    shared_data['pending_remote_calls'].pop(request_id, None)
                _show_toast(f"发送目标点失败: {e}",
                            color='#b30000', bg='#ffd6d6', duration=3.0)
                return
            _show_toast(
                f"发送目标点 ({nav_target_x:.2f}, {nav_target_y:.2f}) 请求中…",
                color='#1f3b7a', bg='#e3ecff', duration=2.0,
            )
            print(f"远程服务: 已发送 {START_NAV_SERVICE_NAME} target=({nav_target_x:.2f}, {nav_target_y:.2f})")
            return

        # 跟随模式下调节窗口大小：- 增大 10m，+ 减小 10m
        if follow_pose_enabled and key in ('+', '=', '-'):
            if key in ('+', '='):
                follow_window_size = max(10.0, follow_window_size - 10.0)
            else:
                follow_window_size += 10.0
            if follow_pose_last_center is not None:
                _set_follow_window_center(follow_pose_last_center[0], follow_pose_last_center[1])
            _update_title()
            fig.canvas.draw_idle()
            print(f"跟随窗口大小调整为 {follow_window_size:.1f}m x {follow_window_size:.1f}m")
            return

        if key == follow_pose_key:
            follow_pose_enabled = not follow_pose_enabled
            if follow_pose_enabled:
                if view_before_follow is None:
                    view_before_follow = (ax.get_xlim(), ax.get_ylim())
                with data_lock:
                    pose = shared_data.get('robot_pose')
                if pose is not None:
                    cx = float(pose['x'])
                    cy = float(pose['y'])
                    _set_follow_window_center(cx, cy)
                    follow_pose_last_center = (cx, cy)
                    print(f"已开启跟随: 锁定到 /hric/loc/pose，中心=({cx:.2f}, {cy:.2f})，窗口={follow_window_size:.1f}m")
                else:
                    follow_pose_last_center = None
                    print("已开启跟随: 等待 /hric/loc/pose 数据后自动锁定窗口")
            else:
                follow_pose_last_center = None
                if view_before_follow is not None:
                    prev_x, prev_y = view_before_follow
                    ax.set_xlim(prev_x[0], prev_x[1])
                    ax.set_ylim(prev_y[0], prev_y[1])
                    view_before_follow = None
                print("已关闭跟随: 恢复为自由视角")
            _update_title()
            fig.canvas.draw_idle()
            return

        # 额外兜底：ESC 快速关闭跟随
        if key == 'escape' and follow_pose_enabled:
            follow_pose_enabled = False
            follow_pose_last_center = None
            if view_before_follow is not None:
                prev_x, prev_y = view_before_follow
                ax.set_xlim(prev_x[0], prev_x[1])
                ax.set_ylim(prev_y[0], prev_y[1])
                view_before_follow = None
            print("已关闭跟随: ESC 触发自由视角")
            _update_title()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    # 交互功能（如果需要）
    if args.interactive:
        successors, predecessors, lefts, rights = build_neighbor_maps(nodes, edges_typed)
        lane_id_to_points: Dict[str, List[Tuple[float, float]]] = {lid: nodes[lid].points for lid in lane_ids_in_order}
        highlighted: List = []
        info_annotation: Optional[Annotation] = None
        
        def clear_highlight():
            nonlocal highlighted, info_annotation
            for art in highlighted:
                try:
                    art.remove()
                except Exception:
                    pass
            highlighted = []
            if info_annotation is not None:
                try:
                    info_annotation.remove()
                except Exception:
                    pass
                info_annotation = None
            fig.canvas.draw_idle()
        
        def nearest_lane(x: float, y: float, radius: float) -> Optional[str]:
            best_id = None
            best_d2 = float('inf')
            px, py = x, y
            r2 = radius * radius
            for lid in lane_ids_in_order:
                pts = lane_id_to_points[lid]
                for i in range(len(pts) - 1):
                    x1, y1 = pts[i]
                    x2, y2 = pts[i + 1]
                    vx, vy = x2 - x1, y2 - y1
                    wx, wy = px - x1, py - y1
                    denom = vx * vx + vy * vy
                    if denom <= 1e-12:
                        t = 0.0
                    else:
                        t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
                    cx = x1 + t * vx
                    cy = y1 + t * vy
                    dx = px - cx
                    dy = py - cy
                    d2 = dx * dx + dy * dy
                    if d2 < best_d2 and d2 <= r2:
                        best_d2 = d2
                        best_id = lid
            return best_id
        
        def highlight(lid: str):
            nonlocal highlighted
            if lid not in nodes:
                return
            base_color = (1.0, 0.3, 0.0, 0.95)
            succ_color = (0.0, 0.65, 0.2, 0.95)
            pred_color = (0.2, 0.4, 1.0, 0.95)
            side_color = (0.7, 0.2, 0.9, 0.95)
            
            def add_polyline(poly, color):
                lc = LineCollection([poly], colors=[color], linewidths=args.highlight_width, zorder=10)
                ax.add_collection(lc)
                highlighted.append(lc)
            
            poly = nodes[lid].points
            add_polyline(poly, base_color)
            for nid in sorted(successors[lid]):
                if nid in nodes:
                    add_polyline(nodes[nid].points, succ_color)
            for nid in sorted(predecessors[lid]):
                if nid in nodes:
                    add_polyline(nodes[nid].points, pred_color)
            for nid in sorted((lefts[lid] | rights[lid])):
                if nid in nodes:
                    add_polyline(nodes[nid].points, side_color)
        
        def format_info(lid: str) -> str:
            return (
                f"lane: {lid}\n"
                f"前: {', '.join(sorted(successors[lid])) or '-'}\n"
                f"后: {', '.join(sorted(predecessors[lid])) or '-'}\n"
                f"左: {', '.join(sorted(lefts[lid])) or '-'}\n"
                f"右: {', '.join(sorted(rights[lid])) or '-'}"
            )

        def locate_lane(lid: str) -> bool:
            """定位到指定车道：调整视图并高亮。"""
            nonlocal info_annotation
            if lid not in nodes:
                print(f"未找到车道: {lid}")
                return False

            pts = nodes[lid].points
            if not pts:
                print(f"车道 {lid} 没有几何数据")
                return False

            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)

            pad_x = max((max_x - min_x) * 0.2, 10.0)
            pad_y = max((max_y - min_y) * 0.2, 10.0)
            ax.set_xlim(min_x - pad_x, max_x + pad_x)
            ax.set_ylim(min_y - pad_y, max_y + pad_y)

            clear_highlight()
            highlight(lid)
            info_annotation = ax.annotate(
                format_info(lid),
                xy=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
                xytext=(10, 10),
                textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.3', fc='w', ec='0.5', alpha=0.9),
                fontsize=9,
                zorder=11,
            )
            _update_title()
            fig.canvas.draw_idle()
            print(f"已定位到车道: {lid}")
            return True
        
        def on_click(event):
            if event.inaxes != ax:
                return
            if event.button != 1:
                return
            if event.xdata is None or event.ydata is None:
                return
            lid = nearest_lane(event.xdata, event.ydata, radius=args.pick_radius)
            clear_highlight()
            if lid is None:
                fig.canvas.draw_idle()
                return
            highlight(lid)
            text = format_info(lid)
            nonlocal info_annotation
            info_annotation = ax.annotate(
                text,
                xy=(event.xdata, event.ydata),
                xytext=(10, 10),
                textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.3', fc='w', ec='0.5', alpha=0.9),
                fontsize=9,
                zorder=11,
            )
            fig.canvas.draw_idle()
        
        cid_click = fig.canvas.mpl_connect('button_press_event', on_click)
        
        if args.hover:
            def on_move(event):
                nonlocal hover_lane_id
                if event.inaxes != ax:
                    return
                if event.xdata is None or event.ydata is None:
                    return
                lid = nearest_lane(event.xdata, event.ydata, radius=args.pick_radius)
                hover_lane_id = lid
                _update_title()
                fig.canvas.draw_idle()
            cid_move = fig.canvas.mpl_connect('motion_notify_event', on_move)

        if args.enable_search:
            search_queue: queue.Queue = queue.Queue()
            search_running = True

            def stdin_listener():
                """后台线程：监听 stdin 输入的车道 ID。"""
                print("\n搜索功能已启用。输入车道ID进行搜索（输入 'q' 或 'quit' 退出搜索模式）:")
                while search_running:
                    try:
                        line = input().strip()
                        if not line:
                            continue
                        if line.lower() in ('q', 'quit', 'exit'):
                            print("退出搜索模式")
                            break
                        search_queue.put(line)
                    except (EOFError, KeyboardInterrupt):
                        break
                    except Exception as e:
                        print(f"输入错误: {e}")

            def process_search_queue():
                """在主线程中处理搜索请求，避免跨线程操作 matplotlib。"""
                try:
                    while not search_queue.empty():
                        lid = search_queue.get_nowait()
                        locate_lane(lid)
                except queue.Empty:
                    pass

            search_thread = threading.Thread(target=stdin_listener, daemon=True)
            search_thread.start()

            search_timer = fig.canvas.new_timer(interval=100)
            search_timer.add_callback(process_search_queue)
            search_timer.start()
    elif args.enable_search:
        print("提示: --enable-search 依赖 --interactive；当前未开启交互模式，搜索功能不会生效。")
    
    print("实时可视化已启动，按Ctrl+C退出...")
    plt.tight_layout()
    plt.show()
    
    # 清理
    rclpy.shutdown()


if __name__ == '__main__':
    main()

