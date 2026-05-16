# Visual Tool — 人形机器人实时路线可视化工具

实时可视化人形机器人的导航路径、定位位姿、感知障碍物、关节状态等信息，支持远程 UDP 数据传输与交互式地图操作。

## 系统架构

```
┌──────────────────── 机器人端 ──────────────────────┐     UDP/JSON    ┌────────────────── 操作端 ──────────────────┐
│                                                   │    ◄────────►   │                                            │
│  ROS 2 Topics                                     │                 │  udp_client.py                             │
│  ├─ /hric/loc/pose          (Odometry)            │   8080 端口     │  ├─ 接收 JSON 数据流                       │
│  ├─ /planning/path          (Path)                │   ──────────►   │  ├─ 重发布为 /visual/* ROS 2 话题          │
│  ├─ /perception/objects     (ObjectArray)         │                 │  └─ 心跳保活（1s 间隔）                    │
│  ├─ /leg/motor_status       (MotorStatusMsg1)     │                 │                                            │
│  ├─ /arm/motor_status       (MotorStatusMsg1)     │                 │  visualize_routing_map_realtime.py         │
│  ├─ /rtk_gps                (RtkGps)              │                 │  ├─ 订阅 /visual/* + 本地 ROS 话题         │
│  └─ /power/battery/status   (PowerBatteryStatus)  │                 │  ├─ Apollo HD-Map 解析与渲染               │
│                                                   │                 │  ├─ matplotlib 交互式地图                  │
│  udp_server.py                                    │                 │  └─ 状态面板（速度/电量/关节温度）         │
│  ├─ 订阅上述话题                                   │                 │                                            │
│  ├─ 序列化为 JSON                                  │                 │  远程服务调用                              │
│  └─ 广播到已注册 UDP 客户端（~2.5Hz）               │   ◄──────────   │  ├─ 快捷键调速 (./>  加速, ,/< 减速)      │
│                                                   │   服务调用请求   │  └─ Ctrl+G 下发导航目标点                  │
└───────────────────────────────────────────────────┘                 └────────────────────────────────────────────┘
```

**核心设计**：可视化工具不直接进行 UDP 通信。`udp_client.py` 作为唯一的 UDP 桥接节点，将远程数据转发为标准 ROS 2 话题，使可视化工具在远程 UDP 模式和本地局域网 ROS 话题模式下行为完全一致。

## 目录结构

```
visual_tool_v7/
├── script/
│   ├── udp_server.py                       # [机器人端] UDP 数据转发服务
│   ├── udp_client.py                       # [操作端]   UDP 客户端 → ROS 2 话题桥接
│   ├── visualize_routing_map_realtime.py   # [操作端]   交互式实时地图可视化（~3400 行）
│   └── fonts/
│       └── NotoSansCJKsc-Regular.otf       # 中文字体（离线环境备用）
├── map/                                    # Apollo HD-Map 地图数据
│   ├── malasong/                           # 各场景地图
│   ├── guoqizhilian_modified5/
│   ├── guoqizhilian_modified7/
│   ├── guoqizhilian_modified8/
│   ├── yuanqu02/
│   └── yuanquda84/
│       ├── routing_map.txt                 # 路线拓扑（车道节点 + 前后继边）
│       ├── base_map.txt                    # 车道几何（左右边界采样点）
│       ├── routing_map.bin                 # 二进制格式（本工具不使用）
│       └── base_map.bin
├── src/
│   ├── bodyctrl_msgs/                      # ROS 2 消息包：电机/电源/RTK-GPS
│   │   ├── msg/                            # MotorStatusMsg1, RtkGps, PowerBatteryStatus 等
│   │   └── srv/
│   └── hric_msgs/                          # ROS 2 消息包：感知/规划/导航
│       ├── msg/                            # ObjectArray, PlannedTrajectory 等
│       ├── srv/                            # AdjustNavSpeed, StartNav 等
│       └── action/
├── run_visualization.sh                    # 一键启动脚本（udp_client + 可视化）
└── CLAUDE.md                               # AI 编码助手指南
```

## 环境要求

- **ROS 2 Humble** (Ubuntu 22.04)
- **Python 3.10+**
- Python 依赖（随 ROS 2 已安装或需额外安装）：
  - `matplotlib` (TkAgg 后端)
  - `rclpy`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`
- **colcon** 构建工具

## 快速开始

### 1. 编译消息包

```bash
# 确保已 source ROS 2 Humble 环境
source /opt/ros/humble/setup.bash

# 编译自定义消息包
colcon build

# source 编译产物
source install/setup.bash
```

### 2. 一键启动（推荐）

编辑 `run_visualization.sh` 顶部的配置变量：

```bash
MAP_PATH="$WORKSPACE_DIR/map/malasong"      # 地图目录
UDP_SERVER_HOST="10.11.177.221"              # 机器人 IP
UDP_SERVER_PORT="8080"                       # UDP 端口
SPEED_DELTA="0.5"                            # 调速步长 (m/s)
NAV_TARGET_X="453000.47"                     # 导航目标 X 坐标
NAV_TARGET_Y="4404732.38"                    # 导航目标 Y 坐标
```

然后运行：

```bash
./run_visualization.sh
```

脚本会自动启动 `udp_client.py`（后台）和 `visualize_routing_map_realtime.py`（前台），退出时自动清理后台进程。

### 3. 分步启动

**机器人端**（在机器人上运行）：

```bash
python3 script/udp_server.py
# 绑定 0.0.0.0:8080，等待客户端 connect/ping
```

**操作端**（在操作电脑上运行）：

```bash
# 终端 1：启动 UDP 桥接
python3 script/udp_client.py --server-host 10.11.177.221 --server-port 8080

# 终端 2：启动可视化
python3 script/visualize_routing_map_realtime.py \
    -m map/malasong \
    --interactive --enable-search \
    --use-origin-obs --use-converted-obs \
    --planning-path dp_path --qp_path
```

### 4. 本地模式（无需 UDP）

当操作端与机器人在同一 ROS 2 网络中，可直接使用本地话题，跳过 UDP：

```bash
python3 script/visualize_routing_map_realtime.py \
    -m map/malasong --interactive --enable-search \
    --use-origin-obs --planning-path dp_path \
    --follow-pose-on-start --follow-pose-key f
```

## 可视化功能

### 地图渲染

- 解析 Apollo HD-Map 格式的 `routing_map.txt`（车道拓扑）和 `base_map.txt`（车道边界几何）
- 车道中心线绘制，支持按道路/代价着色
- 左右边界虚线绘制
- 车道方向箭头标注
- 补给站、起终点线等特殊标记

### 实时叠加层

| 叠加层 | 数据源 | 说明 |
|--------|--------|------|
| 机器人位姿 | `/hric/loc/pose` | 实时位置 + 朝向箭头 |
| 规划路径 | `/planning/path` | 全局规划路线 |
| DP 路径 | `/planning/dp_path` | 动态规划路径（需 `--planning-path dp_path`） |
| QP 路径 | `/planning/qp_path` | 二次规划路径（需 `--qp_path`） |
| 轨迹 | `/hric/nav/plan_trajectory` | 规划轨迹（需 `--plot-tra`，替代 planning/path） |
| 原始障碍物 | `/perception/objects` | ObjectArray 格式（需 `--use-origin-obs`） |
| 转换障碍物 | `/perception/converted_obstacles` | MarkerArray 格式（需 `--use-converted-obs`） |

### 状态面板（右上角）

- 当前速度（线速度 + 角速度）
- 电池电量与电压
- RTK-GPS 定位状态（Fixed/Float/SPP）与卫星数
- 腿部/手臂关节温度（高温报警：>120°C 红色，>80°C 橙色）

### 交互操作

| 操作 | 说明 |
|------|------|
| 鼠标左键点击 | 选中最近车道，显示 ID、邻接关系 |
| 鼠标悬停 | 标题栏显示当前车道 ID（需 `--hover`） |
| `F` 键 | 切换视窗跟随机器人位姿 |
| `.` / `>` 键 | 远程加速（步长由 `--speed-delta` 控制） |
| `,` / `<` 键 | 远程减速 |
| `Ctrl+G` | 下发导航目标点 |
| 命令行输入车道 ID | 搜索定位到指定车道（需 `--enable-search`） |

## 命令行参数

### visualize_routing_map_realtime.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-m, --map-dir` | — | 地图目录（需含 routing_map.txt 和 base_map.txt） |
| `--interactive` | off | 启用交互模式（点击查看车道信息） |
| `--enable-search` | off | 启用车道搜索（依赖 `--interactive`） |
| `--planning-path` | `path` | `path`=仅 planning/path，`dp_path`=叠加 dp_path |
| `--qp_path` | off | 叠加 planning/qp_path |
| `--plot-tra` | off | 使用 PlannedTrajectory 替代 planning/path |
| `--use-origin-obs` | off | 使用 /perception/objects |
| `--use-converted-obs` | off | 使用 /perception/converted_obstacles |
| `--follow-pose-on-start` | off | 启动即开启视窗跟随 |
| `--follow-pose-key` | `f` | 切换视窗跟随的按键 |
| `--follow-pose-window` | `20.0` | 跟随窗口边长（米） |
| `--pick-radius` | `3.0` | 交互拾取半径 |
| `--figsize W H` | `12 12` | 画布尺寸（英寸） |
| `--color-by` | `road` | 着色方式：road / cost / none |
| `--draw-edges` | off | 绘制拓扑边 |
| `--bbox` | — | 可选绘制边界框 (MIN_X MIN_Y MAX_X MAX_Y) |
| `--cjk-font-file` | — | 中文字体文件路径（或环境变量 `VISUAL_TOOL_CJK_FONT`） |
| `--speed-delta` | `0.5` | 远程调速步长 (m/s) |
| `--nav-target-x` | `453000.47` | 导航目标 X 坐标 |
| `--nav-target-y` | `4404732.38` | 导航目标 Y 坐标 |
| `--routing-response` | — | RoutingResponse proto 文件路径 |
| `--route-width` | `5.0` | 路线覆盖线宽 |
| `--boundary-line-width` | `0.6` | 边界虚线线宽 |
| `--boundary-alpha` | `0.85` | 边界虚线透明度 |

### udp_client.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server-host` | — | UDP 服务器 IP |
| `--server-port` | `8080` | UDP 服务器端口 |

## UDP 数据协议

### 通信流程

```
客户端 ──── b'connect' ────► 服务端         # 注册
客户端 ──── b'ping' ───────► 服务端         # 心跳（每 1s）
客户端 ◄──── JSON 数据 ───── 服务端         # 数据广播（~2.5Hz）
客户端 ──── JSON 请求 ─────► 服务端         # 远程服务调用
客户端 ◄──── JSON 响应 ───── 服务端         # 服务调用结果

超时：客户端 5s 无心跳则被剔除
```

### JSON 数据字段

| JSON Key | 机器人端话题 | 操作端重发布话题 | 消息类型 |
|----------|-------------|-----------------|----------|
| `loc_pose` | `/hric/loc/pose` | `/visual/loc/pose` | `nav_msgs/Odometry` |
| `plan_path` | `/planning/path` | `/visual/plan_path` | `nav_msgs/Path` |
| `object_array` | `/perception/objects` | `/visual/perception/objects` | `MarkerArray` |
| `leg_motor_status` | `/leg/motor_status` | `/visual/leg/motor_status` | `MotorStatusMsg1` |
| `arm_motor_status` | `/arm/motor_status` | `/visual/arm/motor_status` | `MotorStatusMsg1` |
| `rtk_gps` | `/rtk_gps` | `/visual/rtk_gps` | `RtkGps` |
| `battery_status` | `/power/battery/status` | `/visual/power/battery/status` | `PowerBatteryStatus` |

### 远程服务白名单

| 服务名 | 类型 | 触发方式 |
|--------|------|----------|
| `/hric/nav/adjust_nav_speed` | `hric_msgs/srv/AdjustNavSpeed` | `.`/`>` 加速，`,`/`<` 减速 |
| `/hric/nav/start_nav` | `hric_msgs/srv/StartNav` | `Ctrl+G` |

## 地图数据格式

地图使用 **Apollo HD-Map 文本 Proto** 格式（非 ROS nav_map）：

- **`routing_map.txt`**：车道拓扑图
  - 节点 = 车道（含 `central_curve` 中心线点序列）
  - 边 = 前后继关系（successor / predecessor）
  - 特殊标注：补给站、赛道起终点线

- **`base_map.txt`**：车道几何边界
  - `left_sample` / `right_sample`：左右边界采样点序列
  - 用于绘制车道边界虚线

可视化工具直接解析文本 proto（`parse_routing_map` / `parse_lane_samples_from_base_map`），首次解析后缓存为 `.parse_cache.pkl` 加速后续加载。

## 自定义消息类型

### bodyctrl_msgs

| 消息 | 说明 |
|------|------|
| `MotorStatusMsg1` | 电机状态（包含 `MotorStatus1[]`：关节名、电机温度、MOS 温度） |
| `RtkGps` | RTK-GPS 数据（定位状态、经纬度、卫星数、航向、速度） |
| `PowerBatteryStatus` | 电池信息（大/小电池电压、电流、电量、电源板状态） |

### hric_msgs

| 消息 | 说明 |
|------|------|
| `ObjectArray` | 感知障碍物列表（含 `Object[]`：类型、置信度、位姿、形状） |
| `Object` | 单个障碍物（支持 18 种类型：车辆、行人、动物、机器人、家具等） |
| `PlannedTrajectory` | 规划轨迹（`TrajectoryPoint[]`：位姿、速度、累积路程） |

## 中文字体配置

所有 UI 文字均为中文。字体查找优先级：

1. `--cjk-font-file` 参数指定的字体文件
2. `VISUAL_TOOL_CJK_FONT` 环境变量
3. 系统字体（Noto Sans CJK SC → SimHei → WenQuanYi Zen Hei → Source Han Sans SC）
4. 内置备用字体：`script/fonts/NotoSansCJKsc-Regular.otf`
