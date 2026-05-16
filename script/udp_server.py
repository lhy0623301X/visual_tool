#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hric_msgs.msg import ObjectArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from bodyctrl_msgs.msg import MotorStatusMsg1, RtkGps, PowerBatteryStatus
import socket
import json
import threading
import time
import errno
import queue
import importlib

try:
    from rosidl_runtime_py.set_message import set_message_fields
    from rosidl_runtime_py.convert import message_to_ordereddict
    _ROSIDL_RUNTIME_AVAILABLE = True
except ImportError:
    _ROSIDL_RUNTIME_AVAILABLE = False


# 远程服务白名单：服务名 -> 期望的 srv 类型（pkg/srv/Name）。
# 不在表内的调用请求一律拒绝；srv_type 不匹配也拒绝。
ALLOWED_SERVICES = {
    "/hric/nav/adjust_nav_speed": "hric_msgs/srv/AdjustNavSpeed",
    "/hric/nav/start_nav": "hric_msgs/srv/StartNav",
}


def _to_jsonable(obj):
    """把 message_to_ordereddict 的结果转成纯 JSON 可序列化的 dict/list。"""
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (bytes, bytearray)):
        return list(obj)
    return str(obj)


class UdpServerPublisher(Node):
    def __init__(self):
        super().__init__('udp_server')
        
        # UDP 设置
        self.udp_host = '0.0.0.0'  # 监听所有接口
        self.udp_port = 8080
        self.connected_clients = set()  # 存储已连接的客户端
        self.client_last_seen = {}  # addr -> monotonic 时间，用于心跳超时剔除
        self._clients_lock = threading.Lock()
        # 客户端需在超时内至少收到一次心跳类报文（ping/connect 等）
        self.heartbeat_timeout_sec = 5.0
        
        # 回调频率控制 (Hz) - 可配置
        self.callback_frequency = 2.5  # 默认 10Hz，与发送频率一致
        
        # 创建 UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 增大发送缓冲区，缓解瞬时拥塞导致的 EAGAIN
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 100 << 20)  # 100MB
        self.udp_socket.bind((self.udp_host, self.udp_port))
        
        # 设置为非阻塞模式
        self.udp_socket.setblocking(False)
        
        # 初始化存储消息的变量
        self.loc_pose_data = None
        self.plan_path_data = None
        self.object_array_data = None
        self.leg_motor_status_data = None
        self.arm_motor_status_data = None
        self.rtk_gps_data = None
        self.battery_status_data = None
        
        # 创建回调组，用于控制回调频率
        self.callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        
        # 创建订阅者
        self.loc_pose_sub = self.create_subscription(
            Odometry,
            '/hric/loc/pose',
            self.loc_pose_callback,
            2,
            callback_group=self.callback_group
        )
        
        self.plan_path_sub = self.create_subscription(
            Path,
            '/planning/path',
            self.plan_path_callback,
            2,
            callback_group=self.callback_group
        )
        
        self.object_array_sub = self.create_subscription(
            ObjectArray,
            '/perception/objects',
            self.object_array_callback,
            2,
            callback_group=self.callback_group
        )
        
        # 腿部关节数据订阅者
        self.leg_joint_status_sub = self.create_subscription(
            MotorStatusMsg1,
            '/leg/motor_status',
            self.leg_joint_status_callback,
            2,
            callback_group=self.callback_group
        )

        self.arm_joint_status_sub = self.create_subscription(
            MotorStatusMsg1,
            '/arm/motor_status',
            self.arm_joint_status_callback,
            2,
            callback_group=self.callback_group
        )
        
        # RTK 数据订阅者
        self.rtk_gps_sub = self.create_subscription(
            RtkGps,
            '/rtk_gps',
            self.rtk_gps_callback,
            2,
            callback_group=self.callback_group
        )
        
        # 电池数据订阅者
        self.battery_status_sub = self.create_subscription(
            PowerBatteryStatus,
            '/power/battery/status',
            self.battery_status_callback,
            2,
            callback_group=self.callback_group
        )
        
        # 远程服务 RPC 相关
        self._service_call_queue: queue.Queue = queue.Queue()
        self._srv_clients: dict = {}  # svc_name -> (Client, srv_cls)
        # 预先尝试导入白名单里的 srv 类型，启动时暴露问题
        self._preload_whitelisted_srv_types()

        # 启动接收客户端连接的线程
        self.receive_thread = threading.Thread(target=self.receive_clients)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # 启动发送消息的定时器
        self.timer = self.create_timer(0.4, self.send_messages_to_clients)  # 10Hz

        # 启动处理回调的定时器（控制回调处理频率）
        self.callback_timer = self.create_timer(0.4, self.process_callbacks)  # 10Hz 处理回调

        # 启动服务调用调度定时器（20Hz，快一点，保证按键响应）
        self.service_call_timer = self.create_timer(
            0.05, self._drain_service_calls, callback_group=self.callback_group
        )

        self.get_logger().info(f'UDP Server started on port {self.udp_port}')
        self.get_logger().info(f'Callback frequency limited to {self.callback_frequency}Hz')
        self.get_logger().info(
            f'Client heartbeat timeout: {self.heartbeat_timeout_sec}s (expect periodic ping)'
        )
        self.get_logger().info(f'Allowed remote services: {list(ALLOWED_SERVICES.keys())}')
        self.get_logger().info('Listening for client connections...')

    def loc_pose_callback(self, msg):
        """处理定位姿态消息"""
        self.loc_pose_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'child_frame_id': msg.child_frame_id,
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            },
            'twist': {
                'linear': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
        }

    def plan_path_callback(self, msg):
        """处理路径消息"""
        poses = []
        for pose_stamped in msg.poses:
            poses.append({
                'header': {
                    'stamp': {'sec': pose_stamped.header.stamp.sec, 'nanosec': pose_stamped.header.stamp.nanosec},
                    'frame_id': pose_stamped.header.frame_id
                },
                'pose': {
                    'position': {
                        'x': pose_stamped.pose.position.x,
                        'y': pose_stamped.pose.position.y,
                        'z': pose_stamped.pose.position.z
                    },
                    'orientation': {
                        'x': pose_stamped.pose.orientation.x,
                        'y': pose_stamped.pose.orientation.y,
                        'z': pose_stamped.pose.orientation.z,
                        'w': pose_stamped.pose.orientation.w
                    }
                }
            })
        
        self.plan_path_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'poses': poses
        }

    def object_array_callback(self, msg):
        """处理感知对象消息"""
        objects = []
        for obj in msg.objects:
            obj_dict = {
                'id': obj.id,
                'type': obj.type,
                'confidence': obj.confidence,
                'velocity': {
                    'linear': {
                        'x': obj.velocity.linear.x,
                        'y': obj.velocity.linear.y,
                        'z': obj.velocity.linear.z
                    },
                    'angular': {
                        'x': obj.velocity.angular.x,
                        'y': obj.velocity.angular.y,
                        'z': obj.velocity.angular.z
                    }
                },
                'position': {
                    'x': obj.position.x,
                    'y': obj.position.y,
                    'z': obj.position.z
                },
                'dimension': {
                    'x': obj.dimension.x,
                    'y': obj.dimension.y,
                    'z': obj.dimension.z
                },
                'orientation': {
                    'x': obj.orientation.x,
                    'y': obj.orientation.y,
                    'z': obj.orientation.z,
                    'w': obj.orientation.w
                },
                'shape': [],
                'height': [float(h) for h in obj.height],
                'algo': obj.algo
            }
            
            # 添加形状点
            for point in obj.shape:
                obj_dict['shape'].append({
                    'x': point.x,
                    'y': point.y,
                    'z': point.z
                })
                
            objects.append(obj_dict)
        
        self.object_array_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'objects': objects,
            'egopose': {
                'header': {
                    'stamp': {'sec': msg.egopose.header.stamp.sec, 'nanosec': msg.egopose.header.stamp.nanosec},
                    'frame_id': msg.egopose.header.frame_id
                },
                'transform': {
                    'translation': {
                        'x': msg.egopose.transform.translation.x,
                        'y': msg.egopose.transform.translation.y,
                        'z': msg.egopose.transform.translation.z
                    },
                    'rotation': {
                        'x': msg.egopose.transform.rotation.x,
                        'y': msg.egopose.transform.rotation.y,
                        'z': msg.egopose.transform.rotation.z,
                        'w': msg.egopose.transform.rotation.w
                    }
                }
            }
        }

    def leg_joint_status_callback(self, msg):
        """处理腿部关节状态消息"""
        status_list = []
        for motor in msg.status:
            status_list.append({
                'name': motor.name,
                'motortemperature': motor.motortemperature,
                'mostemperature': motor.mostemperature
            })
        
        self.leg_motor_status_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'status': status_list
        }
    def arm_joint_status_callback(self, msg):
        """处理手部关节状态消息"""
        status_list = []
        for motor in msg.status:
            status_list.append({
                'name': motor.name,
                'motortemperature': motor.motortemperature,
                'mostemperature': motor.mostemperature
            })
        
        self.arm_motor_status_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'status': status_list
        }

    def rtk_gps_callback(self, msg):
        """处理 RTK GPS 消息"""
        self.rtk_gps_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'status': msg.status,
            'service': msg.service,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance),
            'position_covariance_type': msg.position_covariance_type,
            'gprmc_track': msg.gprmc_track,
            'gprmc_speed': msg.gprmc_speed,
            'heading': msg.heading,
            'rtk_difference_age': msg.rtk_difference_age,
            'satelites_num': msg.satelites_num,
            'interval_gps_sys': msg.interval_gps_sys,
            'gps_sys_delay': msg.gps_sys_delay
        }

    def battery_status_callback(self, msg):
        """处理电池状态消息"""
        self.battery_status_data = {
            'header': {
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'frame_id': msg.header.frame_id
            },
            'battery_installed': msg.battery_installed,
            'battery_working': msg.battery_working,
            'master_battery_voltage': msg.master_battery_voltage,
            'master_battery_current': msg.master_battery_current,
            'master_battery_power': msg.master_battery_power,
            'little_battery_voltage': msg.little_battery_voltage,
            'little_battery_current': msg.little_battery_current,
            'little_battery_power': msg.little_battery_power,
            'pg12a': msg.pg12a,
            'pg12b': msg.pg12b,
            'pg12c': msg.pg12c,
            'pg12d': msg.pg12d,
            'pg5cd': msg.pg5cd,
            'pg5ab': msg.pg5ab,
            'pgrdc2': msg.pgrdc2,
            'pgrdc1': msg.pgrdc1,
            'pgheader': msg.pgheader,
            'pgbutton2': msg.pgbutton2
        }

    def process_callbacks(self):
        """周期性处理回调队列，控制回调执行频率"""
        # 让 ROS2 处理待处理的回调
        # 这样可以控制回调的执行频率，而不是来一个消息就立即执行
        rclpy.spin_once(self, timeout_sec=0.0)

    def _remove_client_unlocked(self, addr):
        """在持有 _clients_lock 时调用"""
        if addr in self.connected_clients:
            self.connected_clients.discard(addr)
            self.client_last_seen.pop(addr, None)
            return True
        self.client_last_seen.pop(addr, None)
        return False

    def _touch_client_unlocked(self, addr):
        """在持有 _clients_lock 时调用：登记或刷新心跳时间"""
        now = time.monotonic()
        self.client_last_seen[addr] = now
        if addr not in self.connected_clients:
            self.connected_clients.add(addr)
            self.get_logger().info(f'New client connected: {addr}')

    def _prune_stale_clients(self):
        """剔除超过 heartbeat_timeout_sec 未收到心跳的客户端"""
        now = time.monotonic()
        with self._clients_lock:
            stale = [
                addr for addr in self.connected_clients
                if now - self.client_last_seen.get(addr, 0) > self.heartbeat_timeout_sec
            ]
            for addr in stale:
                self._remove_client_unlocked(addr)
                self.get_logger().info(f'Client heartbeat timeout, removed: {addr}')

    def receive_clients(self):
        """接收 UDP 客户端连接 / 心跳 / 断开 / 服务调用请求"""
        # 创建独立的定时器来控制接收频率
        receive_rate = self.create_rate(20.0)  # 20Hz 轮询：服务调用需要低延迟

        while rclpy.ok():
            try:
                # 非阻塞模式接收，没有数据时会抛出 BlockingIOError
                data, addr = self.udp_socket.recvfrom(65536)

                # 以 '{' 开头的优先按 JSON 解析（service_call 等控制报文）
                if data.lstrip().startswith(b'{'):
                    try:
                        msg = json.loads(data.decode('utf-8'))
                    except Exception as e:
                        self.get_logger().warn(f'Bad JSON from {addr}: {e}')
                        continue
                    mtype = msg.get('type')
                    if mtype == 'service_call':
                        # 服务调用也算心跳，刷新 last_seen
                        with self._clients_lock:
                            self._touch_client_unlocked(addr)
                        self._service_call_queue.put((addr, msg))
                    else:
                        self.get_logger().warn(f'Unknown JSON type from {addr}: {mtype!r}')
                    continue

                # 兜底：旧的字节指令（connect / ping / heartbeat / disconnect）
                cmd = data.strip().lower()
                with self._clients_lock:
                    if cmd == b'disconnect':
                        if self._remove_client_unlocked(addr):
                            self.get_logger().info(f'Client disconnected: {addr}')
                    elif cmd in (b'connect', b'ping', b'heartbeat'):
                        self._touch_client_unlocked(addr)
            except BlockingIOError:
                # 非阻塞模式下没有数据时的正常情况
                pass

            # 控制轮询频率（与 create_rate 一致）
            receive_rate.sleep()

    def send_messages_to_clients(self):
        """向所有已连接的客户端发送消息"""
        self._prune_stale_clients()
        with self._clients_lock:
            if not self.connected_clients:
                return
            clients = tuple(self.connected_clients)
            
        # 准备要发送的数据
        message_data = {}
        
        if self.loc_pose_data is not None:
            message_data['loc_pose'] = self.loc_pose_data
            
        if self.plan_path_data is not None:
            message_data['plan_path'] = self.plan_path_data
            
        if self.object_array_data is not None:
            message_data['object_array'] = self.object_array_data
            
        if self.leg_motor_status_data is not None:
            message_data['leg_motor_status'] = self.leg_motor_status_data
        if self.arm_motor_status_data is not None:
            message_data['arm_motor_status'] = self.arm_motor_status_data
            
        if self.rtk_gps_data is not None:
            message_data['rtk_gps'] = self.rtk_gps_data
        
        if self.battery_status_data is not None:
            message_data['battery_status'] = self.battery_status_data
        
        if not message_data:
            return  # 如果没有任何数据，则不发送
        
        try:
            # 序列化消息
            serialized_data = json.dumps(message_data).encode('utf-8')
            
            # 发送给所有客户端
            for client in clients:
                try:
                    self.udp_socket.sendto(serialized_data, client)
                except BlockingIOError as e:
                    # 非阻塞 socket 下：暂时不可写（EAGAIN/EWOULDBLOCK）
                    self.get_logger().warn(f'UDP send would block for {client}: {e}')
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        self.get_logger().warn(f'UDP send would block for {client}: {e}')
                        continue
                    self.get_logger().warn(f'Removing client {client} due to error: {e}')
                    with self._clients_lock:
                        self._remove_client_unlocked(client)
                except Exception as e:
                    self.get_logger().warn(f'Removing client {client} due to error: {e}')
                    with self._clients_lock:
                        self._remove_client_unlocked(client)
                    
        except Exception as e:
            self.get_logger().error(f'Error sending messages to clients: {e}')

    # ------------------ 远程服务 RPC ------------------

    def _preload_whitelisted_srv_types(self):
        """启动时尝试 import 白名单里的 srv 类型；失败只打印警告。"""
        for svc_name, srv_type in ALLOWED_SERVICES.items():
            try:
                self._import_srv_class(srv_type)
            except Exception as e:
                self.get_logger().warn(
                    f'Preload srv type failed for {svc_name} ({srv_type}): {e}'
                )

    @staticmethod
    def _import_srv_class(srv_type: str):
        """'pkg/srv/Name' -> pkg.srv.Name 类对象。"""
        parts = srv_type.split('/')
        if len(parts) != 3 or parts[1] != 'srv':
            raise ValueError(f'invalid srv_type: {srv_type!r}')
        pkg, _, name = parts
        module = importlib.import_module(f'{pkg}.srv')
        return getattr(module, name)

    def _get_service_client(self, svc_name, srv_type):
        cached = self._srv_clients.get(svc_name)
        if cached is not None:
            return cached
        srv_cls = self._import_srv_class(srv_type)
        client = self.create_client(
            srv_cls, svc_name, callback_group=self.callback_group
        )
        self._srv_clients[svc_name] = (client, srv_cls)
        return client, srv_cls

    def _drain_service_calls(self):
        """从队列取服务调用请求并分派（在 rclpy 线程里跑）。"""
        while True:
            try:
                addr, msg = self._service_call_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._dispatch_service_call(addr, msg)
            except Exception as e:
                rid = msg.get('request_id', '') if isinstance(msg, dict) else ''
                self._send_service_response(
                    addr, rid, success=False,
                    error=f'dispatch exception: {e}'
                )

    def _dispatch_service_call(self, addr, msg):
        request_id = str(msg.get('request_id', '') or '')
        svc_name = msg.get('service', '')
        srv_type = msg.get('srv_type', '')
        payload = msg.get('payload') or {}

        expected = ALLOWED_SERVICES.get(svc_name)
        if expected is None:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'service not in whitelist: {svc_name!r}'
            )
        if expected != srv_type:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'srv_type mismatch: expected {expected!r}, got {srv_type!r}'
            )
        if not _ROSIDL_RUNTIME_AVAILABLE:
            return self._send_service_response(
                addr, request_id, success=False,
                error='rosidl_runtime_py unavailable on server'
            )

        try:
            client, srv_cls = self._get_service_client(svc_name, srv_type)
        except Exception as e:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'cannot create client: {e}'
            )

        # 快速可用性检查；首次调用若 DDS 还没发现到服务，给 0.5s 等待
        if not client.wait_for_service(timeout_sec=0.5):
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'service unavailable: {svc_name}'
            )

        try:
            req = srv_cls.Request()
            if payload:
                set_message_fields(req, payload)
        except Exception as e:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'cannot fill request: {e}'
            )

        try:
            future = client.call_async(req)
        except Exception as e:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'call_async failed: {e}'
            )

        self.get_logger().info(
            f'Dispatched service call {svc_name} req_id={request_id[:8]} from {addr}'
        )
        future.add_done_callback(
            lambda f, addr=addr, rid=request_id, name=svc_name:
                self._on_service_future(f, addr, rid, name)
        )

    def _on_service_future(self, future, addr, request_id, svc_name):
        try:
            resp = future.result()
        except Exception as e:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'service call exception: {e}'
            )
        try:
            resp_dict = _to_jsonable(message_to_ordereddict(resp))
        except Exception as e:
            return self._send_service_response(
                addr, request_id, success=False,
                error=f'cannot serialize response: {e}'
            )
        self.get_logger().info(
            f'Service {svc_name} req_id={request_id[:8]} -> {resp_dict}'
        )
        self._send_service_response(
            addr, request_id, success=True, response=resp_dict
        )

    def _send_service_response(self, addr, request_id, success,
                               response=None, error=None):
        payload = {
            'type': 'service_response',
            'request_id': request_id,
            'success': bool(success),
            'response': response,
            'error': error,
        }
        try:
            data = json.dumps(payload).encode('utf-8')
            self.udp_socket.sendto(data, addr)
        except Exception as e:
            self.get_logger().warn(
                f'Failed to send service_response to {addr}: {e}'
            )

    # --------------------------------------------------

    def destroy_node(self):
        """清理资源"""
        self.udp_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    udp_server = UdpServerPublisher()
    
    try:
        # 使用 Timer 驱动而不是 spin
        # 所有任务都由内部的 Timer 控制频率
        while rclpy.ok():
            # 主循环只需要维持节点运行即可
            rclpy.spin_once(udp_server, timeout_sec=0.1)
    except KeyboardInterrupt:
        udp_server.get_logger().info('Shutting down UDP Server...')
    finally:
        udp_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()