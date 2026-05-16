#!/usr/bin/env python3

import argparse
import os
import socket
import json
import uuid
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, String
from bodyctrl_msgs.msg import MotorStatusMsg1, RtkGps, PowerBatteryStatus
import time


class UdpClientSubscriber(Node):
    def __init__(self, server_host: str, server_port: int):
        super().__init__('udp_client')
        
        # 创建发布者
        self.loc_pose_pub = self.create_publisher(Odometry, '/visual/loc/pose', 10)
        self.plan_path_pub = self.create_publisher(Path, '/visual/plan_path', 10)
        self.perception_objects_pub = self.create_publisher(MarkerArray, '/visual/perception/objects', 10)
        self.leg_motor_status_pub = self.create_publisher(MotorStatusMsg1, '/visual/leg/motor_status', 10)
        self.arm_motor_status_pub = self.create_publisher(MotorStatusMsg1, '/visual/arm/motor_status', 10)
        self.rtk_gps_pub = self.create_publisher(RtkGps, '/visual/rtk_gps', 10)
        self.battery_status_pub = self.create_publisher(PowerBatteryStatus, '/visual/power/battery/status', 10)
        
        # UDP 相关设置
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.server_address = ('127.0.0.1', 8080)  # 连接到 UDP 服务器端口 8080
        self.server_address = (server_host, server_port)
        # 发送初始连接消息到服务器
        self.udp_socket.sendto(b'connect', self.server_address)

        # 远程服务调用：请求发布/响应订阅（std_msgs/String，内容是 JSON）
        self.remote_call_sub = self.create_subscription(
            String, '/visual/remote_service_call', self._on_remote_service_call, 10
        )
        self.remote_result_pub = self.create_publisher(
            String, '/visual/remote_service_result', 10
        )
        # request_id -> (deadline_monotonic, svc_name)
        self._pending_calls = {}
        self._pending_lock = threading.Lock()
        self.default_service_timeout_sec = 5.0

        # 心跳周期需明显小于服务端 heartbeat_timeout_sec（默认 5s）
        self.heartbeat_interval_sec = 1.0
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval_sec, self._send_heartbeat
        )

        # 启动接收消息的定时器
        self.timer = self.create_timer(0.1, self.receive_and_publish_messages)  # 10Hz

        # 服务调用超时扫描
        self.service_timeout_timer = self.create_timer(0.2, self._check_pending_timeouts)

        self.get_logger().info('UDP Client started, connecting to server...')

    def _send_heartbeat(self):
        try:
            self.udp_socket.sendto(b'ping', self.server_address)
        except OSError:
            pass

    def receive_and_publish_messages(self):
        """接收 UDP 消息并发布为 ROS2 消息"""
        try:
            # 设置 socket 超时，非阻塞接收
            self.udp_socket.settimeout(0.05)  # 50 毫秒超时
            data, addr = self.udp_socket.recvfrom(65536)  # 接收最大 64KB 数据

            if data:
                try:
                    # 解析 JSON 数据
                    message_data = json.loads(data.decode('utf-8'))

                    # 控制报文：按顶层 type 分流（service_response 等）
                    mtype = message_data.get('type') if isinstance(message_data, dict) else None
                    if mtype == 'service_response':
                        self._handle_service_response(message_data)
                        return

                    # 处理定位姿态消息并发布为 Odometry
                    if 'loc_pose' in message_data:
                        self.publish_loc_pose(message_data['loc_pose'])

                    # 处理路径消息并发布
                    if 'plan_path' in message_data:
                        self.publish_plan_path(message_data['plan_path'])

                    # 处理感知对象消息并发布为 MarkerArray
                    if 'object_array' in message_data:
                        self.publish_perception_objects(message_data['object_array'])

                    # 处理腿部关节状态消息
                    if 'leg_motor_status' in message_data:
                        self.publish_leg_motor_status(message_data['leg_motor_status'])

                    # 处理手部关节状态消息
                    if 'arm_motor_status' in message_data:
                        self.publish_arm_motor_status(message_data['arm_motor_status'])

                    # 处理 RTK GPS 消息
                    if 'rtk_gps' in message_data:
                        self.publish_rtk_gps(message_data['rtk_gps'])

                    # 处理电池状态消息
                    if 'battery_status' in message_data:
                        self.publish_battery_status(message_data['battery_status'])
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'Error decoding JSON: {e}')
                except Exception as e:
                    self.get_logger().error(f'Error processing received data: {e}')

        except socket.timeout:
            # 超时是正常的，因为我们设置了非阻塞模式
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving messages: {e}')

    # ------------------ 远程服务调用桥接 ------------------

    def _on_remote_service_call(self, msg: String):
        """收到 ROS 侧的调用请求（JSON）→ 打 UDP service_call 报文。"""
        try:
            req = json.loads(msg.data) if msg.data else {}
        except Exception as e:
            self.get_logger().error(f'remote_service_call JSON decode: {e}')
            return

        svc_name = req.get('service', '')
        srv_type = req.get('srv_type', '')
        payload = req.get('payload') or {}
        timeout_sec = float(req.get('timeout_sec', self.default_service_timeout_sec))
        # 客户端可以带 request_id；不带则我们生成一个
        request_id = req.get('request_id') or uuid.uuid4().hex

        if not svc_name or not srv_type:
            self._emit_result(request_id, False, None,
                              error='missing service or srv_type')
            return

        udp_payload = {
            'type': 'service_call',
            'request_id': request_id,
            'service': svc_name,
            'srv_type': srv_type,
            'payload': payload,
            'timeout_sec': timeout_sec,
        }
        try:
            self.udp_socket.sendto(
                json.dumps(udp_payload).encode('utf-8'), self.server_address
            )
        except Exception as e:
            self._emit_result(request_id, False, None,
                              error=f'UDP send failed: {e}')
            return

        with self._pending_lock:
            self._pending_calls[request_id] = (
                time.monotonic() + timeout_sec, svc_name
            )
        self.get_logger().info(
            f'Remote service_call sent: {svc_name} req_id={request_id[:8]}'
        )

    def _handle_service_response(self, message_data: dict):
        request_id = str(message_data.get('request_id', '') or '')
        success = bool(message_data.get('success', False))
        response = message_data.get('response')
        error = message_data.get('error')
        with self._pending_lock:
            self._pending_calls.pop(request_id, None)
        self._emit_result(request_id, success, response, error)

    def _check_pending_timeouts(self):
        now = time.monotonic()
        expired = []
        with self._pending_lock:
            for rid, (deadline, svc) in list(self._pending_calls.items()):
                if now >= deadline:
                    expired.append((rid, svc))
                    self._pending_calls.pop(rid, None)
        for rid, svc in expired:
            self.get_logger().warn(f'Remote service timeout: {svc} req_id={rid[:8]}')
            self._emit_result(rid, False, None, error='timeout')

    def _emit_result(self, request_id, success, response, error):
        out = String()
        out.data = json.dumps({
            'request_id': request_id,
            'success': bool(success),
            'response': response,
            'error': error,
        })
        try:
            self.remote_result_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'publish remote_service_result failed: {e}')

    # ------------------------------------------------------

    def publish_loc_pose(self, loc_pose_data):
        """发布定位姿态消息为 Odometry"""
        odom_msg = Odometry()
        
        # 设置头部
        header_data = loc_pose_data['header']
        odom_msg.header.stamp.sec = header_data['stamp']['sec']
        odom_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        odom_msg.header.frame_id = header_data['frame_id']
        
        # 设置 child_frame_id
        odom_msg.child_frame_id = loc_pose_data['child_frame_id']
        
        # 设置位置
        pos_data = loc_pose_data['pose']['position']
        odom_msg.pose.pose.position.x = pos_data['x']
        odom_msg.pose.pose.position.y = pos_data['y']
        odom_msg.pose.pose.position.z = pos_data['z']
        
        # 设置方向
        orient_data = loc_pose_data['pose']['orientation']
        odom_msg.pose.pose.orientation.x = orient_data['x']
        odom_msg.pose.pose.orientation.y = orient_data['y']
        odom_msg.pose.pose.orientation.z = orient_data['z']
        odom_msg.pose.pose.orientation.w = orient_data['w']
        
        # 设置速度
        linear_vel = loc_pose_data['twist']['linear']
        angular_vel = loc_pose_data['twist']['angular']
        odom_msg.twist.twist.linear.x = linear_vel['x']
        odom_msg.twist.twist.linear.y = linear_vel['y']
        odom_msg.twist.twist.linear.z = linear_vel['z']
        odom_msg.twist.twist.angular.x = angular_vel['x']
        odom_msg.twist.twist.angular.y = angular_vel['y']
        odom_msg.twist.twist.angular.z = angular_vel['z']
        
        self.loc_pose_pub.publish(odom_msg)

    def publish_plan_path(self, plan_path_data):
        """发布路径消息"""
        path_msg = Path()
        
        # 设置头部
        header_data = plan_path_data['header']
        path_msg.header.stamp.sec = header_data['stamp']['sec']
        path_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        path_msg.header.frame_id = header_data['frame_id']
        
        # 设置路径点
        for pose_data in plan_path_data['poses']:
            pose_stamped = self.create_pose_stamped_from_dict(pose_data)
            path_msg.poses.append(pose_stamped)
        
        self.plan_path_pub.publish(path_msg)

    def publish_perception_objects(self, object_array_data):
        """发布感知对象消息为 MarkerArray"""
        marker_array = MarkerArray()
        
        # 为每个对象创建一个 marker
        for i, obj in enumerate(object_array_data['objects']):
            # 创建基本 marker
            marker = Marker()
            marker.header.stamp.sec = object_array_data['header']['stamp']['sec']
            marker.header.stamp.nanosec = object_array_data['header']['stamp']['nanosec']
            marker.header.frame_id = object_array_data['header']['frame_id']
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE  # 使用立方体表示对象
            marker.action = Marker.ADD
            
            # 设置位置
            marker.pose.position.x = obj['position']['x']
            marker.pose.position.y = obj['position']['y']
            marker.pose.position.z = obj['position']['z']
            
            # 设置朝向
            marker.pose.orientation.x = obj['orientation']['x']
            marker.pose.orientation.y = obj['orientation']['y']
            marker.pose.orientation.z = obj['orientation']['z']
            marker.pose.orientation.w = obj['orientation']['w']
            
            # 设置大小（使用维度信息）
            marker.scale.x = max(0.1, obj['dimension']['x'])  # 最小尺寸 0.1
            marker.scale.y = max(0.1, obj['dimension']['y'])
            marker.scale.z = max(0.1, obj['dimension']['z'])
            
            # 根据对象类型设置颜色
            color = self.get_color_by_object_type(obj['type'])
            marker.color = color
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 秒生命周期
            
            marker_array.markers.append(marker)
            
            # 如果有形状点，也可以创建额外的 markers
            if len(obj['shape']) > 0:
                shape_marker = Marker()
                shape_marker.header.stamp.sec = object_array_data['header']['stamp']['sec']
                shape_marker.header.stamp.nanosec = object_array_data['header']['stamp']['nanosec']
                shape_marker.header.frame_id = object_array_data['header']['frame_id']
                shape_marker.ns = "object_shapes"
                shape_marker.id = i + 1000  # ID 偏移以避免冲突
                shape_marker.type = Marker.LINE_STRIP
                shape_marker.action = Marker.ADD
                
                # 设置线条颜色
                shape_marker.color.a = 0.8  # 透明度
                shape_marker.color.r = 0.0
                shape_marker.color.g = 1.0
                shape_marker.color.b = 0.0
                
                # 设置线条宽度
                shape_marker.scale.x = 0.05
                
                # 添加形状点
                for point in obj['shape']:
                    pt = Point()
                    pt.x = point['x']
                    pt.y = point['y']
                    pt.z = point['z']
                    shape_marker.points.append(pt)
                
                # 闭合线条
                if len(shape_marker.points) > 0:
                    shape_marker.points.append(shape_marker.points[0])
                
                shape_marker.lifetime.sec = 0
                shape_marker.lifetime.nanosec = 200000000  # 0.2 秒生命周期
                marker_array.markers.append(shape_marker)
        
        self.perception_objects_pub.publish(marker_array)

    def publish_leg_motor_status(self, leg_motor_status_data):
        """发布腿部关节状态消息"""
        motor_status_msg = MotorStatusMsg1()
        
        # 设置头部
        header_data = leg_motor_status_data['header']
        motor_status_msg.header.stamp.sec = header_data['stamp']['sec']
        motor_status_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        motor_status_msg.header.frame_id = header_data['frame_id']
        
        # 设置电机状态列表
        from bodyctrl_msgs.msg import MotorStatus1
        for status_data in leg_motor_status_data['status']:
            motor_status = MotorStatus1()
            motor_status.name = status_data['name']
            motor_status.motortemperature = status_data['motortemperature']
            motor_status.mostemperature = status_data['mostemperature']
            motor_status_msg.status.append(motor_status)
        
        self.leg_motor_status_pub.publish(motor_status_msg)

    def publish_arm_motor_status(self, arm_status_data):
        """发布腿部关节状态消息"""
        motor_status_msg = MotorStatusMsg1()
        
        # 设置头部
        header_data = arm_status_data['header']
        motor_status_msg.header.stamp.sec = header_data['stamp']['sec']
        motor_status_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        motor_status_msg.header.frame_id = header_data['frame_id']
        
        # 设置电机状态列表
        from bodyctrl_msgs.msg import MotorStatus1
        for status_data in arm_status_data['status']:
            motor_status = MotorStatus1()
            motor_status.name = status_data['name']
            motor_status.motortemperature = status_data['motortemperature']
            motor_status.mostemperature = status_data['mostemperature']
            motor_status_msg.status.append(motor_status)
        
        self.arm_motor_status_pub.publish(motor_status_msg)

    def publish_rtk_gps(self, rtk_gps_data):
        """发布 RTK GPS 消息"""
        rtk_gps_msg = RtkGps()
        
        # 设置头部
        header_data = rtk_gps_data['header']
        rtk_gps_msg.header.stamp.sec = header_data['stamp']['sec']
        rtk_gps_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        rtk_gps_msg.header.frame_id = header_data['frame_id']
        
        # 设置其他字段
        rtk_gps_msg.status = rtk_gps_data['status']
        rtk_gps_msg.service = rtk_gps_data['service']
        rtk_gps_msg.latitude = rtk_gps_data['latitude']
        rtk_gps_msg.longitude = rtk_gps_data['longitude']
        rtk_gps_msg.altitude = rtk_gps_data['altitude']
        rtk_gps_msg.position_covariance = rtk_gps_data['position_covariance']
        rtk_gps_msg.position_covariance_type = rtk_gps_data['position_covariance_type']
        rtk_gps_msg.gprmc_track = rtk_gps_data['gprmc_track']
        rtk_gps_msg.gprmc_speed = rtk_gps_data['gprmc_speed']
        rtk_gps_msg.heading = rtk_gps_data['heading']
        rtk_gps_msg.rtk_difference_age = rtk_gps_data['rtk_difference_age']
        rtk_gps_msg.satelites_num = rtk_gps_data['satelites_num']
        rtk_gps_msg.interval_gps_sys = rtk_gps_data['interval_gps_sys']
        rtk_gps_msg.gps_sys_delay = rtk_gps_data['gps_sys_delay']
        
        self.rtk_gps_pub.publish(rtk_gps_msg)

    def publish_battery_status(self, battery_status_data):
        """发布电池状态消息"""
        battery_status_msg = PowerBatteryStatus()
        
        # 设置头部
        header_data = battery_status_data['header']
        battery_status_msg.header.stamp.sec = header_data['stamp']['sec']
        battery_status_msg.header.stamp.nanosec = header_data['stamp']['nanosec']
        battery_status_msg.header.frame_id = header_data['frame_id']
        
        # 设置电池状态信息
        battery_status_msg.battery_installed = battery_status_data['battery_installed']
        battery_status_msg.battery_working = battery_status_data['battery_working']
        battery_status_msg.master_battery_voltage = battery_status_data['master_battery_voltage']
        battery_status_msg.master_battery_current = battery_status_data['master_battery_current']
        battery_status_msg.master_battery_power = battery_status_data['master_battery_power']
        battery_status_msg.little_battery_voltage = battery_status_data['little_battery_voltage']
        battery_status_msg.little_battery_current = battery_status_data['little_battery_current']
        battery_status_msg.little_battery_power = battery_status_data['little_battery_power']
        battery_status_msg.pg12a = battery_status_data['pg12a']
        battery_status_msg.pg12b = battery_status_data['pg12b']
        battery_status_msg.pg12c = battery_status_data['pg12c']
        battery_status_msg.pg12d = battery_status_data['pg12d']
        battery_status_msg.pg5cd = battery_status_data['pg5cd']
        battery_status_msg.pg5ab = battery_status_data['pg5ab']
        battery_status_msg.pgrdc2 = battery_status_data['pgrdc2']
        battery_status_msg.pgrdc1 = battery_status_data['pgrdc1']
        battery_status_msg.pgheader = battery_status_data['pgheader']
        battery_status_msg.pgbutton2 = battery_status_data['pgbutton2']
        
        self.battery_status_pub.publish(battery_status_msg)

    def create_pose_stamped_from_dict(self, pose_data):
        """从字典创建 PoseStamped 消息"""
        from geometry_msgs.msg import PoseStamped
        
        pose_stamped = PoseStamped()
        
        header_data = pose_data['header']
        pose_stamped.header.stamp.sec = header_data['stamp']['sec']
        pose_stamped.header.stamp.nanosec = header_data['stamp']['nanosec']
        pose_stamped.header.frame_id = header_data['frame_id']
        
        pose_info = pose_data['pose']
        pose_stamped.pose.position.x = pose_info['position']['x']
        pose_stamped.pose.position.y = pose_info['position']['y']
        pose_stamped.pose.position.z = pose_info['position']['z']
        
        pose_stamped.pose.orientation.x = pose_info['orientation']['x']
        pose_stamped.pose.orientation.y = pose_info['orientation']['y']
        pose_stamped.pose.orientation.z = pose_info['orientation']['z']
        pose_stamped.pose.orientation.w = pose_info['orientation']['w']
        
        return pose_stamped

    def get_color_by_object_type(self, obj_type):
        """根据对象类型返回颜色"""
        color = ColorRGBA()
        color.a = 0.8  # 透明度
        
        # 根据对象类型分配颜色
        if obj_type == 1:  # CAR
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        elif obj_type == 6:  # PEDESTRIAN
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
        elif obj_type == 4:  # BICYCLE
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
        else:  # OTHERS
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        
        return color


def main(argv=None):
    # 先解析本脚本参数，其余参数保留给 rclpy
    parser = argparse.ArgumentParser(description='UDP client bridge for /visual/* topics')
    parser.add_argument(
        '--server-host',
        type=str,
        default=os.environ.get('UDP_SERVER_HOST', '127.0.0.1'),
        help='UDP server host (or env UDP_SERVER_HOST)',
    )
    parser.add_argument(
        '--server-port',
        type=int,
        default=int(os.environ.get('UDP_SERVER_PORT', '8080')),
        help='UDP server port (or env UDP_SERVER_PORT)',
    )
    parsed, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    udp_client = UdpClientSubscriber(parsed.server_host, parsed.server_port)
    
    try:
        rclpy.spin(udp_client)
    except KeyboardInterrupt:
        udp_client.get_logger().info('Shutting down UDP Client...')
    finally:
        udp_client.udp_socket.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()