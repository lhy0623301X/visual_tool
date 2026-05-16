#运控错误码
uint32 NONE=0 # 无异常
uint32 UNKNOWN=400

std_msgs/Header header #时间戳和坐标系
geometry_msgs/Twist velocity #机器人腰部坐标系速度
uint8 walk_mode #行走状态 0: stop , 1: zero, 2: Z2S, 3: stand, 4: S2W, 5: walk, 6: start, 7: run
bool is_console_control #是否为遥控器操控 true:遥控器操纵false：导航操控
bool is_swing_arm #是否为walk摆臂状态
uint32 error_code #错误码