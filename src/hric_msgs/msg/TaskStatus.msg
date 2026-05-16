#导航错误码
uint32 NONE=0 # 无异常
uint32 UNKNOWN=100
uint32 GOAL_OCCUPIED=101 #目标点被占用
uint32 TIMEOUT=102 #规划超时
uint32 NO_VALID_PATH=103 #无可通行路线
uint32 NO_MAP_LOADED=104 #未加载地图
uint32 DYNAMIC_OBSTACLES=105 #动态障碍物阻塞路径
uint32 RC_TAKEOVER=106 #遥控接管

#导航状态码
# uint8 IDLE = 0   # 空闲状态
# uint8 EXECUTE_TASK =1 #执行导航任务，去往目标点
# uint8 CANCELED = 2 #导航任务被取消
# uint8 FAILED = 3 #导航任务失败
# uint8 REACHED_GOAL = 4#已到达目标位置，导航任务完成

uint8 IDLE = 0          # 空闲状态,未开始执行
uint8 EXECUTE =1   #正在执行
uint8 CANCELED = 2      #取消
uint8 FAILED = 3        #失败
uint8 SUCCEED = 4  #成功
uint8 ABNORMAL = 5 #任务异常
uint8 CAUTION = 6 #任务风险

string task_id
uint8 task_type
uint8 task_status #任务状态
uint32 error_code #错误码