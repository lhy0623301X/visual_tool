uint8 status
float64 confidence
uint32 error_code
geometry_msgs/PoseStamped pose

#状态码
uint8 IDLE=100 #空闲状态， 不能导航
uint8 INITIALIZING=101 #初始化中， 不能导航
uint8 RUNNING=102 #正常状态
uint8 WEAK=103 #
uint8 LOST=104 #（不能导航）
uint8 BLOCKED=105 #（传感器数据异常、初始化失败，不能导航）

#错误码
uint32 NONE=0
uint32 UNKNOWN=200
uint32 TF_ERROR=201 #TF错误
uint32 SENSOR_ERROR=202 #传感器异常
uint32 RELOCALIZATION_ERROR=203 #重定位失败