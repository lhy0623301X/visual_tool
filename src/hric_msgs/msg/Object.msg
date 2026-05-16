uint8 UNKNOWN=0
uint8 CAR=1
uint8 TRUCK=2
uint8 BUS=3
uint8 BICYCLE=4
uint8 MOTORBIKE=5
uint8 PEDESTRIAN=6
uint8 ANIMAL=7
uint8 ROBOT=8
uint8 CHAIR=9
uint8 TABLE=10
uint8 WALL=11
uint8 WINDOW=12
uint8 DOOR=13
uint8 PLANT=14
uint8 APPLIANCE=15
uint8 FURNITURE=16
uint8 OBJECTS=17

# 算法类型常量
int8 BEV=0
int8 OCC=1

string id
uint32 type
float64 confidence
geometry_msgs/Twist velocity
geometry_msgs/Vector3 position
geometry_msgs/Vector3 dimension
geometry_msgs/Quaternion orientation
geometry_msgs/Point[] shape
float64[2] height 
int8 algo                               # 算法类型: BEV=0, OCC=1
