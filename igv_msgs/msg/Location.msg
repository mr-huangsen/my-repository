float64 latitude                 #纬度
float64 longitude                #经度
float64 altitude                 #高度
int32 status                     #定位状态 1-success ，其他-failure
string state                     #状态
uint64 timestamp                 #时间戳
float64 steer                    #车辆前轮转角
geometry_msgs/Vector3 vel        #速度vx\vy\vz
geometry_msgs/Vector3 gyr        #角速度roll\pitch\yaw
geometry_msgs/Pose car_pose      #位置和姿态，姿态是四元素