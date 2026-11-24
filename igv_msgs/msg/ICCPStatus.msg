# 集成底盘控制平台状态信息 ICCP(Integrated Chassis Control Platform)
# 发布："/iccp_status"

std_msgs/Header header

# BMS info
float32 soc                 # 电池电量, 0.4%, max=100%
uint8 bms_charge_sts        # 充电状态，0-未充电，1-正在充电，2-充电完成，3-无效
uint8 battery_number        # 电池包标识: 当前使用电池包，1-1号，2-2号
uint8 battery_warning       # 电量报警: 0-无报警，1-电量低，2-急需充电,3-预留

# diagnostic info
# uint8 error_level         # 底盘故障等级
uint8 error_eps_level       # 线控转向故障等级
uint8 error_eb_level        # 线控制动故障等级
uint8 error_bms_level       # BMS故障等级
uint8 error_motor_code      # 驱动电机故障代码

# auto driving system
float32 motor_rpm           # 电机转速, 1rpm, 正负30000

float32 current_angle       # 当前转角, °, 正负18°
uint8 ses_en_sts            # 转向电机使能状态：0-Not En,1-En
uint8 motor_en_sts          # 驱动电机使能状态：0-Not En,1-En
float32 current_speed       # 当前车速 (0.01m/s), max=12.7m/s

float32 motor_torque        # 当前电机扭矩(Nm),max=25.5

uint8 vcu_mode              # VCU当前模式: 0-预备切换模式；1-人工遥控模式；2-线控模式；3-紧急制动模式
uint8 vcu_life_signal       # VCU生命信号: 0-15循环递增变化

float32 current_brk_press   # 当前压力(Mpa), max=8Mpa

uint8 rseb_mode             # 线控制动模式：0-预留，1-位置模式，2-压力模式
uint8 current_gear          # 当前挡位: 0-N挡，1-D挡，2-R挡，3-预留

uint8 current_spd_mode      # 当前速度模式: 0-低速，1-中速，2-高速，3-预留
uint8 controller_online_sts # 遥控器连接状态: 0-离线，1-在线
uint8 ipc_online_status     # IPC连接状态: 0-离线，1-在线

uint8 motor_direction       # 驱动电机运动方向，0-静止，1-前进，2-后退,3-预留
float32 ode_info            # 里程计信息，0.01km
uint8 park_sts              # 驻车状态,0-抱死，1-释放
uint8 collision_status      # 碰撞状态，0-无，1-发生碰撞
# uint8 automode_on         # 自动驾驶开关，0-关闭，1-开启，遥控器开关状态
# uint8 automode_enable     # 自动驾驶使能，0-不使能，1-使能，遥控器开关状态

uint8 emergency_status      # 急停标识位：0-无急停，1-VCU急停，2-DBS急停，3-两者同时存在
string vcu_version          # VCU版本
uint8 error_level           # 底盘故障等级
uint8 first_level_fault     # 一级故障码
uint8 second_level_fault    # 二级故障码
uint8 third_level_fault     # 三级故障码

uint8 ipc_low_beam_light    # 近光灯开关状态 0-关闭, 1-开启
uint8 door_lock_status      # 车门闭合状态 0-车门未关闭, 1-车门关闭
uint8 door_lock_relay       # 电磁门锁继电器状态 0-关闭, 1-开启
uint8 ac_relay              # 空调继电器状态 0-关闭, 1-开启

