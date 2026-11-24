int8        motor_en         #驱动电机使能 0- not en，1-en
int8        gear             #目标挡位 0-N档，1-D档，2-R档
int8        mode             #目标模式 0-人工模式，1-线控模式
int8        life_signal      #心跳包 0-15循环递增
float64     speed            #目标车速
int8        spdvalid         #目标车速是否有效，0-无效，1-有效
float64     brk_press        #目标制动压力
float64     angle            #目标转角
int8        steering_en      #转向电机使能 0- not en，1-en
float64     torque           #目标扭矩
int32       torque_rpm_mode  #0-扭矩模式，1-速度模式
int8        turnlight        #0-不进行控制，1-左转向灯，2-右转向灯，3-双闪
int8        spdmode          #0-低速模式，1-中速模式，2-高速模式，3-预留
int8        ode_clear        #是否清除里程计信息,0-无效，1-清除
