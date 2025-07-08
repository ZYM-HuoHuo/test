# ROBOT_FRAME_24

华南师范大学 PIONEER战队 2024-2025 RoboMaster电控~~通用~~机器人框架

<img src=".asset/picture/pioneer_badge.png"/>

**PIONEER ROBOT FRAME**

**NOT THE BEST EMBEDDED ROBOTIC FRAME! 欢迎派霓电控er继续完善和补充**

[TOC]

# 框架目录

```python

─robot_core	
	├─APP
    │  		behaviour.c			状态机逻辑模块
    │  		behaviour.h
    |  		boards_interact.c	多板通信逻辑模块
    |  		boards_interact.h
    │  		bus_detect.c		   模块离线检测逻辑
    │  		bus_detect.h
    │  		chassis.c			   底盘控制逻辑模块
    │  		chassis.h
    |  		fire_ctrl.c         目标预测和火力控制逻辑
    |  		fire_ctrl.h
    │  		gimbal.c				云台控制逻辑模块
    │  		gimbal.h
    │  		pid_lpf_param.c		PID参数保存位置
    │  		pid_lpf_param.h
    │  		robot.c				   机器人功能总成
    │  		robot.h
    │  		shooter17.c			17mm发射控制逻辑模块
    |  		shooter42.c			42mm发射控制逻辑模块
    │  		shooter.h
    |  		vision.c				视觉接口
    |  		vision.h
    |      	......
    ├─Algorithm			
    |       CMSIS_DSP       DSP算法库文件夹
    |      	ballistic.c		弹道算法
    |      	ballistic.h
    |      	cordic.c        坐标旋转数字计算
    |      	cordic.h
    │      	crc.c		      官方CRC校验算法
    │      	crc.h
    │      	filter.c		   常见滤波算法（IIR，均值）
    │      	filter.h
    │      	imu_fusion.c	   六轴IMU解算算法
    │      	imu_fusion.h
    |      	imu_calibrate.c IMU校准算法
    |      	imu_calibrate.h
    |      	imu_dcm.c       方向余弦矩阵IMU解算
    |      	imu_dcm.h
    |      	KF.c		   卡尔曼滤波算法
    |      	KF.h
    │      	pid.c		      PID算法
    │      	pid.h
    │      	pid_ladrc.c	   LADRC算法
    │      	pid_ladrc.h
    │      	pid_leso.c	   集成LESO的PID算法
    │      	pid_leso.h
    │      	util.h		      常见数学函数（限幅，S曲线）
    |      	......
    ├─Base
    │    	base_chassis.c	底盘解算基础
    │    	base_chassis.h
    |     	base_gimbal.c   云台解算基础
    |     	base_gimbal.h
    │     	drv_can.c		   CAN接收基础
    │     	drv_can.h
    |     	drv_uart.c		串口收发基础
    |     	drv_uart.h
    |     	drv_ui.c        针对RM的图传UI底层
    |     	drv_uiOld.c     23年旧UI驱动 验证可用
    |     	drv_ui.h
    │     	drv_conf.h		框架配置文件
    |     	ext_imu.c		   外置HI229 IMU接收协议（已弃用）
    |     	ext_imu.h
    │     	power_ctrl.c		功率控制逻辑模块
    │     	power_ctrl.h
    │     	rc.c				DR16遥控/图传串口接收基础
    │     	rc.h
    │     	referee.c		   裁判系统接收基础
    │     	referee.h
    │     	referee_conf.h	裁判系统数据类型配置
    │     	super_cap.c		超级电容收发基础
    │  	  	super_cap.h
    ├─Devices
    |  ├─BMI088         bmi08x驱动库
    |  |     └─🕊️
    |  ├───	MOTOR       电机驱动库
    |  |     |    motor_lpf_param.c     电机自带滤波器参数
    |  |     |    motor.c              电机库接口
    |  |     |    motor.h              电机库主要配置
    |  |     |    motor_ctrl.c         集合电机底层控制逻辑
    |  |     |    motor_ctrl.h
    |  |     |    motor_ctrl_DM.c      达妙电机底层控制逻辑
    |  |     |    motor_ctrl_LK.c      瓴控电机底层控制逻辑
    |  |     |    motor_ctrl_RM.c      大疆电机底层控制逻辑
    |  |     |    motor_lib.h          电机主要参数
    |  |     |    ......
    |  |     └─🕊️
    ├─OS
    |  ├───	drv_freertos    移植的freertos核心
    |  |     └─🕊️
    |  └─  	robot_task.c    操作系统调度总览
    |      	robot_task.h
    ├─tests					
    |      	test.c          测试功能
    |      	test.h
    └─tools
    |		......
    └─drv_conf.h 			机器人配置文件
```

# 速通调试手册 ver 0.1

## 控制命令

【DT7遥控器控制】

详细见`behavior.c`中函数`update_robot_status_for_remoteControl`

1. 右上拨杆控制机器人模式
   1. 右上拨杆打下：强制断控
   2. 右上拨杆打中：正常模式
   3. 右上拨杆打上：检录模式
2. 正常模式中左拨杆控制发弹
   1. 若开启`drv_conf.h`中的`USE_DT7_WHEEL`
      1. 左上拨杆打下：关闭摩擦轮，不发弹
      2. 左上拨杆打中：连发，开启摩擦轮，拨动拨轮开火（弹频取决于拨轮拨动幅度）
      3. 左上拨杆打上：单发，开启摩擦轮，扣动拨轮开火
   2. 若不开启`drv_conf.h`中的`USE_DT7_WHEEL`
      1. 左上拨杆打下：关闭摩擦轮，不发弹
      2. 左上拨杆打中：连发，开启摩擦轮，不开火
      3. 左上拨杆打上：连发，开启摩擦轮，立即以最大弹频开火
3. 检录模式中左拨杆控制检录相关行为
   1. 左上拨杆打下：正常行为
   2. 左上拨杆打中：开启小陀螺(通过再次开启以更换小陀螺方向)
   3. 左上拨杆打上：巡检姿态

【键鼠遥控命令】

详细见`behavior.c`中函数`update_robot_status_for_keyboard`

## 机器人主要配置

`drv_conf.h`中涉及许多关于配置的宏定义：**定时器相关配置**、**串口相关配置**、**多板通信配置**、**机构配置**、**常用配置**等

`motor.h`中涉及机器人电机的相关配置，需要事先在`motors_t`联结体中**定义好存在的电机**，**并定义好各机构电机的INDEX和NUM**，按照**规定的方式对其进行初始化**（详见下文电机库的使用）

`chassis.h`中涉及机器人**底盘速度上限**、**底盘最大加速度**、**底盘小陀螺相关增益**等配置

`gimbal.h`中涉及机器人**俯仰角上限**、**云台零点位置**、**响应补偿**、**缓启动速度**、**缓启动死区**等配置

`shooter.h`中涉及机器人**弹速设置**、**弹频设置**、**堵转电流**等配置（视17mm和42mm而定）

`boards_interact.h`中可配置**多板通信**的各个**数据包属性**

`pid_parameter.c`中保存机器人的**所有pid参数**

## DEBUG注意事项

1. 推荐观测变量

   1. 建议拉取变量`robot`和变量`motor`

      前者记录机器人的所有控制信息（**详见`robot.h`**），包含机器人期望值和当前值，以及机载IMU的数据，后者记录机器人所有电机的信息（**详见`motor.h`**），方便观测电机使用情况、输出力矩、反馈情况

2. 云台

   1. **在`gimbal.c`和`gimbal.h`已有较为详尽的注释以说明一些容易出现的问题，如电机反装的可能性及其结果、一些基于现实效果的变量符号规定等**
   2. 注意变量`is_confirm_0degree`如果为0，则说明上次缓启动失败或从未进行缓启动，**拥有多种零点可能性的机器人需注意缓启动前的yaw轴摆放位置**（如2024赛季英雄），以便正确确定对应云台电机的零点。用于确定零点的缓启动被称为“真·缓启动”，基于电机编码值闭环控制。不用于确定零点的缓启动为普通缓启动，基于确定零点后的云台相对弧度闭环控制
   3. 云台在缓启动时`gimbal_ctrl_type`都应为`USE_MOTOR`，平常的默认控制则为`USE_IMU`，变量名的含义说明当前云台的`exp_state`和`cur_state`是基于电机系还是基于IMU系，真·缓启动时的**电机系参考量为的电机绝对编码值**`raw_scale`，其他情况的**电机系参考量都是云台的相对弧度**`pitch_rad,yaw_rad`



****

# 进阶简要

## 先看旧框架使用手册

https://scnu-pioneer.coding.net/p/ec/d/robot_frame_21/git/tree/master/docs

****

## 重要概念——ver 0.1

- 【**机构**】：机器人被模块化分成多个机构，各机构均能独立运行完成工作，现有步兵机器人机构被分为底盘、云台、发射机构

  a. **motors_type_t**表现出机构这一概念，不同机构所包含的电机不同，因此电机集合又以机构为划分标准，被分为**CHASSIS_MOTORS**/**GIMBAL_MOTORS**/**SHOOTER_MOTORS**

  b. **文件的命名**表现出机构这一概念，不同机构的控制逻辑代码被放置在不同文件中，例如底盘的中层控制逻辑被放置在`chassis.c`中，底层逻辑被放置在`base_chassis.c`中，云台和发射机构同理（tips:发射机构的中层逻辑和底层逻辑都被放置在了`shooter.c`）

  c. **机构初始化**在机构中层逻辑文件中进行，如`chassis.c`中的`chassis_init()`初始化底盘用到的所有pid参数以及底盘含有的所有电机

- 【**火控**】：机器人因弹药量有限且受热量限制，比赛中要尽量避免浪费弹丸的情况，为了保证机器人弹无虚发，`fire_ctrl.c`、`vision.c`、`gimbal.c`、`shooter.c`及部分底层文件配合完成机器人云台识别跟踪装甲板，并在合适的时机发射弹丸

- 【**TAG**】：TAG意为**机构的标签**，`behaviour.h`头文件中存在对机器人TAG的定义，使用枚举来实现，`C_TAG G_TAG S_TAG`分别对应着底盘、云台、发射机构的标签，每个标签都意味着一种控制模式，将会被传值到机构中层控制逻辑中，经由分类执行不同的控制逻辑

  a. C_TAG以**底盘是否保持平衡**、**机体旋转速度w_z**作为划分标准

  b. G_TAG以**云台是否受控于人**作为划分标准

  c.  S_TAG 以**发射频率**作为划分标准

- 【**电机集**】：在`motor.h`中定义了电机集`motors_t`，其由多个电机`motor_t`和一个`_[]`组成，联结体的设计使后者便于遍历，这是一个面向对象的电机集，获取电机集的指针后可以直接访问某个具体的电机。

  a. 电机以`motor_t`的形式存在于电机集中，每个电机都自带一个**低通滤波器**，携有其**反馈频率**、各**物理量**、**特定电机参数**，以及一些对于特殊电机的**辅助信息**

- 【**机器人信息**】：在`behaviour.h`中定义了机器人信息`robot_t`，其包含机器人状态以及各模块的启用情况，被后续逻辑频繁访问

- 【**机器人状态**】：在`behaviour.h`中定义了机器人状态`robot_state_t`，其中存放机器人状态变量**[v_x,v_y,w_z,pitch_ang,yaw_ang,roll_ang]**，而在**机器人信息**`robot_t`中实例化两个机器人状态

​	`robot_state_t expected_state;`

​	`robot_state_t current_state;`

​	分别表示机器人的**期望状态**和**当前状态**

​	**期望状态**多由**机构**控制中层逻辑文件中计算所得

​	**当前状态**则由**[云台]**控制类型决定参考量，如处于电机系控制时，其值是量程为[0,2PI]的云台相对云台零点的弧度值；处于IMU系控制时，其值是量程为[-4096,4096]的惯性测量欧拉角

- 【**启用摩擦轮**】默认下意为使摩擦轮转速达到定值，使射出来的子弹初速稳定在27m/s左右
- 【**启用推弹机构**】连发或限制状态下意为使拨弹盘电机转速达到定值，使弹频达到18Hz~20Hz之间，单发状态下则意为使拨弹盘电机能够根据**机器人信息**中的`use_fire_flag`标志位而让其旋转发射一颗弹丸的角度**ONE_SHOOT_ANG**

****

## 机器人控制逻辑链简介

1. `robot.c`中的周期函数被各自以一定频率运行，在控制对应机构前，先进行状态机的进程
2. `behaviour.c`作为机器人的状态机逻辑，在这里获取输入，完成状态机的切换（控制机器人上下控、选择控制源、检测按下的按键、检测遥控器的变化、切换机构TAG）

3. `chassis.c`中**初始化底盘**，根据状态机计算机体整体的期望底盘状态**v_x v_y w_z**
4. `base_chassis.c`中根据机体整体的期望底盘状态解算底盘具体电机的状态
5. `gimbal.c`中**初始化云台**，根据状态机计算机体整体的期望底盘状态**pitch_ang yaw_ang**  (tips:**roll_ang**未被设计)
6. `base_gimbal.c`中根据机体整体的期望云台状态解算底盘具体电机的状态(tips:分为电机系控制和IMU系控制)
7. `shooter.c`中**初始化发射机构**，根据状态机选择启用**推弹结构**或**摩擦轮**，并根据两者速度或角度期望解算对应电机状态
8. `motor_ctrl.c`根据所有电机的期望状态即output，分电机种类驱动电机

****

## 电机库使用

1. **初始化电机**：初始化电机分为两部分——为电机对象分配参数和为电机对象分配滤波器
   1. `RM_motor_inti()` 分配hcan 、定义电机类型、分配电机ID(电调闪几次就是几、定义控制频率)
   2. `can_user_init()` 以电机的反馈帧头为索引，函数一次可分配4个对象的滤波，注意fifo0和fifo1是否已经满溢，各分配超过13次将会出错
2. **关闭电机**：本框架采用给电机发0来关闭电机
   1. `shutdown_all_motor()`用于在断控时关闭所有电机，不建议放在高速定时器中生效，徒增can负载率，毫无价值
   2. `shutdown_motor()`用于关闭单个电机 **该函数应放在时钟频率不高的地方，较小CAN的负载率**
3. **电机输出**：建议电机统一使用力矩控制模式，认定电机output即为输出力矩，即载入的值的量纲为牛米（值得一提的是本框架中控制GM6020的output表示的是期望电压）
   1. `LOAD_MOTOR_TFF ()`即为电机载入力矩输出值，`LOAD_MOTOR_LPFTFF()`即在此基础上套入电机自带的LPF滤波器
   2. `set_all_motor_output()`即为驱动特定**机构**的所有(tips:若参数填入"DEFAULT"则使所有电机的所有电机输出)
   3. `set_all_motor_output()`中有对应的**convert**函数将电机输出值output转换成编码值再发送，因此不需要自行修改output量纲
4. **电机库优化**：本电机库面向对象，且致力于方便移植和控制所有的电机（旧框架只适用于RM电机），目前已收录DM电机、LK电机、RM电机，以后若有就新的电机被收录，需要完成以下工作
   1. 创建新的`XX_motor_ctrl()`，并完成对应的函数(参考其他电机ctrl.c文件的函数)
   2. 在`motor_lib.h`中补充新电机的参数(参考其他电机的写法)
   3. 在`set_all_motor_output()`函数中补充对应的逻辑(参考其他电机的写法)
5. **CAN上限：1M波特率 一发一收1000hz最多可以收发三个包 异步的一般是一发四收 最多可以控6个电机差不多，详细可用USB2CAN看CAN负载率来判断具体使用情况**

****

## 裁判系统使用

 与旧框架无异

****

## 多板通信使用

1. `drv_conf`中存在关键宏`BOARDS_MODE`和`MACHINE_TYPE`控制框架多板通信
2. 数据包面向对象编写，定义存在于`boards_interact.h`中，更改数据包数据需要注意修改`boards_interact.c`中的装载和解析函数
3. 相关can帧参数配置在`boards_interact.c`，详见注释

补充：确认没问题了再继续说明

****

## RTOS移植和使用

### 移植方法

1. **cubemx的SYS选择TIM2作为系统时钟**因为systim将作为FreeRTOS的时钟源，关闭全部定时器中断，保留一个定时器作为IMU启动时钟(用于实现bmi08x_delay_us_tim)
3. **imu启动函数的delay做修改** drv_bmi08x里面的宏`bmi08x_delay_us`应为 `bmi08x_Delay_us_tim`
4. ****屏蔽it里面的`void SysTick_Handler(void); void PendSV_Handler(void); void SVC_Handler(void);`
5. **移植整个drv_freertos文件(在robot_core/OS目录)到工程中，并且仿照导入到keil工程当中，选择heap4，port选择CMF4里的**（根据自己芯片做选择，C板为Cortex M4且非MPU 则使用CMF4）
6. **调用robot_init**并且屏蔽while函数
7. **task_create**开启所有任务调度，并调用`uxTaskGetStackHighWaterMark()`进行堆栈剩余测试，确保剩余超过25基本够用，在stm32中，1代表剩余4个byte

****

#### !!注意!!

如果独立出.c.h文件(CubeMX里的设置)，则允许存在的定时器**最大数量为9**

如果不独立出，则允许存在**最大数量为10**

**否则串口将出现异常，导致存在此限制的具体原因未知** （猜测是内存问题 过多定时器存在，即使不中断也会自个计数）

****

### 使用方法

`drv_conf.h`中有宏`OS_ENABLE` 置一开启操作系统，置零关闭操作系统

官方文档[Tasks and Co-routines [Getting Started\] - FreeRTOS](https://www.freertos.org/zh-cn-cmn-s/taskandcr.html)

🕊️

****

## 故障检测

故障检测有专用的10Hz定时器

现有的故障检测还不够完整，理想的情况应为对于所有报错情况都能得到反馈，能够第一时间辅助使用者定位问题

robot_core/tools/asr.c 用于语音播报机器人的调试情况，现在只是粗写了对应的接口，最终要具体到asr控制板上的逻辑

🕊️

****

# 新框架未完成部分

## UI

旧框架的UI最多只能创建21个图像，其中允许7个图像可动，且无法创建字符、数字图像

新框架对UI部分重新进行编写，效果需要验证

2024/7/17现已编写完成，但是启用函数会严重影响单发精度

## 半自动化

🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️🕊️



