
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 防止头文件被重复包含。 */
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/* 中文解读：电机驱动级总参数。包含16 kHz PWM、速度/电流PI、速度范围、保护阈值、
 * STO-PLL增益与可靠性判据、开环启动五阶段曲线。它描述“怎样控制”，是主要调参入口。 */

/** 电机控制参数区：以下宏由MCSDK配置对象在编译期引用。 */

/** 主速度/位置传感器设置：本工程主传感器为STO-PLL无感观测器。 */

/** 速度测量范围与可靠性设置。 */
#define MAX_APPLICATION_SPEED_RPM       3000 /* 允许的最大机械转速，单位rpm。 */
#define MIN_APPLICATION_SPEED_RPM       50 /* 允许的最小机械转速绝对值，单位rpm。 */
#define MEAS_ERRORS_BEFORE_FAULTS       200 /* 连续速度测量错误达到该次数后报告传感器故障。 */
/** STO-PLL状态观测器设置。 */
#define VARIANCE_THRESHOLD              0.1 /* 可接受的估算速度最大方差百分比。 */
/* 状态观测器的定点缩放因子F1、F2及其以2为底的指数。 */
#define F1                               16384
#define F2                               4096
#define F1_LOG                           LOG2(16384)
#define F2_LOG                           LOG2(4096)

/* 状态观测器离散模型增益。 */
#define GAIN1                            -23248
#define GAIN2                            18225
/* PLL环路PI增益及定点除数。 */
#define PLL_KP_GAIN                      532
#define PLL_KI_GAIN                      23
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2(PLL_KPDIV)
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2(PLL_KIDIV)

#define OBS_MEAS_ERRORS_BEFORE_FAULTS    200  /* 连续观测错误达到该次数后判定观测器不可靠。 */
#define STO_FIFO_DEPTH_DPP               64  /* 电角速度DPP滑动平均缓冲区深度。 */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2(64)

#define STO_FIFO_DEPTH_UNIT              64  /* 机械速度单位滑动平均缓冲区深度。 */
#define BEMF_CONSISTENCY_TOL             64   /* 反电势幅值一致性允许偏差。 */
#define BEMF_CONSISTENCY_GAIN            64   /* 反电势一致性判据增益。 */

/* 用户代码开始 */
#define PARK_ANGLE_COMPENSATION_FACTOR 0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* 用户代码结束 */

/** PWM生成、电流采样与电流环设置。 */

#define PWM_FREQUENCY   16000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   500 /* MCU互补PWM插入的死区时间，单位ns。 */

/* Id/Iq电流环每个PWM周期执行一次。 */
#define REGULATION_EXECUTION_RATE     1
/* 转矩轴Iq与磁链轴Id的默认PI/PID增益。 */
#define PID_TORQUE_KP_DEFAULT         2769
#define PID_TORQUE_KI_DEFAULT         2394
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           2769
#define PID_FLUX_KI_DEFAULT           2394
#define PID_FLUX_KD_DEFAULT           100

/* 电流环增益定点除数；LOG宏供移位运算使用。 */
#define TF_KPDIV                      2048
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2(2048)
#define TF_KIDIV_LOG                  LOG2(16384)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

//#define POSITION_LOOP_FREQUENCY_HZ    2000 /* 位置控制调节环的执行频率，单位Hz。 */
#define SPEED_LOOP_FREQUENCY_HZ       1000   /* 速度调节环执行频率，单位Hz。 */

#define PID_SPEED_KP_DEFAULT          230/(SPEED_UNIT/10) /* 速度环默认比例增益。 */
#define PID_SPEED_KI_DEFAULT          90/(SPEED_UNIT/10) /* 速度环默认积分增益。 */
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* 速度环默认微分增益，本工程关闭。 */
/* 速度环增益定点除数。 */
#define SP_KPDIV                      16
#define SP_KIDIV                      256
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2(16)
#define SP_KIDIV_LOG                  LOG2(256)
#define SP_KDDIV_LOG                  LOG2(16)

/* 用户代码开始 */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* 用户代码结束 */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          9532

/* 默认控制模式和上电参考值。 */
#define DEFAULT_CONTROL_MODE           STC_SPEED_MODE /* 默认使用速度闭环模式。 */
#define DEFAULT_TARGET_SPEED_RPM       -1500 //1500
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

/** 母线电压、温度和硬件过流保护参数。 */
#define OV_VOLTAGE_PROT_ENABLING        ENABLE
#define UV_VOLTAGE_PROT_ENABLING        ENABLE
#define OV_VOLTAGE_THRESHOLD_V          60 /* 母线过压阈值，单位V。 */
#define UD_VOLTAGE_THRESHOLD_V          12 /* 母线欠压阈值，单位V。 */
#if 0
#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /* 中文说明：处理并更新PWM和占空比。 */
#endif /* 0 */
#define R_BRAKE_SWITCH_OFF_THRES_V      48

#define OV_TEMPERATURE_THRESHOLD_C      110 /* 过温阈值，单位摄氏度。 */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /* 过温恢复滞回，单位摄氏度。 */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /* 中文说明：设置相电流、电压、功率。 */
/** 无感启动多阶段Rev-Up参数。每阶段定义持续时间、终速和终点Iq电流。 */

/* 第1阶段：初始对齐/占位，本配置持续时间为0。 */
#define PHASE1_DURATION                0 /* 单位ms。 */
#define PHASE1_FINAL_SPEED_UNIT         (0*SPEED_UNIT/_RPM)
#define PHASE1_FINAL_CURRENT           0
/* 第2阶段：保持零速并建立启动电流。 */
#define PHASE2_DURATION                700 /* 单位ms。 */
#define PHASE2_FINAL_SPEED_UNIT         (0*SPEED_UNIT/_RPM)
#define PHASE2_FINAL_CURRENT           1429
/* 第3阶段：由零速加速到630 rpm。 */
#define PHASE3_DURATION                350 /* 单位ms。 */
#define PHASE3_FINAL_SPEED_UNIT         (630*SPEED_UNIT/_RPM)
#define PHASE3_FINAL_CURRENT           1668
/* 第4阶段：继续加速到2700 rpm，为STO建立足够反电势。 */
#define PHASE4_DURATION                1150 /* 单位ms。 */
#define PHASE4_FINAL_SPEED_UNIT         (2700*SPEED_UNIT/_RPM)
#define PHASE4_FINAL_CURRENT           1668
/* 第5阶段：终点保持，本配置持续时间为0。 */
#define PHASE5_DURATION                0 /* 单位ms。 */
#define PHASE5_FINAL_SPEED_UNIT         (2700*SPEED_UNIT/_RPM)
#define PHASE5_FINAL_CURRENT           1668

#define ENABLE_SL_ALGO_FROM_PHASE      3
/* 开环起始电角度。 */
#define STARTING_ANGLE_DEG             90  /* 单位电角度。 */
/* 允许STO参与启动收敛判断的最低机械转速。 */
#define OBS_MINIMUM_SPEED_RPM          580

#define NB_CONSECUTIVE_TESTS           2 /* 连续通过收敛测试的最少次数。 */
#define SPEED_BAND_UPPER_LIMIT         17 /* 估算速度相对强制速度的上边界比例。 */
#define SPEED_BAND_LOWER_LIMIT         15  /* 估算速度相对强制速度的下边界比例。 */

#define TRANSITION_DURATION            25  /* 虚拟角度向STO角度平滑过渡的控制周期数。 */
/** Motor 1母线电压ADC采样时间。 */
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(28)
/** 相电流ADC采样周期，包含平台校正量。 */
#define ADC_SAMPLING_CYCLES (28 + SAMPLING_CYCLE_CORRECTION)

/** 附加功能参数区。 */

/** 飞车启动功能参数区；本工程未启用。 */

/** 其他控制参数和用户扩展区。 */

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /* __DRIVE_PARAMETERS_H */
/************************ （C）版权所有，文件结束 ************************/
