/**
  ******************************************************************************
  * @file    mc_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

/* 中文解读：导出mc_config.c中实例化的全部控制对象，使任务层能访问PI、速度传感器、
 * STO-PLL、PWM电流采样、状态机和功率测量句柄。 */

#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"
 #include "user_interface.h"

#include "motor_control_protocol.h"
#include "usart_frame_communication_protocol.h"

#include "dac_common_ui.h"
#include "dac_ui.h"

#include "r3_2_f4xx_pwm_curr_fdbk.h"

extern RevUpCtrl_Handle_t RevUpControlM1;
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"

#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
/* 用户代码开始 */

/* 用户代码结束 */
extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensorParamsM1;
extern PWMC_R3_2_Handle_t PWM_Handle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern STO_Handle_t STO_M1;
extern STO_PLL_Handle_t STO_PLL_M1;
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
extern UI_Handle_t UI_Params;

extern DAC_UI_Handle_t DAC_UI_Params;

extern UFCP_Handle_t pUSART;

/* 用户代码开始 */

/* 用户代码结束 */
#define NBR_OF_MOTORS 1
#endif /* 中文说明：处理并更新相关数据和控制状态。 */
/************************ （C）版权所有，文件结束 ************************/
