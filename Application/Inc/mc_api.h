/**
  ******************************************************************************
  * @file    mc_api.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：启动或使能电机。 */
#ifndef __MC_API_H
#define __MC_API_H

/* 中文解读：应用层唯一建议直接调用的电机控制API声明。速度内部单位由SPEED_UNIT/_RPM换算，
 * 故障接口区分“当前仍存在”和“历史发生过”两类故障。 */

#include "mc_type.h"
#include "mc_interface.h"
#include "state_machine.h"

#ifdef __cplusplus
 extern "C" {
#endif /* 中文说明：启动或使能电机。 */

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCIAPI
  * @{
  */

/* 中文说明：启动或使能电机。 */
bool MC_StartMotor1(void);

/* 中文说明：停止或禁用电机。 */
bool MC_StopMotor1(void);

/* 中文说明：设置速度、斜坡、电机。 */
void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms );

/* 中文说明：设置转矩、斜坡、电机。 */
void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms );

/* 中文说明：设置相电流、电机。 */
void MC_SetCurrentReferenceMotor1( qd_t Iqdref );

/* 中文说明：读取并返回状态机状态、电机。 */
MCI_CommandState_t  MC_GetCommandStateMotor1( void);

/* 中文说明：停止或禁用相电流、速度、斜坡、电机。 */
bool MC_StopSpeedRampMotor1(void);

/* 中文说明：停止或禁用斜坡、电机。 */
void MC_StopRampMotor1(void);

/* 中文说明：检查并判断斜坡、电机。 */
bool MC_HasRampCompletedMotor1(void);

/* 中文说明：设置相电流、速度、电机。 */
int16_t MC_GetMecSpeedReferenceMotor1(void);

/* 中文说明：读取并返回速度、电机。 */
int16_t MC_GetMecSpeedAverageMotor1(void);

/* 中文说明：读取并返回速度、斜坡、电机。 */
int16_t MC_GetLastRampFinalSpeedMotor1(void);

/* 中文说明：读取并返回相电流、速度、转矩、电机。 */
STC_Modality_t MC_GetControlModeMotor1(void);

/* 中文说明：读取并返回电机。 */
int16_t MC_GetImposedDirectionMotor1(void);

/* 中文说明：读取并返回相电流、速度、电机。 */
bool MC_GetSpeedSensorReliabilityMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
int16_t MC_GetPhaseCurrentAmplitudeMotor1(void);

/* 中文说明：读取并返回电压、电机。 */
int16_t MC_GetPhaseVoltageAmplitudeMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
ab_t MC_GetIabMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
alphabeta_t MC_GetIalphabetaMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
qd_t MC_GetIqdMotor1(void);

/* 中文说明：读取并返回电机。 */
qd_t MC_GetIqdrefMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
qd_t MC_GetVqdMotor1(void);

/* 中文说明：读取并返回相电流、电机。 */
alphabeta_t MC_GetValphabetaMotor1(void);

/* 中文说明：读取并返回位置和角度、电机。 */
int16_t MC_GetElAngledppMotor1(void);

/* 中文说明：读取并返回相电流、转矩、电机。 */
int16_t MC_GetTerefMotor1(void);

/* 中文说明：设置电机。 */
void MC_SetIdrefMotor1( int16_t hNewIdref );

/* 中文说明：初始化故障与保护、电机。 */
void MC_Clear_IqdrefMotor1(void);

/* 中文说明：处理并更新故障与保护、电机。 */
bool MC_AcknowledgeFaultMotor1( void );

/* 中文说明：读取并返回故障与保护、状态机状态、电机。 */
uint16_t MC_GetOccurredFaultsMotor1(void);

/* 中文说明：读取并返回相电流、故障与保护、电机。 */
uint16_t MC_GetCurrentFaultsMotor1(void);

/* 中文说明：读取并返回相电流、状态机状态、电机。 */
State_t  MC_GetSTMStateMotor1(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#endif /* 中文说明：处理并更新相关数据和控制状态。 */
/************************ （C）版权所有，文件结束 ************************/
