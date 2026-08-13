/**
  ******************************************************************************
  * @file    mc_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __MC_INTERFACE_H
#define __MC_INTERFACE_H

/* 中文解读：MCI命令接口的数据结构与函数声明。该对象在前台API和中频状态机之间传递异步
 * 命令，并记录命令状态、控制模式、目标值和故障字。 */

#ifdef __cplusplus
extern "C" {
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 头文件 */
#include "mc_type.h"
#include "state_machine.h"
#include "speed_torq_ctrl.h"
/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCInterface
  * @{
  */

/* 导出类型 */
typedef enum
{
  MCI_BUFFER_EMPTY,                  /* 中文说明：处理并更新缓冲区。 */
  MCI_COMMAND_NOT_ALREADY_EXECUTED,  /* 中文说明：处理并更新缓冲区。 */
  MCI_COMMAND_EXECUTED_SUCCESFULLY,  /* 中文说明：处理并更新缓冲区。 */
  MCI_COMMAND_EXECUTED_UNSUCCESFULLY /* 中文说明：处理并更新缓冲区。 */
} MCI_CommandState_t ;

typedef enum
{
  MCI_NOCOMMANDSYET,        /* 中文说明：设置相关数据和控制状态。 */
  MCI_EXECSPEEDRAMP,        /* 中文说明：处理并更新速度、斜坡。 */
  MCI_EXECTORQUERAMP,       /* 中文说明：处理并更新转矩、斜坡。 */
  MCI_SETCURRENTREFERENCES, /* 中文说明：处理并更新相电流。 */
} MCI_UserCommands_t;

typedef struct
{
  STM_Handle_t * pSTM; /* 中文说明：处理并更新状态机状态。 */
  SpeednTorqCtrl_Handle_t * pSTC; /* 中文说明：处理并更新速度、转矩、PI/PID调节器。 */
  pFOCVars_t pFOCVars;    /* 中文说明：处理并更新相关数据和控制状态。 */
  MCI_UserCommands_t lastCommand; /* 中文说明：处理并更新相关数据和控制状态。 */
  int16_t hFinalSpeed;        /* 中文说明：处理并更新速度、斜坡。 */
  int16_t hFinalTorque;       /* 中文说明：处理并更新转矩、斜坡。 */
  qd_t Iqdref;     /* 中文说明：初始化相电流。 */
  uint16_t hDurationms;       /* 中文说明：初始化速度、转矩、斜坡。 */

  MCI_CommandState_t CommandState; /* 中文说明：初始化缓冲区。 */
  STC_Modality_t LastModalitySetByUser; /* 中文说明：初始化相关数据和控制状态。 */
} MCI_Handle_t;

/* 导出函数 */
void MCI_Init( MCI_Handle_t * pHandle, STM_Handle_t * pSTM, SpeednTorqCtrl_Handle_t * pSTC, pFOCVars_t pFOCVars );
void MCI_ExecBufferedCommands( MCI_Handle_t * pHandle );
void MCI_ExecSpeedRamp( MCI_Handle_t * pHandle,  int16_t hFinalSpeed, uint16_t hDurationms );
void MCI_ExecTorqueRamp( MCI_Handle_t * pHandle,  int16_t hFinalTorque, uint16_t hDurationms );
void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, qd_t Iqdref );

bool MCI_StartMotor( MCI_Handle_t * pHandle );
bool MCI_StopMotor( MCI_Handle_t * pHandle );
bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle );
bool MCI_EncoderAlign( MCI_Handle_t * pHandle );
MCI_CommandState_t  MCI_IsCommandAcknowledged( MCI_Handle_t * pHandle );
State_t  MCI_GetSTMState( MCI_Handle_t * pHandle );
uint16_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle );
uint16_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle );
int16_t MCI_GetMecSpeedRefUnit( MCI_Handle_t * pHandle );
int16_t MCI_GetAvrgMecSpeedUnit( MCI_Handle_t * pHandle );
/* 中文说明：读取并返回相电流、转矩。 */
int16_t MCI_GetPhaseCurrentAmplitude( MCI_Handle_t * pHandle );
int16_t MCI_GetPhaseVoltageAmplitude( MCI_Handle_t * pHandle );
STC_Modality_t MCI_GetControlMode( MCI_Handle_t * pHandle );
int16_t MCI_GetImposedMotorDirection( MCI_Handle_t * pHandle );
int16_t MCI_GetLastRampFinalSpeed( MCI_Handle_t * pHandle );
bool MCI_RampCompleted( MCI_Handle_t * pHandle );
bool MCI_StopSpeedRamp( MCI_Handle_t * pHandle );
void MCI_StopRamp( MCI_Handle_t * pHandle );
bool MCI_GetSpdSensorReliability( MCI_Handle_t * pHandle );
int16_t MCI_GetAvrgMecSpeedUnit( MCI_Handle_t * pHandle );
int16_t MCI_GetMecSpeedRefUnit( MCI_Handle_t * pHandle );
ab_t MCI_GetIab( MCI_Handle_t * pHandle );
alphabeta_t MCI_GetIalphabeta( MCI_Handle_t * pHandle );
qd_t MCI_GetIqd( MCI_Handle_t * pHandle );
qd_t MCI_GetIqdHF( MCI_Handle_t * pHandle );
qd_t MCI_GetIqdref( MCI_Handle_t * pHandle );
qd_t MCI_GetVqd( MCI_Handle_t * pHandle );
alphabeta_t MCI_GetValphabeta( MCI_Handle_t * pHandle );
int16_t MCI_GetElAngledpp( MCI_Handle_t * pHandle );
int16_t MCI_GetTeref( MCI_Handle_t * pHandle );
int16_t MCI_GetPhaseCurrentAmplitude( MCI_Handle_t * pHandle );
int16_t MCI_GetPhaseVoltageAmplitude( MCI_Handle_t * pHandle );
void MCI_SetIdref( MCI_Handle_t * pHandle, int16_t hNewIdref );
void MCI_Clear_Iqdref( MCI_Handle_t * pHandle );

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

