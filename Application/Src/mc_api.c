/**
 * @file mc_api.c
 * @brief 面向应用层的电机控制 API。
 *
 * 中文解读：提供启动、停止、速度/转矩斜坡、故障读取与确认等接口。API通常只写入MCI
 * 命令缓冲区或读取状态，不直接操作PWM；命令是否执行取决于 mc_tasks.c 当前状态。
 * 例如故障只能在FAULT_OVER状态确认，确认后仍需重新提交启动命令。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

#include "mc_interface.h"
#include "mc_api.h"
#include "mc_config.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup MCIAPI 相电流、速度、转矩、状态机状态、斜坡、电机功能模块
  * @brief 相电流、速度、转矩、状态机状态、斜坡、电机相关组件及其公共接口。
  * @{
  */

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

/**
  * @brief  MC_StartMotor1：检查并判断相电流、速度、转矩、状态机状态、斜坡、电机，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK bool MC_StartMotor1(void)
{
	return MCI_StartMotor( pMCI[M1] );
}

/**
  * @brief  MC_StopMotor1：检查并判断状态机状态、电机，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK bool MC_StopMotor1(void)
{
	return MCI_StopMotor( pMCI[M1] );
}

/**
  * @brief  MC_ProgramSpeedRampMotor1：设置相电流、速度、转矩、故障与保护、状态机状态、斜坡、缓冲区、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  hFinalSpeed  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK void MC_ProgramSpeedRampMotor1( int16_t hFinalSpeed, uint16_t hDurationms )
{
	MCI_ExecSpeedRamp( pMCI[M1], hFinalSpeed, hDurationms );
}

/**
  * @brief  MC_ProgramTorqueRampMotor1：设置相电流、速度、转矩、故障与保护、状态机状态、斜坡、缓冲区、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  hFinalTorque  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK void MC_ProgramTorqueRampMotor1( int16_t hFinalTorque, uint16_t hDurationms )
{
	MCI_ExecTorqueRamp( pMCI[M1], hFinalTorque, hDurationms );
}

/**
  * @brief  MC_SetCurrentReferenceMotor1：设置相电流、状态机状态、缓冲区、串口通信帧、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  Iqdref  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MC_SetCurrentReferenceMotor1( qd_t Iqdref )
{
	MCI_SetCurrentReferences( pMCI[M1], Iqdref );
}

/**
  * @brief  MC_GetCommandStateMotor1：清除并复位相电流、状态机状态、缓冲区、电机，保持对象状态和控制流程一致。
  */
__WEAK MCI_CommandState_t  MC_GetCommandStateMotor1( void)
{
	return MCI_IsCommandAcknowledged( pMCI[M1] );
}

/**
  * @brief  MC_StopSpeedRampMotor1：停止或禁用相电流、速度、斜坡、电机，保持对象状态和控制流程一致。
  */
__WEAK bool MC_StopSpeedRampMotor1(void)
{
	return MCI_StopSpeedRamp( pMCI[M1] );
}

/**
  * @brief  MC_StopRampMotor1：停止或禁用相电流、速度、转矩、斜坡、电机，保持对象状态和控制流程一致。
  */
__WEAK void MC_StopRampMotor1(void)
{
  MCI_StopRamp( pMCI[M1] );
}

/**
  * @brief  MC_HasRampCompletedMotor1：检查并判断斜坡、电机，保持对象状态和控制流程一致。
  */
__WEAK bool MC_HasRampCompletedMotor1(void)
{
	return MCI_RampCompleted( pMCI[M1] );
}

/**
  * @brief  MC_GetMecSpeedReferenceMotor1：设置相电流、速度、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetMecSpeedReferenceMotor1(void)
{
	return MCI_GetMecSpeedRefUnit( pMCI[M1] );
}

/**
  * @brief  MC_GetMecSpeedAverageMotor1：读取并返回速度、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetMecSpeedAverageMotor1(void)
{
	return MCI_GetAvrgMecSpeedUnit( pMCI[M1] );
}

/**
  * @brief  MC_GetLastRampFinalSpeedMotor1：读取并返回速度、斜坡、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetLastRampFinalSpeedMotor1(void)
{
	return MCI_GetLastRampFinalSpeed( pMCI[M1] );
}

/**
  * @brief  MC_GetControlModeMotor1：读取并返回速度、转矩、电机，保持对象状态和控制流程一致。
  */
__WEAK STC_Modality_t MC_GetControlModeMotor1(void)
{
	return MCI_GetControlMode( pMCI[M1] );
}

/**
  * @brief  MC_GetImposedDirectionMotor1：读取并返回相电流、速度、转矩、斜坡、电机，保持对象状态和控制流程一致。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK int16_t MC_GetImposedDirectionMotor1(void)
{
	return MCI_GetImposedMotorDirection( pMCI[M1] );
}

/**
  * @brief  MC_GetSpeedSensorReliabilityMotor1：读取并返回速度、电机，保持对象状态和控制流程一致。
  */
__WEAK bool MC_GetSpeedSensorReliabilityMotor1(void)
{
	return MCI_GetSpdSensorReliability( pMCI[M1] );
}

/**
  * @brief  MC_GetPhaseCurrentAmplitudeMotor1：读取并返回相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetPhaseCurrentAmplitudeMotor1(void)
{
	return MCI_GetPhaseCurrentAmplitude( pMCI[M1] );
}

/**
  * @brief  MC_GetPhaseVoltageAmplitudeMotor1：读取并返回电压、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetPhaseVoltageAmplitudeMotor1(void)
{
	return MCI_GetPhaseVoltageAmplitude( pMCI[M1] );
}

/**
  * @brief  MC_GetIabMotor1：读取并返回相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK ab_t MC_GetIabMotor1(void)
{
	return MCI_GetIab( pMCI[M1] );
}

/**
  * @brief  MC_GetIalphabetaMotor1：读取并返回相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK alphabeta_t MC_GetIalphabetaMotor1(void)
{
	return MCI_GetIalphabeta( pMCI[M1] );
}

/**
  * @brief  MC_GetIqdMotor1：读取并返回相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK qd_t MC_GetIqdMotor1(void)
{
	return MCI_GetIqd( pMCI[M1] );
}

/**
  * @brief  MC_GetIqdrefMotor1：读取并返回相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK qd_t MC_GetIqdrefMotor1(void)
{
	return MCI_GetIqdref( pMCI[M1] );
}

/**
  * @brief  MC_GetVqdMotor1：读取并返回电压、电机，保持对象状态和控制流程一致。
  */
__WEAK qd_t MC_GetVqdMotor1(void)
{
	return MCI_GetVqd( pMCI[M1] );
}

/**
  * @brief  MC_GetValphabetaMotor1：读取并返回电压、电机，保持对象状态和控制流程一致。
  */
__WEAK alphabeta_t MC_GetValphabetaMotor1(void)
{
	return MCI_GetValphabeta( pMCI[M1] );
}

/**
  * @brief  MC_GetElAngledppMotor1：读取并返回位置和角度、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetElAngledppMotor1(void)
{
	return MCI_GetElAngledpp( pMCI[M1] );
}

/**
  * @brief  MC_GetTerefMotor1：读取并返回转矩、电机，保持对象状态和控制流程一致。
  */
__WEAK int16_t MC_GetTerefMotor1(void)
{
	return MCI_GetTeref( pMCI[M1] );
}

/**
  * @brief  MC_SetIdrefMotor1：设置相电流、电机，保持对象状态和控制流程一致。
  */
__WEAK void MC_SetIdrefMotor1( int16_t hNewIdref )
{
	MCI_SetIdref( pMCI[M1], hNewIdref );
}

/**
  * @brief  MC_Clear_IqdrefMotor1：初始化速度、转矩、故障与保护、PI/PID调节器、电机，保持对象状态和控制流程一致。
  */
__WEAK void MC_Clear_IqdrefMotor1(void)
{
	MCI_Clear_Iqdref( pMCI[M1] );
}

/**
  * @brief  MC_AcknowledgeFaultMotor1：处理并更新故障与保护、状态机状态、电机，保持对象状态和控制流程一致。
  */
__WEAK bool MC_AcknowledgeFaultMotor1( void )
{
	return MCI_FaultAcknowledged( pMCI[M1] );
}

/**
  * @brief  MC_GetOccurredFaultsMotor1：读取并返回故障与保护、状态机状态、电机，保持对象状态和控制流程一致。
  */
__WEAK uint16_t MC_GetOccurredFaultsMotor1(void)
{
	return MCI_GetOccurredFaults( pMCI[M1] );
}

/**
  * @brief  MC_GetCurrentFaultsMotor1：读取并返回相电流、故障与保护、电机，保持对象状态和控制流程一致。
  */
__WEAK uint16_t MC_GetCurrentFaultsMotor1(void)
{
	return MCI_GetCurrentFaults( pMCI[M1] );
}

/**
  * @brief  MC_GetSTMStateMotor1：读取并返回相电流、状态机状态、电机，保持对象状态和控制流程一致。
  */
__WEAK State_t  MC_GetSTMStateMotor1(void)
{
	return MCI_GetSTMState( pMCI[M1] );
}

