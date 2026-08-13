/**
 * @file mc_interface.c
 * @brief 应用命令与速度/转矩控制器之间的命令队列和状态接口。
 *
 * 中文解读：MCI对象保存待执行的启动/停止命令、速度或转矩斜坡以及最近一次命令结果，
 * 中频任务在安全状态下消费命令。函数返回“命令已登记”不等于转子已达到目标转速，
 * 应用还应结合状态和故障接口判断最终结果。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
/* 头文件 */
#include "mc_math.h"
#include "speed_torq_ctrl.h"

#include "mc_interface.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup MCInterface 电机功能模块
  * @brief 电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
/**
  * @brief  当前接口：处理并更新状态机状态，保持对象状态和控制流程一致。
  */
#define BC(state) (1u<<((uint16_t)((uint8_t)(state))))

/* 函数 */

/**
  * @brief  MCI_Init：初始化速度、转矩、状态机状态、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pSTM  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pSTC  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pFOCVars  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_Init( MCI_Handle_t * pHandle, STM_Handle_t * pSTM, SpeednTorqCtrl_Handle_t * pSTC, pFOCVars_t pFOCVars )
{
  pHandle->pSTM = pSTM;
  pHandle->pSTC = pSTC;
  pHandle->pFOCVars = pFOCVars;
// 中文说明：处理并更新相关数据和控制状态。

  /* 中文说明：初始化缓冲区。 */
  pHandle->lastCommand = MCI_NOCOMMANDSYET;
  pHandle->hFinalSpeed = 0;
  pHandle->hFinalTorque = 0;
  pHandle->hDurationms = 0;
  pHandle->CommandState = MCI_BUFFER_EMPTY;
}

/**
  * @brief  MCI_ExecSpeedRamp：设置速度、状态机状态、斜坡、缓冲区、调试DAC、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalSpeed  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_ExecSpeedRamp( MCI_Handle_t * pHandle,  int16_t hFinalSpeed, uint16_t hDurationms )
{
  pHandle->lastCommand = MCI_EXECSPEEDRAMP;
  pHandle->hFinalSpeed = hFinalSpeed;
  pHandle->hDurationms = hDurationms;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_SPEED_MODE;
}

/**
  * @brief  MCI_ExecTorqueRamp：设置相电流、转矩、状态机状态、斜坡、缓冲区、调试DAC、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalTorque  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_ExecTorqueRamp( MCI_Handle_t * pHandle,  int16_t hFinalTorque, uint16_t hDurationms )
{
  pHandle->lastCommand = MCI_EXECTORQUERAMP;
  pHandle->hFinalTorque = hFinalTorque;
  pHandle->hDurationms = hDurationms;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
  * @brief  MCI_SetCurrentReferences：设置相电流、状态机状态、缓冲区、串口通信帧、调试DAC、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Iqdref  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_SetCurrentReferences( MCI_Handle_t * pHandle, qd_t Iqdref )
{
  pHandle->lastCommand = MCI_SETCURRENTREFERENCES;
  pHandle->Iqdref.q = Iqdref.q;
  pHandle->Iqdref.d = Iqdref.d;
  pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  pHandle->LastModalitySetByUser = STC_TORQUE_MODE;
}

/**
  * @brief  MCI_StartMotor：检查并判断相电流、速度、转矩、状态机状态、斜坡、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_StartMotor( MCI_Handle_t * pHandle )
{
  bool RetVal = STM_NextState( pHandle->pSTM, IDLE_START );

  if ( RetVal == true )
  {
    pHandle->CommandState = MCI_COMMAND_NOT_ALREADY_EXECUTED;
  }

  return RetVal;
}

/**
  * @brief  MCI_StopMotor：检查并判断状态机状态、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_StopMotor( MCI_Handle_t * pHandle )
{
  return STM_NextState( pHandle->pSTM, ANY_STOP );
}

/**
  * @brief  MCI_FaultAcknowledged：处理并更新故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_FaultAcknowledged( MCI_Handle_t * pHandle )
{
  return STM_FaultAcknowledged( pHandle->pSTM );
}

/**
  * @brief  MCI_EncoderAlign：检查并判断状态机状态、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_EncoderAlign( MCI_Handle_t * pHandle )
{
  return STM_NextState( pHandle->pSTM, IDLE_ALIGNMENT );
}

/**
  * @brief  MCI_ExecBufferedCommands：检查并判断缓冲区，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_ExecBufferedCommands( MCI_Handle_t * pHandle )
{
  if ( pHandle != MC_NULL )
  {
    if ( pHandle->CommandState == MCI_COMMAND_NOT_ALREADY_EXECUTED )
    {
      bool commandHasBeenExecuted = false;
      switch ( pHandle->lastCommand )
      {
        case MCI_EXECSPEEDRAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode( pHandle->pSTC, STC_SPEED_MODE );
          commandHasBeenExecuted = STC_ExecRamp( pHandle->pSTC, pHandle->hFinalSpeed, pHandle->hDurationms );
        }
        break;
        case MCI_EXECTORQUERAMP:
        {
          pHandle->pFOCVars->bDriveInput = INTERNAL;
          STC_SetControlMode( pHandle->pSTC, STC_TORQUE_MODE );
          commandHasBeenExecuted = STC_ExecRamp( pHandle->pSTC, pHandle->hFinalTorque, pHandle->hDurationms );
        }
        break;
        case MCI_SETCURRENTREFERENCES:
        {
          pHandle->pFOCVars->bDriveInput = EXTERNAL;
          pHandle->pFOCVars->Iqdref = pHandle->Iqdref;
          commandHasBeenExecuted = true;
        }
        break;
        default:
          break;
      }

      if ( commandHasBeenExecuted )
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_SUCCESFULLY;
      }
      else
      {
        pHandle->CommandState = MCI_COMMAND_EXECUTED_UNSUCCESFULLY;
      }
    }
  }
}

/**
  * @brief  MCI_IsCommandAcknowledged：清除并复位状态机状态、缓冲区、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK MCI_CommandState_t  MCI_IsCommandAcknowledged( MCI_Handle_t * pHandle )
{
  MCI_CommandState_t retVal = pHandle->CommandState;

  if ( ( retVal == MCI_COMMAND_EXECUTED_SUCCESFULLY ) |
       ( retVal == MCI_COMMAND_EXECUTED_UNSUCCESFULLY ) )
  {
    pHandle->CommandState = MCI_BUFFER_EMPTY;
  }
  return retVal;
}

/**
  * @brief  MCI_GetSTMState：读取并返回相电流、状态机状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK State_t  MCI_GetSTMState( MCI_Handle_t * pHandle )
{
  return STM_GetState( pHandle->pSTM );
}

/**
  * @brief  MCI_GetOccurredFaults：读取并返回故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t MCI_GetOccurredFaults( MCI_Handle_t * pHandle )
{
  return ( uint16_t )( STM_GetFaultState( pHandle->pSTM ) );
}

/**
  * @brief  MCI_GetCurrentFaults：读取并返回相电流、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t MCI_GetCurrentFaults( MCI_Handle_t * pHandle )
{
  return ( uint16_t )( STM_GetFaultState( pHandle->pSTM ) >> 16 );
}

/**
  * @brief  MCI_GetControlMode：读取并返回速度、转矩、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK STC_Modality_t MCI_GetControlMode( MCI_Handle_t * pHandle )
{
  return pHandle->LastModalitySetByUser;
}

/**
  * @brief  MCI_GetImposedMotorDirection：读取并返回相电流、速度、转矩、斜坡、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetImposedMotorDirection( MCI_Handle_t * pHandle )
{
  int16_t retVal = 1;

  switch ( pHandle->lastCommand )
  {
    case MCI_EXECSPEEDRAMP:
      if ( pHandle->hFinalSpeed < 0 )
      {
        retVal = -1;
      }
      break;
    case MCI_EXECTORQUERAMP:
      if ( pHandle->hFinalTorque < 0 )
      {
        retVal = -1;
      }
      break;
    case MCI_SETCURRENTREFERENCES:
      if ( pHandle->Iqdref.q < 0 )
      {
        retVal = -1;
      }
      break;
    default:
      break;
  }
  return retVal;
}

/**
  * @brief  MCI_GetLastRampFinalSpeed：读取并返回速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetLastRampFinalSpeed( MCI_Handle_t * pHandle )
{
  int16_t hRetVal = 0;

  /* 中文说明：处理并更新斜坡、缓冲区。 */
  if ( pHandle->lastCommand == MCI_EXECSPEEDRAMP )
  {
    hRetVal = pHandle->hFinalSpeed;
  }
  return hRetVal;
}

/**
  * @brief  MCI_RampCompleted：检查并判断速度、转矩、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_RampCompleted( MCI_Handle_t * pHandle )
{
  bool retVal = false;

  if ( ( STM_GetState( pHandle->pSTM ) ) == RUN )
  {
    retVal = STC_RampCompleted( pHandle->pSTC );
  }

  return retVal;
}

/**
  * @brief  MCI_StopSpeedRamp：停止或禁用速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_StopSpeedRamp( MCI_Handle_t * pHandle )
{
  return STC_StopSpeedRamp( pHandle->pSTC );
}

/**
  * @brief  MCI_StopRamp：停止或禁用斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCI_StopRamp( MCI_Handle_t * pHandle)
{
   STC_StopRamp( pHandle->pSTC );
}

/**
  * @brief  MCI_GetSpdSensorReliability：读取并返回速度、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool MCI_GetSpdSensorReliability( MCI_Handle_t * pHandle )
{
  SpeednPosFdbk_Handle_t * SpeedSensor = STC_GetSpeedSensor( pHandle->pSTC );

  return ( SPD_Check( SpeedSensor ) );
}

/**
  * @brief  MCI_GetAvrgMecSpeedUnit：读取并返回速度，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK int16_t MCI_GetAvrgMecSpeedUnit( MCI_Handle_t * pHandle )
{
  SpeednPosFdbk_Handle_t * SpeedSensor = STC_GetSpeedSensor( pHandle->pSTC );

  return ( SPD_GetAvrgMecSpeedUnit( SpeedSensor ) );
}

/**
  * @brief  MCI_GetMecSpeedRefUnit：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK int16_t MCI_GetMecSpeedRefUnit( MCI_Handle_t * pHandle )
{
  return ( STC_GetMecSpeedRefUnit( pHandle->pSTC ) );
}

/**
  * @brief  MCI_GetIab：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK ab_t MCI_GetIab( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iab );
}

/**
  * @brief  MCI_GetIalphabeta：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK alphabeta_t MCI_GetIalphabeta( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Ialphabeta );
}

/**
  * @brief  MCI_GetIqd：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t MCI_GetIqd( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iqd );
}

/**
  * @brief  MCI_GetIqdHF：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t MCI_GetIqdHF( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->IqdHF );
}

/**
  * @brief  MCI_GetIqdref：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t MCI_GetIqdref( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Iqdref );
}

/**
  * @brief  MCI_GetVqd：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t MCI_GetVqd( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Vqd );
}

/**
  * @brief  MCI_GetValphabeta：读取并返回相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK alphabeta_t MCI_GetValphabeta( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->Valphabeta );
}

/**
  * @brief  MCI_GetElAngledpp：读取并返回位置和角度、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetElAngledpp( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->hElAngle );
}

/**
  * @brief  MCI_GetTeref：读取并返回转矩，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetTeref( MCI_Handle_t * pHandle )
{
  return ( pHandle->pFOCVars->hTeref );
}

/**
  * @brief  MCI_GetPhaseCurrentAmplitude：读取并返回相电流、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetPhaseCurrentAmplitude( MCI_Handle_t * pHandle )
{
  alphabeta_t Local_Curr;
  int32_t wAux1, wAux2;

  Local_Curr = pHandle->pFOCVars->Ialphabeta;
  wAux1 = ( int32_t )( Local_Curr.alpha ) * Local_Curr.alpha;
  wAux2 = ( int32_t )( Local_Curr.beta ) * Local_Curr.beta;

  wAux1 += wAux2;
  wAux1 = MCM_Sqrt( wAux1 );

  if ( wAux1 > INT16_MAX )
  {
    wAux1 = ( int32_t ) INT16_MAX;
  }

  return ( ( int16_t )wAux1 );
}

/**
  * @brief  MCI_GetPhaseVoltageAmplitude：读取并返回电压、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MCI_GetPhaseVoltageAmplitude( MCI_Handle_t * pHandle )
{
  alphabeta_t Local_Voltage;
  int32_t wAux1, wAux2;

  Local_Voltage = pHandle->pFOCVars->Valphabeta;
  wAux1 = ( int32_t )( Local_Voltage.alpha ) * Local_Voltage.alpha;
  wAux2 = ( int32_t )( Local_Voltage.beta ) * Local_Voltage.beta;

  wAux1 += wAux2;
  wAux1 = MCM_Sqrt( wAux1 );

  if ( wAux1 > INT16_MAX )
  {
    wAux1 = ( int32_t ) INT16_MAX;
  }

  return ( ( int16_t ) wAux1 );
}

/**
  * @brief  MCI_SetIdref：设置相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  int16_t  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_SetIdref( MCI_Handle_t * pHandle, int16_t hNewIdref )
{
  pHandle->pFOCVars->Iqdref.d = hNewIdref;
  pHandle->pFOCVars->UserIdref = hNewIdref;
}

/**
  * @brief  MCI_Clear_Iqdref：初始化故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCI_Clear_Iqdref( MCI_Handle_t * pHandle )
{
  pHandle->pFOCVars->Iqdref = STC_GetDefaultIqdref( pHandle->pSTC );
}


