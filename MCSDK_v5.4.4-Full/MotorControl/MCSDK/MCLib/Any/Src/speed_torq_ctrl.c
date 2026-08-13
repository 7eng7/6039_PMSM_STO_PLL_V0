/** @file speed_torq_ctrl.c
 *  @brief 速度/转矩模式管理。速度模式由外环PI把速度误差变为Iq；转矩模式直接斜坡Iq参考，
 *  Id通常为0。目标受速度和额定电流范围限制，再由内电流环跟踪。 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

/* 头文件 */
#include "speed_torq_ctrl.h"
#include "speed_pos_fdbk.h"

#include "mc_type.h"

#define CHECK_BOUNDARY

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup SpeednTorqCtrl 速度、转矩、电机功能模块
  * @brief 速度、转矩、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  STC_Init：初始化相电流、速度、转矩、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  oPI  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  oSPD  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_Init( SpeednTorqCtrl_Handle_t * pHandle, PID_Handle_t * pPI, SpeednPosFdbk_Handle_t * SPD_Handle )
{

  pHandle->PISpeed = pPI;
  pHandle->SPD = SPD_Handle;
  pHandle->Mode = pHandle->ModeDefault;
  pHandle->SpeedRefUnitExt = ( int32_t )pHandle->MecSpeedRefUnitDefault * 65536;
  pHandle->TorqueRef = ( int32_t )pHandle->TorqueRefDefault * 65536;
  pHandle->TargetFinal = 0;
  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
}

/**
  * @brief  STC_SetSpeedSensor：设置相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  SPD_Handle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_SetSpeedSensor( SpeednTorqCtrl_Handle_t * pHandle, SpeednPosFdbk_Handle_t * SPD_Handle )
{
  pHandle->SPD = SPD_Handle;
}

/**
  * @brief  STC_GetSpeedSensor：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK SpeednPosFdbk_Handle_t * STC_GetSpeedSensor( SpeednTorqCtrl_Handle_t * pHandle )
{
  return ( pHandle->SPD );
}

/**
  * @brief  STC_Clear：清除并复位相电流、速度、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_Clear( SpeednTorqCtrl_Handle_t * pHandle )
{
  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    PID_SetIntegralTerm( pHandle->PISpeed, 0 );
  }
}

/**
  * @brief  STC_GetMecSpeedRefUnit：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t STC_GetMecSpeedRefUnit( SpeednTorqCtrl_Handle_t * pHandle )
{
  return ( ( int16_t )( pHandle->SpeedRefUnitExt / 65536 ) );
}

/**
  * @brief  STC_GetTorqueRef：读取并返回相电流、速度、转矩、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t STC_GetTorqueRef( SpeednTorqCtrl_Handle_t * pHandle )
{
  return ( ( int16_t )( pHandle->TorqueRef / 65536 ) );
}

/**
  * @brief  STC_SetControlMode：设置相电流、速度、转矩、斜坡、PI/PID调节器、中断、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bMode  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_SetControlMode( SpeednTorqCtrl_Handle_t * pHandle, STC_Modality_t bMode )
{
  pHandle->Mode = bMode;
  pHandle->RampRemainingStep = 0u; /* 中文说明：读取并返回斜坡、中断。 */
}

/**
  * @brief  STC_GetControlMode：读取并返回相电流、速度、转矩、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK STC_Modality_t STC_GetControlMode( SpeednTorqCtrl_Handle_t * pHandle )
{
  return pHandle->Mode;
}

/**
  * @brief  STC_ExecRamp：设置相电流、速度、转矩、斜坡、中断、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hTargetFinal  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STC_ExecRamp( SpeednTorqCtrl_Handle_t * pHandle, int16_t hTargetFinal, uint32_t hDurationms )
{
  bool AllowedRange = true;
  uint32_t wAux;
  int32_t wAux1;
  int16_t hCurrentReference;

  /* 中文说明：检查并判断相关数据和控制状态。 */
  if ( pHandle->Mode == STC_TORQUE_MODE )
  {
    hCurrentReference = STC_GetTorqueRef( pHandle );
#ifdef CHECK_BOUNDARY
    if ( ( int32_t )hTargetFinal > ( int32_t )pHandle->MaxPositiveTorque )
    {
      AllowedRange = false;
    }
    if ( ( int32_t )hTargetFinal < ( int32_t )pHandle->MinNegativeTorque )
    {
      AllowedRange = false;
    }
#endif
  }
  else
  {
    hCurrentReference = ( int16_t )( pHandle->SpeedRefUnitExt / 65536 );

#ifdef CHECK_BOUNDARY
    if ( ( int32_t )hTargetFinal > ( int32_t )pHandle->MaxAppPositiveMecSpeedUnit )
    {
      AllowedRange = false;
    }
    else if ( hTargetFinal < pHandle->MinAppNegativeMecSpeedUnit )
    {
      AllowedRange = false;
    }
    else if ( ( int32_t )hTargetFinal < ( int32_t )pHandle->MinAppPositiveMecSpeedUnit )
    {
      if ( hTargetFinal > pHandle->MaxAppNegativeMecSpeedUnit )
      {
        AllowedRange = false;
      }
    }
    else {}
#endif
  }

  if ( AllowedRange == true )
  {
    /* 中文说明：处理并更新斜坡、中断。 */
    if ( hDurationms == 0u )
    {
      if ( pHandle->Mode == STC_SPEED_MODE )
      {
        pHandle->SpeedRefUnitExt = ( int32_t )hTargetFinal * 65536;
      }
      else
      {
        pHandle->TorqueRef = ( int32_t )hTargetFinal * 65536;
      }
      pHandle->RampRemainingStep = 0u;
      pHandle->IncDecAmount = 0;
    }
    else
    {
      /* 中文说明：处理并更新相关数据和控制状态。 */
      pHandle->TargetFinal = hTargetFinal;

      /* 中文说明：计算并更新斜坡。 */
      wAux = ( uint32_t )hDurationms * ( uint32_t )pHandle->STCFrequencyHz;
      wAux /= 1000u;
      pHandle->RampRemainingStep = wAux;
      pHandle->RampRemainingStep++;

      /* 中文说明：计算并更新转矩。 */
      wAux1 = ( ( int32_t )hTargetFinal - ( int32_t )hCurrentReference ) * 65536;
      wAux1 /= ( int32_t )pHandle->RampRemainingStep;
      pHandle->IncDecAmount = wAux1;
    }
  }

  return AllowedRange;
}

/**
  * @brief  STC_StopRamp：设置相电流、速度、转矩、斜坡、中断，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_StopRamp( SpeednTorqCtrl_Handle_t * pHandle )
{

  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
}

/**
  * @brief  STC_CalcTorqueReference：计算并更新相电流、速度、转矩、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t STC_CalcTorqueReference( SpeednTorqCtrl_Handle_t * pHandle )
{
  int32_t wCurrentReference;
  int16_t hTorqueReference = 0;
  int16_t hMeasuredSpeed;
  int16_t hTargetSpeed;
  int16_t hError;

  if ( pHandle->Mode == STC_TORQUE_MODE )
  {
    wCurrentReference = pHandle->TorqueRef;
  }
  else
  {
    wCurrentReference = pHandle->SpeedRefUnitExt;
  }

  /* 中文说明：处理并更新速度、转矩、斜坡。 */
  if ( pHandle->RampRemainingStep > 1u )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    wCurrentReference += pHandle->IncDecAmount;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    pHandle->RampRemainingStep--;
  }
  else if ( pHandle->RampRemainingStep == 1u )
  {
    /* 中文说明：设置相关数据和控制状态。 */
    wCurrentReference = ( int32_t )pHandle->TargetFinal * 65536;
    pHandle->RampRemainingStep = 0u;
  }
  else
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
  }

  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    /* 中文说明：执行并推进速度。 */

    /* 中文说明：计算并更新速度、故障与保护。 */
    hTargetSpeed = ( int16_t )( wCurrentReference / 65536 );
    hMeasuredSpeed = SPD_GetAvrgMecSpeedUnit( pHandle->SPD );
    hError = hTargetSpeed - hMeasuredSpeed;
    hTorqueReference = PI_Controller( pHandle->PISpeed, ( int32_t )hError );

    pHandle->SpeedRefUnitExt = wCurrentReference;
    pHandle->TorqueRef = ( int32_t )hTorqueReference * 65536;
  }
  else
  {
    pHandle->TorqueRef = wCurrentReference;
    hTorqueReference = ( int16_t )( wCurrentReference / 65536 );
  }

  return hTorqueReference;
}

/**
  * @brief  STC_GetMecSpeedRefUnitDefault：读取并返回相电流、速度、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t STC_GetMecSpeedRefUnitDefault( SpeednTorqCtrl_Handle_t * pHandle )
{
  return pHandle->MecSpeedRefUnitDefault;
}

/**
  * @brief  STC_GetMaxAppPositiveMecSpeedUnit：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK uint16_t STC_GetMaxAppPositiveMecSpeedUnit( SpeednTorqCtrl_Handle_t * pHandle )
{
  return pHandle->MaxAppPositiveMecSpeedUnit;
}

/**
  * @brief  STC_GetMinAppNegativeMecSpeedUnit：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK int16_t STC_GetMinAppNegativeMecSpeedUnit( SpeednTorqCtrl_Handle_t * pHandle )
{
  return pHandle->MinAppNegativeMecSpeedUnit;
}

/**
  * @brief  STC_RampCompleted：检查并判断相电流、速度、转矩、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STC_RampCompleted( SpeednTorqCtrl_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->RampRemainingStep == 0u )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  STC_StopSpeedRamp：停止或禁用相电流、速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STC_StopSpeedRamp( SpeednTorqCtrl_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->Mode == STC_SPEED_MODE )
  {
    pHandle->RampRemainingStep = 0u;
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  STC_GetDefaultIqdref：读取并返回相电流、速度、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t STC_GetDefaultIqdref( SpeednTorqCtrl_Handle_t * pHandle )
{
  qd_t IqdRefDefault;
  IqdRefDefault.q = pHandle->TorqueRefDefault;
  IqdRefDefault.d = pHandle->IdrefDefault;
  return IqdRefDefault;
}

/**
  * @brief  STC_SetNominalCurrent：设置相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hNominalCurrent  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_SetNominalCurrent( SpeednTorqCtrl_Handle_t * pHandle, uint16_t hNominalCurrent )
{
  pHandle->MaxPositiveTorque = hNominalCurrent;
  pHandle->MinNegativeTorque = -hNominalCurrent;
}

/**
  * @brief  STC_ForceSpeedReferenceToCurrentSpeed：初始化相电流、速度、状态机状态，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STC_ForceSpeedReferenceToCurrentSpeed( SpeednTorqCtrl_Handle_t * pHandle )
{
  pHandle->SpeedRefUnitExt = ( int32_t )SPD_GetAvrgMecSpeedUnit( pHandle->SPD ) * ( int32_t )65536;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
