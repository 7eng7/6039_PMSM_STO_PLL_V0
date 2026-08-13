/** @file speed_pos_fdbk.c
 *  @brief 速度/位置传感器基类。统一机械速度、电角度、加速度和可靠性接口；电角度等于机械角度
 *  乘极对数，连续越界或测量误差会标记传感器不可靠并触发停机。 */
/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "speed_pos_fdbk.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup SpeednPosFdbk 速度、位置和角度、状态机状态、观测器、反电势和PLL、电机功能模块
  * @brief 速度、位置和角度、状态机状态、观测器、反电势和PLL、电机相关组件及其公共接口。
  * @{
  */

/**
  * @brief  SPD_GetElAngle：读取并返回相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t SPD_GetElAngle( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hElAngle );
}

/**
  * @brief  SPD_GetMecAngle：读取并返回相电流、速度、位置和角度、斜坡、串口通信帧、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK int32_t SPD_GetMecAngle( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->wMecAngle );
}

/**
  * @brief  SPD_GetAvrgMecSpeedUnit：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK int16_t SPD_GetAvrgMecSpeedUnit( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hAvrMecSpeedUnit );
}

/**
  * @brief  SPD_GetElSpeedDpp：读取并返回相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t SPD_GetElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->hElSpeedDpp );
}

/**
  * @brief  SPD_GetInstElSpeedDpp：读取并返回相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t SPD_GetInstElSpeedDpp( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->InstantaneousElSpeedDpp );
}

/**
  * @brief  SPD_Check：检查并判断相电流、速度、故障与保护、状态机状态、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool SPD_Check( SpeednPosFdbk_Handle_t * pHandle )
{
  bool SpeedSensorReliability = true;
  if ( pHandle->bSpeedErrorNumber ==
       pHandle->bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }
  return ( SpeedSensorReliability );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  SPD_IsMecSpeedReliable：读取并返回相电流、速度、故障与保护、状态机状态、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool SPD_IsMecSpeedReliable( SpeednPosFdbk_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
  bool SpeedSensorReliability = true;
  uint8_t bSpeedErrorNumber;
  uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;

  bool SpeedError = false;
  uint16_t hAbsMecSpeedUnit, hAbsMecAccelUnitP;
  int16_t hAux;

  bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

  /* 中文说明：计算并更新速度。 */
  if ( *pMecSpeedUnit < 0 )
  {
    hAux = -( *pMecSpeedUnit );
    hAbsMecSpeedUnit = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecSpeedUnit = ( uint16_t )( *pMecSpeedUnit );
  }

  if ( hAbsMecSpeedUnit > pHandle->hMaxReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  if ( hAbsMecSpeedUnit < pHandle->hMinReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  /* 中文说明：计算并更新斜坡。 */
  if ( pHandle->hMecAccelUnitP < 0 )
  {
    hAux = -( pHandle->hMecAccelUnitP );
    hAbsMecAccelUnitP = ( uint16_t )( hAux );
  }
  else
  {
    hAbsMecAccelUnitP = ( uint16_t )( pHandle->hMecAccelUnitP );
  }

  if ( hAbsMecAccelUnitP > pHandle->hMaxReliableMecAccelUnitP )
  {
    SpeedError = true;
  }

  if ( SpeedError == true )
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber++;
    }
  }
  else
  {
    if ( bSpeedErrorNumber < bMaximumSpeedErrorsNumber )
    {
      bSpeedErrorNumber = 0u;
    }
  }

  if ( bSpeedErrorNumber == bMaximumSpeedErrorsNumber )
  {
    SpeedSensorReliability = false;
  }

  pHandle->bSpeedErrorNumber = bSpeedErrorNumber;

  return ( SpeedSensorReliability );
}

/**
  * @brief  SPD_GetS16Speed：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t SPD_GetS16Speed( SpeednPosFdbk_Handle_t * pHandle )
{
  int32_t wAux = ( int32_t ) pHandle->hAvrMecSpeedUnit;
  wAux *= INT16_MAX;
  wAux /= ( int16_t ) pHandle->hMaxReliableMecSpeedUnit;
  return ( int16_t )wAux;
}

/**
  * @brief  SPD_GetElToMecRatio：读取并返回相电流、速度、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint8_t SPD_GetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle )
{
  return ( pHandle->bElToMecRatio );
}

/**
  * @brief  SPD_SetElToMecRatio：设置相电流、速度、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPP  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void SPD_SetElToMecRatio( SpeednPosFdbk_Handle_t * pHandle, uint8_t bPP )
{
  pHandle->bElToMecRatio = bPP;
}


/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
