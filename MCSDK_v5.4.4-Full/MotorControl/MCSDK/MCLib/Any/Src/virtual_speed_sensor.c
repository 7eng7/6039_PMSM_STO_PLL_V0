/** @file virtual_speed_sensor.c
 *  @brief 无感启动虚拟速度/角度发生器。按加速度积分速度、按速度积分电角度；切闭环时可逐步
 *  对齐STO角度以避免突跳。它产生的是强制参考，并非真实位置测量。 */
/**
  ******************************************************************************
  * @file    virtual_speed_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "virtual_speed_sensor.h"


/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup SpeednPosFdbk
  * @{
  */

/**
  * @defgroup VirtualSpeedSensor 速度、位置和角度、斜坡、电机功能模块
  * @brief 速度、位置和角度、斜坡、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  VSS_Init：初始化相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void VSS_Init( VirtualSpeedSensor_Handle_t * pHandle )
{
#ifdef FASTDIV
  FD_Init( &( pHandle->fd ) );
#endif

  VSS_Clear( pHandle );
}

/**
  * @brief  VSS_Clear：初始化相电流、速度、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void VSS_Clear( VirtualSpeedSensor_Handle_t * pHandle )
{

  pHandle->_Super.bSpeedErrorNumber = 0u;
  pHandle->_Super.hElAngle = 0;
  pHandle->_Super.hMecAngle = 0;
  pHandle->_Super.hAvrMecSpeedUnit = 0;
  pHandle->_Super.hElSpeedDpp = 0;
  pHandle->_Super.hMecAccelUnitP = 0;
  pHandle->_Super.bSpeedErrorNumber = 0u;

  pHandle->wElAccDppP32 = 0;
  pHandle->wElSpeedDpp32 = 0;
  pHandle->hRemainingStep = 0u;
  pHandle->hElAngleAccu = 0;

  pHandle->bTransitionStarted = false;
  pHandle->bTransitionEnded = false;
  pHandle->hTransitionRemainingSteps = pHandle->hTransitionSteps;
  pHandle->bTransitionLocked = false;

  pHandle->bCopyObserver = false;

#ifdef FASTDIV
  /* 中文说明：处理并更新相关数据和控制状态。 */
  /* 中文说明：处理并更新速度。 */
  FD_FastDiv( &( pHandle->fd ), 1, ( int32_t )( pHandle->_Super.bElToMecRatio ) );
  FD_FastDiv( &( pHandle->fd ), 1, ( int32_t )( pHandle->hTransitionSteps ) );
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  VSS_CalcElAngle：计算并更新相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t VSS_CalcElAngle( VirtualSpeedSensor_Handle_t * pHandle, void * pInputVars_str )
{

  int16_t hRetAngle;
  int16_t hAngleDiff;
  int16_t hAngleCorr;
  int32_t wAux;
  int16_t hSignCorr = 1;

  if ( pHandle->bCopyObserver == true )
  {
    hRetAngle = *( int16_t * )pInputVars_str;
  }
  else
  {
    pHandle->hElAngleAccu += pHandle->_Super.hElSpeedDpp;

#ifdef FASTDIV
    pHandle->_Super.hMecAngle += ( int16_t )( FD_FastDiv( &( pHandle->fd ),
                                 ( int32_t )pHandle->_Super.hElSpeedDpp,
                                 ( int32_t )pHandle->_Super.bElToMecRatio ) );
#else
    pHandle->_Super.hMecAngle += pHandle->_Super.hElSpeedDpp /
                                 ( int16_t )pHandle->_Super.bElToMecRatio;
#endif

    if ( pHandle->bTransitionStarted == true )
    {
      if ( pHandle->hTransitionRemainingSteps == 0 )
      {
        hRetAngle = *( int16_t * )pInputVars_str;
        pHandle->bTransitionEnded = true;
        pHandle->_Super.bSpeedErrorNumber = 0u;
      }
      else
      {
        pHandle->hTransitionRemainingSteps--;

        if ( pHandle->_Super.hElSpeedDpp >= 0 )
        {
          hAngleDiff = *( int16_t * )pInputVars_str - pHandle->hElAngleAccu;
        }
        else
        {
          hAngleDiff = pHandle->hElAngleAccu - *( int16_t * )pInputVars_str;
          hSignCorr = -1;
        }

        wAux = ( int32_t )hAngleDiff * pHandle->hTransitionRemainingSteps;

#ifdef FASTDIV
        hAngleCorr = ( int16_t )( FD_FastDiv( &( pHandle->fd ),
                                              wAux,
                                              ( int32_t )( pHandle->hTransitionSteps ) ) );
#else
        hAngleCorr = ( int16_t )( wAux / pHandle->hTransitionSteps );
#endif

        hAngleCorr *= hSignCorr;

        if ( hAngleDiff >= 0 )
        {
          pHandle->bTransitionLocked = true;
          hRetAngle = *( int16_t * )pInputVars_str - hAngleCorr;
        }
        else
        {
          if ( pHandle->bTransitionLocked == false )
          {
            hRetAngle = pHandle->hElAngleAccu;
          }
          else
          {
            hRetAngle = *( int16_t * )pInputVars_str + hAngleCorr;
          }
        }
      }
    }
    else
    {
      hRetAngle = pHandle->hElAngleAccu;
    }
  }

  pHandle->_Super.hElAngle = hRetAngle;
  return hRetAngle;
}

/**
  * @brief  VSS_CalcAvrgMecSpeedUnit：读取并返回相电流、速度、位置和角度、状态机状态、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool VSS_CalcAvrgMecSpeedUnit( VirtualSpeedSensor_Handle_t * pHandle, int16_t * hMecSpeedUnit )
{
  bool SpeedSensorReliability = false;

  if ( pHandle->hRemainingStep > 1u )
  {
    pHandle->wElSpeedDpp32 += pHandle->wElAccDppP32;
    pHandle->_Super.hElSpeedDpp = ( int16_t )( pHandle->wElSpeedDpp32 / 65536 );

    /* 中文说明：处理并更新相关数据和控制状态。 */
    *hMecSpeedUnit = ( int16_t )( ( ( int32_t )pHandle->_Super.hElSpeedDpp *
                                    ( int32_t )pHandle->_Super.hMeasurementFrequency * SPEED_UNIT ) /
                                  ( ( int32_t )pHandle->_Super.DPPConvFactor * ( int32_t )pHandle->_Super.bElToMecRatio ) );

    pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;

    pHandle->hRemainingStep--;
  }
  else if ( pHandle->hRemainingStep == 1u )
  {
    *hMecSpeedUnit = pHandle->hFinalMecSpeedUnit;

    pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;

    pHandle->_Super.hElSpeedDpp = ( int16_t )( ( ( int32_t )( *hMecSpeedUnit ) *
                                  ( int32_t ) ( pHandle->_Super.DPPConvFactor) ) /
                                  ( ( int32_t )SPEED_UNIT * ( int32_t )pHandle->_Super.hMeasurementFrequency ) );

    pHandle->_Super.hElSpeedDpp *= ( int16_t )( pHandle->_Super.bElToMecRatio );

    pHandle->hRemainingStep = 0u;
  }
  else
  {
    *hMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
  }
  /* 中文说明：检查并判断速度。 */
  if ( pHandle->bTransitionEnded == false )
  {
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    SpeedSensorReliability = false;
  }
  else
  {
    SpeedSensorReliability = SPD_IsMecSpeedReliable ( &pHandle->_Super, hMecSpeedUnit );
  }

  return ( SpeedSensorReliability );
}

/**
  * @brief  VSS_SetMecAngle：设置相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hMecAngle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void VSS_SetMecAngle( VirtualSpeedSensor_Handle_t * pHandle, int16_t hMecAngle )
{

  pHandle->hElAngleAccu = hMecAngle;
  pHandle->_Super.hMecAngle = pHandle->hElAngleAccu / ( int16_t )pHandle->_Super.bElToMecRatio;
  pHandle->_Super.hElAngle = hMecAngle;
}

/**
  * @brief  VSS_SetMecAcceleration：设置相电流、速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void  VSS_SetMecAcceleration( VirtualSpeedSensor_Handle_t * pHandle, int16_t  hFinalMecSpeedUnit,
                              uint16_t hDurationms )
{

  uint16_t hNbrStep;
  int16_t hCurrentMecSpeedDpp;
  int32_t wMecAccDppP32;
  int16_t hFinalMecSpeedDpp;

  if ( pHandle->bTransitionStarted == false )
  {
    if ( hDurationms == 0u )
    {
      pHandle->_Super.hAvrMecSpeedUnit = hFinalMecSpeedUnit;

      pHandle->_Super.hElSpeedDpp = ( int16_t )( ( ( int32_t )( hFinalMecSpeedUnit ) *
                                    ( int32_t )( pHandle->_Super.DPPConvFactor) ) /
                                    ( ( int32_t )SPEED_UNIT * ( int32_t )pHandle->_Super.hMeasurementFrequency ) );

      pHandle->_Super.hElSpeedDpp *= ( int16_t )( pHandle->_Super.bElToMecRatio );

      pHandle->hRemainingStep = 0u;

      pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;
    }
    else
    {
      hNbrStep = ( uint16_t )( ( ( uint32_t )hDurationms *
                                 ( uint32_t )pHandle->hSpeedSamplingFreqHz ) /
                               1000u );

      hNbrStep++;

      pHandle->hRemainingStep = hNbrStep;

      hCurrentMecSpeedDpp = pHandle->_Super.hElSpeedDpp /
                            ( int16_t )pHandle->_Super.bElToMecRatio;

      hFinalMecSpeedDpp = ( int16_t )( ( ( int32_t )hFinalMecSpeedUnit * ( int32_t )( pHandle->_Super.DPPConvFactor) ) /
                                       ( ( int32_t )SPEED_UNIT * ( int32_t )pHandle->_Super.hMeasurementFrequency ) );

      wMecAccDppP32 = ( ( ( int32_t )hFinalMecSpeedDpp - ( int32_t )hCurrentMecSpeedDpp ) *
                        ( int32_t )65536 ) / ( int32_t )hNbrStep;

      pHandle->wElAccDppP32 = wMecAccDppP32 * ( int16_t )pHandle->_Super.bElToMecRatio;

      pHandle->hFinalMecSpeedUnit = hFinalMecSpeedUnit;

      pHandle->wElSpeedDpp32 = ( int32_t )pHandle->_Super.hElSpeedDpp * ( int32_t )65536;
    }
  }
}

/**
  * @brief  VSS_RampCompleted：处理中断并更新相电流、速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool VSS_RampCompleted( VirtualSpeedSensor_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->hRemainingStep == 0u )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  VSS_GetLastRampFinalSpeed：读取并返回相电流、速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t  VSS_GetLastRampFinalSpeed( VirtualSpeedSensor_Handle_t * pHandle )
{
  return pHandle->hFinalMecSpeedUnit;
}

/**
  * @brief  VSS_SetStartTransition：设置相电流、速度、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bool  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool VSS_SetStartTransition( VirtualSpeedSensor_Handle_t * pHandle, bool bCommand )
{
  bool bAux = true;
  if ( bCommand == true )
  {
    pHandle->bTransitionStarted = true;

    if ( pHandle->hTransitionSteps == 0 )
    {
      pHandle->bTransitionEnded = true;
      pHandle->_Super.bSpeedErrorNumber = 0u;
      bAux = false;
    }
  }
  return bAux;
}

/**
  * @brief  VSS_IsTransitionOngoing：读取并返回相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool VSS_IsTransitionOngoing( VirtualSpeedSensor_Handle_t * pHandle )
{
  uint16_t hTS = 0u, hTE = 0u, hAux;
  bool retVal = false;
  if ( pHandle->bTransitionStarted == true )
  {
    hTS = 1u;
  }
  if ( pHandle->bTransitionEnded == true )
  {
    hTE = 1u;
  }
  hAux = hTS ^ hTE;
  if ( hAux != 0u )
  {
    retVal = true;
  }
  return ( retVal );
}

__WEAK bool VSS_TransitionEnded( VirtualSpeedSensor_Handle_t * pHandle )
{
    return pHandle->bTransitionEnded;
}

/**
  * @brief  VSS_SetCopyObserver：设置相电流、速度、位置和角度、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void VSS_SetCopyObserver( VirtualSpeedSensor_Handle_t * pHandle )
{
  pHandle->bCopyObserver = true;
}

/**
  * @brief  VSS_SetElAngle：设置相电流、速度、位置和角度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hElAngle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void VSS_SetElAngle( VirtualSpeedSensor_Handle_t * pHandle, int16_t hElAngle )
{
  pHandle->hElAngleAccu = hElAngle;
  pHandle->_Super.hElAngle = hElAngle;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
