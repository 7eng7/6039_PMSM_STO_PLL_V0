/** @file revup_ctrl.c
 *  @brief 无感开环启动阶段控制。按多阶段时间、终速和Iq生成斜坡，并检查STO估速与强制速度的
 *  方向/幅值一致性；连续可靠后允许切闭环，阶段超时或不收敛会报告启动失败。 */
/**
  ******************************************************************************
  * @file    revup_ctrl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "revup_ctrl.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup RevUpCtrl 速度、PI/PID调节器、电机功能模块
  * @brief 速度、PI/PID调节器、电机相关组件及其公共接口。
  * @{
  */

/* 私有宏定义 */

/**
  * @brief  RUC_Init：初始化观测器、反电势和PLL，保持对象状态和控制流程一致。
  */
#define RUC_OTF_PLL_RESET_TIMEOUT 100u


/**
  * @brief  RUC_Init：初始化速度、转矩、PWM和占空比、状态机状态、观测器、反电势和PLL、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pSTC:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pVSS:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pSNSL:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pPWM:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_Init( RevUpCtrl_Handle_t * pHandle,
               SpeednTorqCtrl_Handle_t * pSTC,
               VirtualSpeedSensor_Handle_t * pVSS,
               STO_Handle_t * pSNSL,
               PWMC_Handle_t * pPWM )
{
  /* 绑定速度/转矩控制器、虚拟速度传感器和STO，并统计有效启动阶段链表；阶段参数本身来自
   * drive_parameters.h生成的mc_config.c静态对象。 */
  RevUpCtrl_PhaseParams_t * pRUCPhaseParams = &pHandle->ParamsData[0];
  uint8_t bPhase = 0u;

  pHandle->pSTC = pSTC;
  pHandle->pVSS = pVSS;
  pHandle->pSNSL = pSNSL;
  pHandle->pPWM = pPWM;
  pHandle->OTFSCLowside = false;
  pHandle->EnteredZone1 = false;

  while ( ( pRUCPhaseParams != MC_NULL ) && ( bPhase < RUC_MAX_PHASE_NUMBER ) )
  {
    pRUCPhaseParams = pRUCPhaseParams->pNext;
    bPhase++;
  }
  pHandle->ParamsData[bPhase - 1u].pNext = MC_NULL;

  pHandle->bPhaseNbr = bPhase;

  pHandle->bResetPLLTh = ( uint8_t )( ( RUC_OTF_PLL_RESET_TIMEOUT * pHandle->hRUCFrequencyHz ) / 1000u );

}

/**
  * @brief  RUC_Clear：初始化状态机状态、PI/PID调节器、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hMotorDirection:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_Clear( RevUpCtrl_Handle_t * pHandle, int16_t hMotorDirection )
{
  /* 从第一阶段重新开始，按方向修正各阶段速度/转矩符号，同时清除切换和收敛计数。 */
  VirtualSpeedSensor_Handle_t * pVSS = pHandle->pVSS;
  SpeednTorqCtrl_Handle_t * pSTC = pHandle->pSTC;
  RevUpCtrl_PhaseParams_t * pPhaseParams = pHandle->ParamsData;

  pHandle->hDirection = hMotorDirection;
  pHandle->EnteredZone1 = false;

  /* 中文说明：初始化相关数据和控制状态。 */
  pHandle->bStageCnt = 0u;
  pHandle->bOTFRelCounter = 0u;
  pHandle->OTFSCLowside = false;

  /* 中文说明：清除并复位相关数据和控制状态。 */
  VSS_Clear( pVSS );

  /* 中文说明：处理并更新转矩。 */
  STC_SetControlMode( pSTC, STC_TORQUE_MODE );

  /* 中文说明：处理并更新位置和角度。 */
  VSS_SetMecAngle( pVSS, pHandle->hStartingMecAngle * hMotorDirection );

  /* 中文说明：处理并更新转矩。 */
  STC_ExecRamp( pSTC, 0, 0u );

  /* 中文说明：处理并更新相关数据和控制状态。 */
  STC_ExecRamp( pSTC, pPhaseParams->hFinalTorque * hMotorDirection,
                ( uint32_t )( pPhaseParams->hDurationms ) );

  VSS_SetMecAcceleration( pVSS, pPhaseParams->hFinalMecSpeedUnit * hMotorDirection,
                          pPhaseParams->hDurationms );

  /* 中文说明：计算并更新相关数据和控制状态。 */
  pHandle->hPhaseRemainingTicks =
    ( uint16_t )( ( ( uint32_t )pPhaseParams->hDurationms *
                    ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );

  pHandle->hPhaseRemainingTicks++;

  /* 中文说明：设置参数和增益。 */
  pHandle->pCurrentPhaseParams = pPhaseParams->pNext;

  /* 中文说明：清除并复位观测器、反电势和PLL。 */
  pHandle->bResetPLLCnt = 0u;
}

/**
  * @brief  RUC_OTF_Exec：设置PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool RUC_OTF_Exec( RevUpCtrl_Handle_t * pHandle )
{
  bool IsSpeedReliable;
  bool retVal = true;
  bool condition = false;

  if ( pHandle->hPhaseRemainingTicks > 0u )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    pHandle->hPhaseRemainingTicks--;

    /* 中文说明：启动或使能相关数据和控制状态。 */
    if ( pHandle->bStageCnt == 0u )
    {
      if ( pHandle->EnteredZone1 == false )
      {
        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          pHandle->bResetPLLCnt++;
          if ( pHandle->bResetPLLCnt > pHandle->bResetPLLTh )
          {
            pHandle->pSNSL->pFctStoOtfResetPLL( pHandle->pSNSL );
            pHandle->bOTFRelCounter = 0u;
            pHandle->bResetPLLCnt = 0u;
          }
        }

        IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck( pHandle->pSNSL );

        if ( IsSpeedReliable )
        {
          if ( pHandle->bOTFRelCounter < 127u )
          {
            pHandle->bOTFRelCounter++;
          }
        }
        else
        {
          pHandle->bOTFRelCounter = 0u;
        }

        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          if ( pHandle->bOTFRelCounter == ( pHandle->bResetPLLTh >> 1 ) )
          {
            condition = true;
          }
        }
        else
        {
          if ( pHandle->bOTFRelCounter == 127 )
          {
            condition = true;
          }
        }

        if ( condition == true )
        {
          bool bCollinearSpeed = false;
          int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit( pHandle->pSNSL->_Super );
          int16_t hObsSpeedUnitAbsValue =
                  ( hObsSpeedUnit < 0 ? ( -hObsSpeedUnit ) : ( hObsSpeedUnit ) ); /* 中文说明：处理并更新速度。 */

          if ( pHandle->hDirection > 0 )
          {
            if ( hObsSpeedUnit > 0 )
            {
              bCollinearSpeed = true; /* 中文说明：处理并更新速度。 */
            }
          }
          else
          {
            if ( hObsSpeedUnit < 0 )
            {
              bCollinearSpeed = true; /* 中文说明：处理并更新速度。 */
            }
          }

          if ( bCollinearSpeed == false )
          {
            /* 中文说明：处理并更新速度。 */
            pHandle->bOTFRelCounter = 0u;
          }
          else /* 中文说明：处理并更新速度。 */
          {
            if ( ( uint16_t )( hObsSpeedUnitAbsValue ) > pHandle->hMinStartUpValidSpeed )
            {
              /* 中文说明：执行并推进相关数据和控制状态。 */
              pHandle->pSNSL->pFctForceConvergency1( pHandle->pSNSL );
              pHandle->EnteredZone1 = true;
            }
            else if ( ( uint16_t )( hObsSpeedUnitAbsValue ) > pHandle->hMinStartUpFlySpeed )
            {
              /* 中文说明：处理并更新相关数据和控制状态。 */
              /* 中文说明：处理并更新相关数据和控制状态。 */
              int16_t hOldFinalMecSpeedUnit = 0;
              int16_t hOldFinalTorque = 0;
              int32_t wDeltaSpeedRevUp;
              int32_t wDeltaTorqueRevUp;
              bool bError = false;
              VSS_SetCopyObserver( pHandle->pVSS );
              pHandle->pSNSL->pFctForceConvergency2( pHandle->pSNSL );

              if (pHandle->pCurrentPhaseParams == MC_NULL)
              {
                bError = true;
                pHandle->hPhaseRemainingTicks = 0u;
              }
              else
              {
                while ( pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit < hObsSpeedUnitAbsValue )
                {
                  if ( pHandle->pCurrentPhaseParams->pNext == MC_NULL )
                  {
                    /* 中文说明：处理并更新故障与保护。 */
                    bError = true;
                    pHandle->hPhaseRemainingTicks = 0u;
                    break;
                  }
                  else
                  {
                    /* 中文说明：处理并更新相关数据和控制状态。 */
                    hOldFinalMecSpeedUnit = pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit;
                    hOldFinalTorque = pHandle->pCurrentPhaseParams->hFinalTorque;
                    pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;
                    pHandle->bStageCnt++;
                  }
                }
              }

              if ( bError == false )
              {
                /* 中文说明：处理并更新相关数据和控制状态。 */
                int16_t hTorqueReference;

                wDeltaSpeedRevUp = ( int32_t )( pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit ) - ( int32_t )( hOldFinalMecSpeedUnit );
                wDeltaTorqueRevUp = ( int32_t )( pHandle->pCurrentPhaseParams->hFinalTorque ) - ( int32_t )( hOldFinalTorque );

                hTorqueReference = ( int16_t )( ( ( ( int32_t )hObsSpeedUnit ) * wDeltaTorqueRevUp ) / wDeltaSpeedRevUp ) +
                        hOldFinalTorque;

                STC_ExecRamp( pHandle->pSTC, hTorqueReference, 0u );

                pHandle->hPhaseRemainingTicks = 1u;

                pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;

                pHandle->bStageCnt = 6u;
              } /* 中文说明：处理并更新故障与保护。 */
            } /* 中文说明：处理并更新速度。 */
            else
            {
            }
          } /* 中文说明：处理并更新速度。 */
        } /* 中文说明：检查并判断速度。 */
      }/* 中文说明：处理并更新相关数据和控制状态。 */
      else
      {
        pHandle->pSNSL->pFctForceConvergency1( pHandle->pSNSL );
      }
    } /* 中文说明：处理并更新相关数据和控制状态。 */
  } /* 中文说明：处理并更新相关数据和控制状态。 */

  if ( pHandle->hPhaseRemainingTicks == 0u )
  {
    if ( pHandle->pCurrentPhaseParams != MC_NULL )
    {
      if ( pHandle->bStageCnt == 0u )
      {
        /* 中文说明：处理并更新相关数据和控制状态。 */
        PWMC_SwitchOffPWM( pHandle->pPWM );
        pHandle->OTFSCLowside = true;
        PWMC_TurnOnLowSides( pHandle->pPWM );
        pHandle->bOTFRelCounter = 0u;
      }
      else if ( ( pHandle->bStageCnt == 1u ) )
      {
        PWMC_SwitchOnPWM( pHandle->pPWM );
        pHandle->OTFSCLowside = false;
      }
      else
      {
      }

      /* 中文说明：处理并更新相电流。 */
      /* 中文说明：处理并更新相关数据和控制状态。 */
      STC_ExecRamp( pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                    ( uint32_t )( pHandle->pCurrentPhaseParams->hDurationms ) );

      VSS_SetMecAcceleration( pHandle->pVSS,
                              pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                              pHandle->pCurrentPhaseParams->hDurationms );

      /* 中文说明：计算并更新相关数据和控制状态。 */
      pHandle->hPhaseRemainingTicks =
              ( uint16_t )( ( ( uint32_t )pHandle->pCurrentPhaseParams->hDurationms *
                      ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );
      pHandle->hPhaseRemainingTicks++;

      /* 中文说明：设置参数和增益。 */
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;

      /* 中文说明：处理并更新相关数据和控制状态。 */
      pHandle->bStageCnt++;
    }
    else
    {
      if ( pHandle->bStageCnt == pHandle->bPhaseNbr - 1 ) /* 中文说明：处理并更新相关数据和控制状态。 */
      {
        retVal = false;
      }
      else if ( pHandle->bStageCnt == 7u ) /* 中文说明：执行并推进相关数据和控制状态。 */
      {
        pHandle->bStageCnt = 0u; /* 中文说明：执行并推进状态机状态。 */
        pHandle->hPhaseRemainingTicks = 0u;
      }
      else
      {
      }
    }
  }
  return retVal;
}

/**
  * @brief  RUC_Exec：设置PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool RUC_Exec( RevUpCtrl_Handle_t * pHandle )
{
  /* 每个中频周期推进当前阶段的速度、转矩斜坡；阶段计时结束后转入下一节点。
   * 返回false表示阶段链已耗尽但观测器仍未满足闭环条件，上层将判定启动失败。 */
  bool retVal = true;

  if ( pHandle->hPhaseRemainingTicks > 0u )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    pHandle->hPhaseRemainingTicks--;

  } /* 中文说明：处理并更新相关数据和控制状态。 */

  if ( pHandle->hPhaseRemainingTicks == 0u )
  {
    if ( pHandle->pCurrentPhaseParams != MC_NULL )
    {

      /* 中文说明：处理并更新相电流。 */
      /* 中文说明：处理并更新相关数据和控制状态。 */
      STC_ExecRamp( pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                    ( uint32_t )( pHandle->pCurrentPhaseParams->hDurationms ) );

      VSS_SetMecAcceleration( pHandle->pVSS,
                              pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                              pHandle->pCurrentPhaseParams->hDurationms );

      /* 中文说明：计算并更新相关数据和控制状态。 */
      pHandle->hPhaseRemainingTicks =
        ( uint16_t )( ( ( uint32_t )pHandle->pCurrentPhaseParams->hDurationms *
                        ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );
      pHandle->hPhaseRemainingTicks++;

      /* 中文说明：设置参数和增益。 */
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;

      /* 中文说明：处理并更新相关数据和控制状态。 */
      pHandle->bStageCnt++;
    }
    else
    {
      retVal = false;
    }
  }
  return retVal;
}

/**
  * @brief  RUC_Completed：设置相电流、状态机状态、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool RUC_Completed( RevUpCtrl_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->pCurrentPhaseParams == MC_NULL )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  RUC_Stop：停止或禁用相电流、速度、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_Stop( RevUpCtrl_Handle_t * pHandle )
{
  VirtualSpeedSensor_Handle_t * pVSS = pHandle->pVSS;
  pHandle->pCurrentPhaseParams = MC_NULL;
  pHandle->hPhaseRemainingTicks = 0u;
  VSS_SetMecAcceleration( pVSS, SPD_GetAvrgMecSpeedUnit( & pVSS->_Super ), 0u );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  RUC_FirstAccelerationStageReached：设置斜坡、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool RUC_FirstAccelerationStageReached( RevUpCtrl_Handle_t * pHandle )
{
  bool retVal = false;

  if ( pHandle->bStageCnt >= pHandle->bFirstAccelerationStage )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  RUC_SetPhaseDurationms：设置PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_SetPhaseDurationms( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase, uint16_t hDurationms )
{
  pHandle->ParamsData[bPhase].hDurationms = hDurationms;
}

/**
  * @brief  RUC_SetPhaseFinalMecSpeedUnit：设置速度、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalMecSpeedUnit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_SetPhaseFinalMecSpeedUnit( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase,
                                    int16_t hFinalMecSpeedUnit )
{
  pHandle->ParamsData[bPhase].hFinalMecSpeedUnit = hFinalMecSpeedUnit;
}

/**
  * @brief  RUC_SetPhaseFinalTorque：设置转矩、PI/PID调节器、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalTorque:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RUC_SetPhaseFinalTorque( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase, int16_t hFinalTorque )
{
  pHandle->ParamsData[bPhase].hFinalTorque = hFinalTorque;
}

/**
  * @brief  RUC_GetPhaseDurationms：设置PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t RUC_GetPhaseDurationms( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( uint16_t )pHandle->ParamsData[bPhase].hDurationms );
}

/**
  * @brief  RUC_GetPhaseFinalMecSpeedUnit：设置速度、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t RUC_GetPhaseFinalMecSpeedUnit( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( int16_t )pHandle->ParamsData[bPhase].hFinalMecSpeedUnit );
}

/**
  * @brief  RUC_GetPhaseFinalTorque：设置转矩、PI/PID调节器、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bPhase:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t RUC_GetPhaseFinalTorque( RevUpCtrl_Handle_t * pHandle, uint8_t bPhase )
{
  return ( ( int16_t )pHandle->ParamsData[bPhase].hFinalTorque );
}

/**
  * @brief  RUC_GetNumberOfPhases：读取并返回PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint8_t RUC_GetNumberOfPhases( RevUpCtrl_Handle_t * pHandle )
{
  return ( ( uint8_t )pHandle->bPhaseNbr );
}

/**
  * @brief  RUC_Get_SCLowsideOTF_Status：设置PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool RUC_Get_SCLowsideOTF_Status( RevUpCtrl_Handle_t * pHandle )
{
  return ( pHandle->OTFSCLowside );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
