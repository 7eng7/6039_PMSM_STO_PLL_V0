/** @file pwm_curr_fdbk.c
 *  @brief PWM和相电流反馈硬件抽象。上层通过函数指针读电流、校准、开关PWM及设置相电压；
 *  SetPhaseVoltage判断SVPWM扇区、计算三相比较值，再交由R3_2选择ADC采样点。 */
/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "pwm_curr_fdbk.h"

#include "mc_type.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup pwm_curr_fdbk 相电流、PWM和占空比、ADC采样、电机功能模块
  * @brief 相电流、PWM和占空比、ADC采样、电机相关组件及其公共接口。
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  PWMC_GetPhaseCurrents：读取并返回相电流、PWM和占空比、ADC采样、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pStator_Currents  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * Iab )
{
  pHandle->pFctGetPhaseCurrents( pHandle, Iab );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  当前接口：处理并更新PWM和占空比，保持对象状态和控制流程一致。
  */

/**
  * @brief  PWMC_SetPhaseVoltage：设置相电流、电压、PWM和占空比、故障与保护、串口通信帧、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Valfa_beta  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle, alphabeta_t Valfa_beta )
{
  /* 由alpha-beta电压计算三相等效作用时间，依据三个投影量符号确定六个SVPWM扇区，再得到
   * A/B/C比较值。派生回调在末尾安排该扇区ADC采样点并检查是否错过更新窗口。 */
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;

  wUAlpha = Valfa_beta.alpha * ( int32_t )pHandle->hT_Sqrt3;
  wUBeta = -( Valfa_beta.beta * ( int32_t )( pHandle->PWMperiod ) ) * 2;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* 中文说明：处理并更新相关数据和控制状态。 */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      pHandle->Sector = SECTOR_5;
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->lowDuty = wTimePhC;
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhB;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_4;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhC;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhA;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_3;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhB;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhA;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      pHandle->Sector = SECTOR_2;
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->lowDuty = wTimePhB;
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhC;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_6;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhB;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_1;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhC;
      }
  }

  pHandle->CntPhA = ( uint16_t )wTimePhA;
  pHandle->CntPhB = ( uint16_t )wTimePhB;
  pHandle->CntPhC = ( uint16_t )wTimePhC;

  if ( pHandle->DTTest == 1u )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    if ( pHandle->Ia > 0 )
    {
      pHandle->CntPhA += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhA -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ib > 0 )
    {
      pHandle->CntPhB += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhB -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ic > 0 )
    {
      pHandle->CntPhC += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhC -= pHandle->DTCompCnt;
    }
  }

  return ( pHandle->pFctSetADCSampPointSectX( pHandle ) );
}

/**
  * @brief  PWMC_SwitchOffPWM：停止或禁用PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  PWMC_SwitchOnPWM：启动或使能PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}

/**
  * @brief  PWMC_CurrentReadingCalibr：初始化相电流、电压、PWM和占空比、ADC采样、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  action  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle, CRCAction_t action )
{
  /* CRC_START启动异步偏置校准，CRC_EXEC由状态机周期轮询进度；返回true表示样本数已满足并恢复
   * 正常相电流读取函数。这样校准无需阻塞中频调度器。 */
  bool retVal = false;
  if ( action == CRC_START )
  {
    PWMC_SwitchOffPWM( pHandle );
    pHandle->OffCalibrWaitTimeCounter = pHandle->OffCalibrWaitTicks;
    if ( pHandle->OffCalibrWaitTicks == 0u )
    {
      pHandle->pFctCurrReadingCalib( pHandle );
      retVal = true;
    }
  }
  else if ( action == CRC_EXEC )
  {
    if ( pHandle->OffCalibrWaitTimeCounter > 0u )
    {
      pHandle->OffCalibrWaitTimeCounter--;
      if ( pHandle->OffCalibrWaitTimeCounter == 0u )
      {
        pHandle->pFctCurrReadingCalib( pHandle );
        retVal = true;
      }
    }
    else
    {
      retVal = true;
    }
  }
  else
  {
  }
  return retVal;
}

/**
  * @brief  PWMC_TurnOnLowSides：启动或使能电压、PWM和占空比、功率、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides( pHandle );
}


/**
  * @brief  PWMC_CheckOverCurrent：检查并判断相电流、PWM和占空比、故障与保护、功率，保持对象状态和控制流程一致。
  */
__WEAK uint16_t PWMC_CheckOverCurrent( PWMC_Handle_t * pHandle )
{
  return pHandle->pFctIsOverCurrentOccurred( pHandle );
}

/**
  * @brief  PWMC_OCPSetReferenceVoltage：设置相电流、电压、PWM和占空比、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDACVref  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_OCPSetReferenceVoltage( PWMC_Handle_t * pHandle, uint16_t hDACVref )
{
  if ( pHandle->pFctOCPSetReferenceVoltage )
  {
    pHandle->pFctOCPSetReferenceVoltage( pHandle, hDACVref );
  }
}

/**
  * @brief  PWMC_GetTurnOnLowSidesAction：读取并返回相电流、PWM和占空比、状态机状态，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
/**
  * @brief  PWMC_GetTurnOnLowSidesAction：读取并返回PWM和占空比、功率，保持对象状态和控制流程一致。
  */
__WEAK bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle )
{
  return pHandle->TurnOnLowSidesAction;
}

/**
* @brief  PWMC_RLDetectionModeEnable：启动或使能PWM和占空比、功率，保持对象状态和控制流程一致。
*/
__WEAK void PWMC_RLDetectionModeEnable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeEnable )
  {
    pHandle->pFctRLDetectionModeEnable( pHandle );
  }
}

/**
* @brief  PWMC_RLDetectionModeDisable：检查并判断PWM和占空比、功率，保持对象状态和控制流程一致。
*/
__WEAK void PWMC_RLDetectionModeDisable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeDisable )
  {
    pHandle->pFctRLDetectionModeDisable( pHandle );
  }
}

/**
  * @brief  PWMC_RLDetectionModeSetDuty：设置PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDuty  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t PWMC_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle, uint16_t hDuty )
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if ( pHandle->pFctRLDetectionModeSetDuty )
  {
    hRetVal = pHandle->pFctRLDetectionModeSetDuty( pHandle, hDuty );
  }
  return hRetVal;
}

/**
  * @brief  PWMC_RegisterGetPhaseCurrentsCallBack：读取并返回相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctGetPhaseCurrents = pCallBack;
}

/**
  * @brief  PWMC_RegisterSwitchOffPwmCallBack：检查并判断PWM和占空比，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
  * @brief  PWMC_RegisterSwitchonPwmCallBack：检查并判断PWM和占空比，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
  * @brief  PWMC_RegisterReadingCalibrationCallBack：读取并返回相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
  * @brief  PWMC_RegisterTurnOnLowSidesCallBack：检查并判断PWM和占空比，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
  * @brief  PWMC_RegisterSampPointSectXCallBack：计算并更新PWM和占空比、ADC采样，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctSetADCSampPointSectX = pCallBack;
}

/**
  * @brief  PWMC_RegisterIsOverCurrentOccurredCallBack：检查并判断相电流、PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  pCallBack  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}

/**
  * @brief  PWMC_RegisterOCPSetRefVoltageCallBack：设置相电流、电压、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}

/**
  * @brief  PWMC_RegisterRLDetectionModeEnableCallBack：检查并判断PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeEnable = pCallBack;
}

/**
  * @brief  PWMC_RegisterRLDetectionModeDisableCallBack：检查并判断PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeDisable = pCallBack;
}

/**
  * @brief  PWMC_RegisterRLDetectionModeSetDutyCallBack：设置PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeSetDuty = pCallBack;
}

/**
  * @brief  PWMC_RegisterIrqHandlerCallBack：检查并判断PWM和占空比、中断，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t * pHandle )
{
  pHandle->pFctIrqHandler = pCallBack;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
