/** @file pwm_common.c
 *  @brief PWM实例公共状态和同步辅助。保存多PWM频率关系与更新计数；单电机仍使用该抽象保持
 *  MCSDK接口一致，真实ET6039寄存器操作位于R3_2实现。 */
/**
  ******************************************************************************
  * @file    pwm_common.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "pwm_common.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup pwm_curr_fdbk 相电流、PWM和占空比、ADC采样、电机功能模块
  * @brief 相电流、PWM和占空比、ADC采样、电机相关组件及其公共接口。
  * @{
  */

#ifdef TIM2
/**
  * @brief  startTimers：设置PWM和占空比、状态机状态，保持对象状态和控制流程一致。
  * @param  none  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void startTimers( void )
{
  uint32_t isTIM2ClockOn;
  uint32_t trigOut;

  isTIM2ClockOn = LL_APB1_GRP1_IsEnabledClock ( LL_APB1_GRP1_PERIPH_TIM2 );
  if ( isTIM2ClockOn == 0 )
  {
    /* 中文说明：启动或使能相关数据和控制状态。 */
    LL_APB1_GRP1_EnableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
    LL_APB1_GRP1_DisableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
  }
  else
  {
    trigOut = LL_TIM_ReadReg( TIM2, CR2 ) & TIM_CR2_MMS;
    LL_TIM_SetTriggerOutput( TIM2, LL_TIM_TRGO_UPDATE );
    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
    LL_TIM_SetTriggerOutput( TIM2, trigOut );
  }
}
#endif

/**
  * @brief  waitForPolarizationEnd：处理并更新PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  TIMx:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void waitForPolarizationEnd(SRPWM_Type *SRPWMx, uint16_t  *SWerror, uint8_t repCnt, uint8_t *cnt )
{
  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;

  if ((SRPWMx->TB_CTL & SRPWM0_TB_CTL_TB_MD_Msk) >> SRPWM0_TB_CTL_TB_MD_Pos == 0) //根据定时器配置的计数模式修改最大校准周期阈值
  {
    hMaxPeriodsNumber=(2*NB_CONVERSIONS)*(((uint16_t)repCnt+1u));
  }
  else
  {
    hMaxPeriodsNumber=(2*NB_CONVERSIONS)*(((uint16_t)repCnt+1u)>>1);
  }

  /* 中文说明：处理并更新ADC采样。 */
  SRPWM_clearInterruptStatus(SRPWMx,SRPWM_IT_EVT);
  hCalibrationPeriodCounter = 0u;
  while (*cnt < NB_CONVERSIONS)
  {
    if (SRPWM_getInterruptStatus(SRPWMx,SRPWM_IT_EVT))
    {
      SRPWM_clearInterruptStatus(SRPWMx,SRPWM_IT_EVT);
      hCalibrationPeriodCounter++;
      if (hCalibrationPeriodCounter >= hMaxPeriodsNumber)
      {
        if (*cnt < NB_CONVERSIONS)
        {
          *SWerror = 1u;
          break;
        }
      }
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
