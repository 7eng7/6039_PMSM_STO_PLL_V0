/** @file pid_regulator.c
 *  @brief 定点PI/PID调节器。Kp提供即时校正，Ki消除稳态误差，可选Kd响应变化率；积分与输出均
 *  饱和以抑制wind-up。增益除数实现无浮点缩放，调参必须同时考虑缩放和稳定裕度。 */
/**
  ******************************************************************************
  * @file    pid_regulator.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "pid_regulator.h"

#include "mc_type.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup PIDRegulator PI/PID调节器、电机、参数和增益功能模块
  * @brief PI/PID调节器、电机、参数和增益相关组件及其公共接口。
  * @{
  */

/**
  * @brief  PID_HandleInit：初始化相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_HandleInit( PID_Handle_t * pHandle )
{
  /* 把运行增益恢复为默认值，并把积分、上次误差等动态状态清零；每次新启动都应有确定初值。 */
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}

/**
  * @brief  PID_SetKP：设置相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKpGain:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetKP( PID_Handle_t * pHandle, int16_t hKpGain )
{
  pHandle->hKpGain = hKpGain;
}

/**
  * @brief  PID_SetKI：设置相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKiGain:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetKI( PID_Handle_t * pHandle, int16_t hKiGain )
{
  pHandle->hKiGain = hKiGain;
}

/**
  * @brief  PID_GetKP：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PID_GetKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpGain );
}

/**
  * @brief  PID_GetKI：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PID_GetKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiGain );
}

/**
  * @brief  PID_GetDefaultKP：读取并返回相电流、故障与保护、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PID_GetDefaultKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKpGain );
}

/**
  * @brief  PID_GetDefaultKI：读取并返回相电流、故障与保护、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PID_GetDefaultKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKiGain );
}

/**
  * @brief  PID_SetIntegralTerm：设置相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  wIntegralTermValue:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetIntegralTerm( PID_Handle_t * pHandle, int32_t wIntegralTermValue )
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}

/**
  * @brief  PID_GetKPDivisor：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t PID_GetKPDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpDivisor );
}

/**
  * @brief  PID_SetKPDivisorPOW2：设置相电流、功率、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKpDivisorPOW2:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetKPDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKpDivisorPOW2 )
{
  pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
  pHandle->hKpDivisor = ( ( uint16_t )( 1u ) << hKpDivisorPOW2 );
}

/**
  * @brief  PID_GetKIDivisor：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t PID_GetKIDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiDivisor );
}

/**
  * @brief  PID_SetKIDivisorPOW2：设置相电流、功率、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKiDivisorPOW2:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetKIDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKiDivisorPOW2 )
{
  int32_t wKiDiv = ( ( int32_t )( 1u ) << hKiDivisorPOW2 );
  pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
  pHandle->hKiDivisor = ( uint16_t )( wKiDiv );
  PID_SetUpperIntegralTermLimit( pHandle, ( int32_t )INT16_MAX * wKiDiv );
  PID_SetLowerIntegralTermLimit( pHandle, ( int32_t ) - INT16_MAX * wKiDiv );
}

/**
  * @brief  PID_SetLowerIntegralTermLimit：设置相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wLowerLimit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetLowerIntegralTermLimit( PID_Handle_t * pHandle, int32_t wLowerLimit )
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}

/**
  * @brief  PID_SetUpperIntegralTermLimit：设置相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wUpperLimit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetUpperIntegralTermLimit( PID_Handle_t * pHandle, int32_t wUpperLimit )
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}

/**
  * @brief  PID_SetLowerOutputLimit：设置相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hLowerLimit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetLowerOutputLimit( PID_Handle_t * pHandle, int16_t hLowerLimit )
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}

/**
  * @brief  PID_SetUpperOutputLimit：设置相电流、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hUpperLimit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetUpperOutputLimit( PID_Handle_t * pHandle, int16_t hUpperLimit )
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}

/**
  * @brief  PID_SetPrevError：设置相电流、故障与保护、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wPrevProcessVarError:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetPrevError( PID_Handle_t * pHandle, int32_t wPrevProcessVarError )
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}

/**
  * @brief  PID_SetKD：设置相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKdGain:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PID_SetKD( PID_Handle_t * pHandle, int16_t hKdGain )
{
  pHandle->hKdGain = hKdGain;
}

/**
  * @brief  PID_GetKD：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PID_GetKD( PID_Handle_t * pHandle )
{
  return pHandle->hKdGain;
}

/**
  * @brief  PID_GetKDDivisor：读取并返回相电流、PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t PID_GetKDDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKdDivisor );
}

/**
  * @brief  PID_SetKDDivisorPOW2：设置PI/PID调节器、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hKdDivisorPOW2  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void PID_SetKDDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKdDivisorPOW2 )
{
  pHandle->hKdDivisorPOW2 = hKdDivisorPOW2;
  pHandle->hKdDivisor = ( ( uint16_t )( 1u ) << hKdDivisorPOW2 );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  PI_Controller：计算并更新相电流、故障与保护、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wProcessVarError:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t PI_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  /* 计算P=Kp*e和I+=Ki*e，积分项先限幅；P与缩放后的I求和后再做输出限幅。
   * 若输出发生饱和，代码回退相应积分增量，实现简单anti-windup，避免积分器继续朝饱和方向累积。 */
  int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* 中文说明：处理并更新相关数据和控制状态。 */
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* 中文说明：处理并更新相关数据和控制状态。 */
  if ( pHandle->hKiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0 )
    {
      if ( pHandle->wIntegralTerm > 0 )
      {
        if ( wIntegral_Term > 0 )
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }

    if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#else
  /* 中文说明：处理并更新相关数据和控制状态。 */
  wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
#endif

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* 中文说明：处理并更新相关数据和控制状态。 */ }

  pHandle->wIntegralTerm += wDischarge;

  return ( ( int16_t )( wOutput_32 ) );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  PID_Controller：计算并更新相电流、故障与保护、PI/PID调节器，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wProcessVarError:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */

__WEAK int16_t PID_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  /* 在PI基础上加入Kd*(本次误差-上次误差)。微分项放大噪声，本工程速度/电流环默认主要使用PI。 */
  int32_t wDifferential_Term;
  int32_t wDeltaError;
  int32_t wTemp_output;

  if ( pHandle->hKdGain != 0 ) /* 中文说明：处理并更新相关数据和控制状态。 */
  {
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    wDifferential_Term = pHandle->hKdGain * wDeltaError;

#ifdef FULL_MISRA_C_COMPLIANCY
    wDifferential_Term /= ( int32_t )pHandle->hKdDivisor;
#else
    /* 中文说明：处理并更新相关数据和控制状态。 */
    wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#endif

    pHandle->wPrevProcessVarError = wProcessVarError;

    wTemp_output = PI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;

    if ( wTemp_output > pHandle->hUpperOutputLimit )
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if ( wTemp_output < pHandle->hLowerOutputLimit )
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = PI_Controller( pHandle, wProcessVarError );
  }
  return ( ( int16_t ) wTemp_output );
}
/**
 * @}
 */

/**
 * @}
 */

/************************ （C）版权所有，文件结束 ************************/
