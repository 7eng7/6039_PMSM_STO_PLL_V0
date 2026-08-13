/** @file r_divider_bus_voltage_sensor.c
 *  @brief 电阻分压母线电压采样与保护。ADC样本滑动平均后与欠压/过压阈值比较，并用滞回防止
 *  临界抖动；故障交由安全任务锁存，电压恢复后仍需应用确认。 */
/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"


/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup BusVoltageSensor
  * @{
  */

/**
  * @defgroup RDividerBusVoltageSensor 母线电压、电压功能模块
  * @brief 母线电压、电压相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  RVBS_Init：初始化母线电压、电压、PWM和占空比、ADC采样，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RVBS_Init( RDivider_Handle_t * pHandle )
{
  /* 中文说明：清除并复位相关数据和控制状态。 */
  pHandle->convHandle = RCM_RegisterRegConv(&pHandle->VbusRegConv);
  /* 中文说明：清除并复位相关数据和控制状态。 */
  RVBS_Clear( pHandle );
}


/**
  * @brief  RVBS_Clear：清除并复位母线电压、电压，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void RVBS_Clear( RDivider_Handle_t * pHandle )
{
  uint16_t aux;
  uint16_t index;

  aux = ( pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold ) / 2u;
  for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
  {
    pHandle->aBuffer[index] = aux;
  }
  pHandle->_Super.LatestConv = aux;
  pHandle->_Super.AvBusVoltage_d = aux;
  pHandle->index = 0;
}

static uint16_t RVBS_ConvertVbusFiltrered( RDivider_Handle_t * pHandle )
{
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max = 0, min = 0;
  uint32_t tot = 0u;

  for ( vindex = 0; vindex < pHandle->LowPassFilterBW; )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      if ( vindex == 0 )
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if ( hAux < min )
        {
          min = hAux;
        }
        if ( hAux > max )
        {
          max = hAux;
        }
      }
      vindex++;

      tot += hAux;
    }
  }

  tot -= max;
  tot -= min;
  return ( uint16_t )( tot / ( pHandle->LowPassFilterBW - 2u ) );
}

/**
  * @brief  RVBS_CalcAvVbusFilt：计算并更新ADC采样、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t RVBS_CalcAvVbusFilt( RDivider_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RVBS_ConvertVbusFiltrered( pHandle );

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;
    pHandle->_Super.LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );

  return ( pHandle->_Super.FaultState );
}

/**
  * @brief  RVBS_CalcAvVbus：计算并更新ADC采样、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t RVBS_CalcAvVbus( RDivider_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RCM_ExecRegularConv(pHandle->convHandle);

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;
    pHandle->_Super.LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );

  return ( pHandle->_Super.FaultState );
}

/**
  * @brief  RVBS_CheckFaultState：检查并判断母线电压、电压、故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle )
{
  uint16_t fault;

  if ( pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold )
  {
    fault = MC_OVER_VOLT;
  }
  else if ( pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold )
  {
    fault = MC_UNDER_VOLT;
  }
  else
  {
    fault = MC_NO_ERROR;
  }
  return fault;
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ （C）版权所有，文件结束 ************************/

