/** @file ntc_temperature_sensor.c
 *  @brief NTC温度采样、滤波与过温保护。ADC样本经低通后与阈值比较；恢复阈值带滞回，防止温度
 *  在临界点附近反复置位/清除MC_OVER_TEMP。传感器关闭时返回配置的默认温度。 */
/**
  ******************************************************************************
  * @file    ntc_temperature_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "ntc_temperature_sensor.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup TemperatureSensor PWM和占空比、ADC采样、温度、电机、数字输出功能模块
  * @brief PWM和占空比、ADC采样、温度、电机、数字输出相关组件及其公共接口。
  * @{
  */

/* 私有函数声明 */
uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle );

/* 私有函数 */

/**
  * @brief  NTC_SetFaultState：设置电压、故障与保护、温度、状态机状态，保持对象状态和控制流程一致。
  */
__WEAK uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle )
{
  uint16_t hFault;

  if ( pHandle->hAvTemp_d > pHandle->hOverTempThreshold )
  {
    hFault = MC_OVER_TEMP;
  }
  else if ( pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold )
  {
    hFault = MC_NO_ERROR;
  }
  else
  {
    hFault = pHandle->hFaultState;
  }
  return hFault;
}

/* 函数 */

/**
  * @brief  NTC_Init：初始化相电流、PWM和占空比、ADC采样、温度，保持对象状态和控制流程一致。
  */
__WEAK void NTC_Init( NTC_Handle_t * pHandle )
{

  if ( pHandle->bSensorType == REAL_SENSOR )     //如果是真的ADC采样就会被注册，否则不注册
  {
    /* 中文说明：清除并复位温度。 */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->TempRegConv);
    NTC_Clear( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
    pHandle->hAvTemp_d = pHandle->hExpectedTemp_d;
  }

}

/**
  * @brief  NTC_Clear：初始化温度，保持对象状态和控制流程一致。
  */
__WEAK void NTC_Clear( NTC_Handle_t * pHandle )
{
  pHandle->hAvTemp_d = 0u;
}

/**
  * @brief  NTC_CalcAvTemp：计算并更新ADC采样、故障与保护、温度，保持对象状态和控制流程一致。
  */
__WEAK uint16_t NTC_CalcAvTemp( NTC_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
      wtemp *= ( uint32_t ) ( pHandle->hAvTemp_d );
      wtemp += hAux;
      wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );

      pHandle->hAvTemp_d = ( uint16_t ) wtemp;
    }

    pHandle->hFaultState = NTC_SetFaultState( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

  return ( pHandle->hFaultState );
}

/**
  * @brief  NTC_GetAvTemp_d：读取并返回相电流、温度，保持对象状态和控制流程一致。
  */
__WEAK uint16_t NTC_GetAvTemp_d( NTC_Handle_t * pHandle )
{
  return ( pHandle->hAvTemp_d );
}

/**
  * @brief  NTC_GetAvTemp_C：读取并返回温度，保持对象状态和控制流程一致。
  */
__WEAK int16_t NTC_GetAvTemp_C( NTC_Handle_t * pHandle )
{
  int32_t wTemp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    wTemp = ( int32_t )( pHandle->hAvTemp_d );
    wTemp -= ( int32_t )( pHandle->wV0 );
    wTemp *= pHandle->hSensitivity;
    wTemp = wTemp / 65536 + ( int32_t )( pHandle->hT0 );
  }
  else
  {
    wTemp = pHandle->hExpectedTemp_C;
  }
  return ( ( int16_t )wTemp );
}

/**
  * @brief  NTC_CheckTemp：读取并返回故障与保护、温度、状态机状态，保持对象状态和控制流程一致。
  */
__WEAK uint16_t NTC_CheckTemp( NTC_Handle_t * pHandle )
{
  return ( pHandle->hFaultState );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
