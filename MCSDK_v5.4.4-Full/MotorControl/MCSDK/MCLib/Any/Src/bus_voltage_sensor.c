/** @file bus_voltage_sensor.c
 *  @brief 母线电压传感器抽象基类。保存最新采样、滤波值和故障状态，为具体传感器提供统一接口；
 *  控制内部使用数字单位，显示接口可换算为伏特。 */
/**
  ******************************************************************************
  * @file    bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件的中文功能、作用和原理说明见紧邻的中文解读注释；
  *          其中各接口继续使用原有Doxygen标签说明参数和返回值。
  ******************************************************************************
  */

/* 头文件 */

#include "bus_voltage_sensor.h"


/**
  * @addtogroup MCSDK
  * @{
  */

/** @defgroup BusVoltageSensor 母线电压传感器
  * @brief MCSDK的母线电压传感器组件。
  *
  * 该抽象层支持电阻分压实测母线电压，也支持由应用提供固定值的虚拟母线电压传感器。
  *
  * @{
  */

/**
  * @brief  VBS_GetBusVoltage_d：读取并返回电压、ADC采样，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t VBS_GetBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->LatestConv );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  VBS_GetAvBusVoltage_d：读取并返回电压，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->AvBusVoltage_d );
}

/**
  * @brief  VBS_GetAvBusVoltage_V：读取并返回电压，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle )
{
  uint32_t temp;

  temp = ( uint32_t )( pHandle->AvBusVoltage_d );
  temp *= pHandle->ConversionFactor;
  temp /= 65536u;

  return ( ( uint16_t )temp );
}

/**
  * @brief  VBS_CheckVbus：检查并判断母线电压、电压、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t VBS_CheckVbus( BusVoltageSensor_Handle_t * pHandle )
{
  return ( pHandle->FaultState );
}

/**
  * @}
  */

/**
  * @}
  */

/**
  ******************************************************************************
  * @file    bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */
