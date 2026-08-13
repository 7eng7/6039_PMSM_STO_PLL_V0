/** @file motor_power_measurement.c
 *  @brief 电机功率测量抽象接口。定义瞬时/平均功率的统一调用方式，具体d-q功率计算由派生对象实现；
 *  功率主要用于监视，不直接参与本工程速度和电流闭环。 */
/**
  ******************************************************************************
  * @file    motor_power_measurement.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup motorpowermeasurement 功率、电机功能模块
  * @brief 功率、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 头文件 */
#include "motor_power_measurement.h"

#include "mc_type.h"


/**
  * @brief  MPM_Clear：初始化缓冲区、功率、电机，保持对象状态和控制流程一致。
  * @param  power  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MPM_Clear( MotorPowMeas_Handle_t * pHandle )
{
  uint16_t i;
  for ( i = 0u; i < MPM_BUFFER_LENGHT; i++ )
  {
    pHandle->hMeasBuffer[i] = 0;
  }
  pHandle->hNextMeasBufferIndex = 0u;
  pHandle->hLastMeasBufferIndex = 0u;

}

/**
  * @brief  MPM_CalcElMotorPower：计算并更新缓冲区、功率、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MPM_CalcElMotorPower( MotorPowMeas_Handle_t * pHandle, int16_t CurrentMotorPower )
{
  uint16_t i;
  int32_t wAux = 0;

  /* 中文说明：处理并更新缓冲区。 */
  pHandle->hMeasBuffer[pHandle->hNextMeasBufferIndex] = CurrentMotorPower;
  pHandle->hLastMeasBufferIndex = pHandle->hNextMeasBufferIndex;
  pHandle->hNextMeasBufferIndex++;
  if ( pHandle->hNextMeasBufferIndex >= MPM_BUFFER_LENGHT )
  {
    pHandle->hNextMeasBufferIndex = 0u;
  }
  /* 中文说明：计算并更新功率、电机。 */
  for ( i = 0u; i < MPM_BUFFER_LENGHT; i++ )
  {
    wAux += ( int32_t )( pHandle->hMeasBuffer[i] );
  }
  wAux /= ( int32_t )MPM_BUFFER_LENGHT;
  pHandle->hAvrgElMotorPowerW = ( int16_t )( wAux );
  /* 返回最近一次测得的电机功率。 */
  return CurrentMotorPower;
}
/**
  * @brief  MPM_GetElMotorPowerW：读取并返回功率、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MPM_GetElMotorPowerW( MotorPowMeas_Handle_t * pHandle )
{
  return ( pHandle->hMeasBuffer[pHandle->hLastMeasBufferIndex] );
}

/**
  * @brief  MPM_GetAvrgElMotorPowerW：读取并返回功率、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t MPM_GetAvrgElMotorPowerW( MotorPowMeas_Handle_t * pHandle )
{
  return ( pHandle->hAvrgElMotorPowerW );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
