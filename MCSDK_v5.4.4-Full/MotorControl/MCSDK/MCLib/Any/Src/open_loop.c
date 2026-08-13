/** @file open_loop.c
 *  @brief 无感启动开环幅值生成。反电势尚不足以可靠估角时，按虚拟角度施加给定电压/电流幅值，
 *  幅值可随速度线性变化；STO可靠后状态机再切入闭环。 */
/**
  ******************************************************************************
  * @file    open_loop.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "open_loop.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup OpenLoop 电机功能模块
  * @brief 电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 私有宏定义 */

/**
  * @brief  OL_Init：初始化速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pVSS:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void OL_Init( OpenLoop_Handle_t * pHandle, VirtualSpeedSensor_Handle_t * pVSS )
{
  pHandle->hVoltage = pHandle->hDefaultVoltage;
  pHandle->pVSS = pVSS;
}

/**
  * @brief  OL_VqdConditioning：设置电压，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t OL_VqdConditioning( OpenLoop_Handle_t * pHandle )
{
  qd_t Vqd;

  Vqd.q = pHandle->hVoltage;
  Vqd.d = 0;

  return ( Vqd );
}

/**
  * @brief  OL_UpdateVoltage：设置电压，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hNewVoltage:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void OL_UpdateVoltage( OpenLoop_Handle_t * pHandle, int16_t hNewVoltage )
{
  pHandle->hVoltage = hNewVoltage;
}

/**
  * @brief  OL_Calc：计算并更新电压、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void OL_Calc( OpenLoop_Handle_t * pHandle )
{
  if ( pHandle->VFMode == true )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    if (pHandle->pVSS->_Super.hAvrMecSpeedUnit >= 0)
    {
      pHandle->hVoltage = pHandle->hVFOffset + ( pHandle->hVFSlope * pHandle->pVSS->_Super.hAvrMecSpeedUnit );
    }
    else
    {
      pHandle->hVoltage = pHandle->hVFOffset - ( pHandle->hVFSlope * pHandle->pVSS->_Super.hAvrMecSpeedUnit );
    }
  }
}

/**
  * @brief  OL_VF：启动或使能电压，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  VFEnabling:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void OL_VF( OpenLoop_Handle_t * pHandle, bool VFEnabling )
{
  pHandle->VFMode = VFEnabling;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
