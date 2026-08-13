/** @file dac_common_ui.c
 *  @brief 调试DAC公共UI层。保存两个输出通道所选的MCSDK寄存器，周期读取内部变量、缩放并
 *  交给板级DAC实现；主要用于示波器观测，不改变控制目标。 */
/**
  ******************************************************************************
  * @file    dac_common_ui.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "dac_common_ui.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/**
  * @defgroup dac_common_ui 相电流、电压、调试DAC、电机功能模块
  * @brief 相电流、电压、调试DAC、电机相关组件及其公共接口。
  * @{
  */


/**
  * @brief  DAC_SetChannelConfig：设置串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  user  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bVariable  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void DAC_SetChannelConfig(UI_Handle_t *pHandle, DAC_Channel_t bChannel,
                              MC_Protocol_REG_t bVariable)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  pDacHandle->bChannel_variable[bChannel] = bVariable;
}

/**
  * @brief  DAC_GetChannelConfig：设置相电流、串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  user  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK MC_Protocol_REG_t DAC_GetChannelConfig(UI_Handle_t *pHandle, DAC_Channel_t bChannel)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  return (pDacHandle->bChannel_variable[bChannel]);
}

/**
  * @brief  DAC_SetUserChannelValue：设置调试DAC，保持对象状态和控制流程一致。
  * @param  user  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bUserChNumber  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hValue  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void DAC_SetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber,
                              int16_t hValue)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  pDacHandle->hUserValue[bUserChNumber] = hValue;
}

/**
  * @brief  DAC_GetUserChannelValue：读取并返回调试DAC，保持对象状态和控制流程一致。
  * @param  user  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bUserChNumber  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t DAC_GetUserChannelValue(UI_Handle_t *pHandle, uint8_t bUserChNumber)
{
  DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
  return (pDacHandle->hUserValue[bUserChNumber]);
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

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
