
/**
  ******************************************************************************
  * @file    regular_conversion_manager.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

/* 中文解读：普通ADC转换请求、状态、回调及句柄定义。其目的不是提高采样速度，而是在不破坏
 * PWM同步电流采样的前提下安全调度低速模拟量。 */

#ifdef __cplusplus
 extern "C" {
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 头文件 */
#include "stdint.h"
#include "stdbool.h"
#include "mc_stm_types.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup RCM
  * @{
  */

/* 导出类型 */

/**
  * @brief  void：执行并推进ADC采样、参数和增益，保持对象状态和控制流程一致。
  */
typedef struct
{
  ADC_Type * regADC;
  uint16_t  channel;
  uint32_t samplingTime;
} RegConv_t;

typedef enum
{
  RCM_USERCONV_IDLE,
  RCM_USERCONV_REQUESTED,
  RCM_USERCONV_EOC
}RCM_UserConvState_t;

typedef void (*RCM_exec_cb_t)(uint8_t handle, uint16_t data, void *UserData);

/* 导出函数 */

/* 中文说明：检查并判断ADC采样。 */
uint8_t RCM_RegisterRegConv(RegConv_t * regConv);

/* 中文说明：检查并判断ADC采样。 */
uint8_t RCM_RegisterRegConv_WithCB (RegConv_t * regConv, RCM_exec_cb_t fctCB, void *data);

/* 中文说明：执行并推进ADC采样。 */
uint16_t RCM_ExecRegularConv (uint8_t handle);

/* 中文说明：设置ADC采样。 */
bool RCM_RequestUserConv(uint8_t handle);

/* 返回最近一次用户规则转换值。 */
uint16_t RCM_GetUserConv(void);

/* 中文说明：执行并推进相关数据和控制状态。 */
void RCM_ExecUserConv (void);

/* 返回用户规则转换状态机的当前状态。 */
RCM_UserConvState_t RCM_GetUserConvState(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#endif /* 中文说明：处理并更新ADC采样。 */

/************************ （C）版权所有，文件结束 ************************/
