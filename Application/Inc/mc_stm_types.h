/**
  ******************************************************************************
  * @file    mc_stm_types.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */
#ifndef __MC_STM_TYPES_H
#define __MC_STM_TYPES_H

/* 中文解读：电机状态机枚举和状态类型。这里的STM表示State Machine而非STM32芯片；
 * FAULT_NOW表示故障条件仍在，FAULT_OVER表示条件消失但等待应用确认。 */

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

#ifdef MISRA_C_2004_BUILD
#error "The code is not ready for that..."
#endif

#include "main.h"
#include "et6x.h"
#include "et6x_gpio.h"
#include "et6x_adc.h"
#include "et6x_eqep.h"
#include "et6x_srpwm.h"
#include "et6x_uart.h"
#include "init_config.h"
#include "et6x_cortex.h"
#include "et6x_interrupt.h"
#include "key.h"
//  #include "stm32f4xx_ll_dma.h"
//  #include "stm32f4xx_ll_bus.h"
/**
  * @brief  当前接口：设置速度、参数和增益，保持对象状态和控制流程一致。
  */
/** @{ */
/**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
#define _RPM 60
/**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
#define _01HZ 10
/**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
#define _001HZ 100
/** @} */

/* 用户代码开始 */
/* 中文说明：处理并更新相关数据和控制状态。 */
/**
  * @brief  当前接口：设置速度、参数和增益，保持对象状态和控制流程一致。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
#define SPEED_UNIT _01HZ

/**
  * @brief  当前接口：处理并更新位置和角度，保持对象状态和控制流程一致。
  */
/*#define CIRCLE_LIMITATION_VD*/

/* 用户代码结束 */

#endif /* 中文说明：处理并更新相关数据和控制状态。 */
/************************ （C）版权所有，文件结束 ************************/
