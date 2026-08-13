/* 用户代码开始 */
/**
  ******************************************************************************
  * @file    main.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */
/* 用户代码结束 */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __MAIN_H
#define __MAIN_H

/* 中文解读：应用公共硬件定义，包含LED、按键和三相PWM引脚别名。它只描述板级连接，
 * 电机控制状态和算法对象分别由mc_interface.h、mc_config.h管理。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 头文件 */
#include "et6039.h"

/* 私有头文件 */
/* 用户代码开始 */

/* 用户代码结束 */

/* 导出类型 */
/* 用户代码开始 */

/* 用户代码结束 */

/* 导出常量 */
/* 用户代码开始 */

/* 用户代码结束 */

/* 导出宏 */
/* 用户代码开始 */

/* 用户代码结束 */

// 中文说明：处理中断并更新故障与保护。

/* 导出函数 */
void Error_Handler(void);

/* 用户代码开始 */

/* 用户代码结束 */

/* 私有宏定义 */

//#define Start_Stop_Pin GPIO_PIN_4
//#define Start_Stop_EXTI_IRQn EXTI4_IRQn

#define UART_RX_Pin GPIO25
#define UART_TX_Pin GPIO2

#define M1_PWM_WH_Pin GPIO8
#define M1_PWM_VH_Pin GPIO14
#define M1_PWM_UH_Pin GPIO28

#define M1_ENCODER_Z_Pin GPIO26
#define M1_ENCODER_B_Pin GPIO33
#define M1_ENCODER_A_Pin GPIO11

#define M1_ENCODER_Z_EXTI_IRQn EXTI9_5_IRQn

#define SHUTDOWN1_Pin GPIO27



/* 用户代码开始 */

/* 用户代码结束 */

#ifdef __cplusplus
}
#endif

#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/************************ （C）版权所有，文件结束 ************************/
