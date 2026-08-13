/**
  ******************************************************************************
  * @file    parameters_conversion_f4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __PARAMETERS_CONVERSION_F4XX_H
#define __PARAMETERS_CONVERSION_F4XX_H

/* 中文解读：保留的MCSDK平台兼容层名称。当前工程虽沿用f4xx文件名，实际中断和外设已映射到
 * ET6039；不要按文件名误认为仍在使用STM32 TIM/ADC寄存器。 */

#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "drive_parameters.h"
#include "mc_math.h"

/**
* @brief  当前接口：处理并更新ADC采样，保持对象状态和控制流程一致。
*/

#define SYSCLK_FREQ      200000000uL
#define TIM_CLOCK_DIVIDER  1
#define TIMAUX_CLOCK_DIVIDER (TIM_CLOCK_DIVIDER)
#define ADV_TIM_CLK_MHz  200/TIM_CLOCK_DIVIDER   //ET6039工作时钟200MHz
#define ADC_CLK_MHz     67
//#define HALL_TIM_CLK    84000000uL

#define ADC1_2  ADC1

/**
* @brief  当前接口：处理中断并更新中断，保持对象状态和控制流程一致。
*/
#define TIMx_UP_M1_IRQHandler TIM1_UP_TIM10_IRQHandler
#define DMAx_R1_M1_IRQHandler DMA2_Stream4_IRQHandler
#define DMAx_R1_M1_Stream     DMA2_Stream4
//#define TIMx_BRK_M1_IRQHandler TIM1_BRK_TIM9_IRQHandler

/**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
/* 中文说明：处理并更新相关数据和控制状态。 */
#define R1_PWM_AUX_TIM                  TIM4

/**
* @brief  当前接口：处理并更新ADC采样，保持对象状态和控制流程一致。
*/
#define ADC_TRIG_CONV_LATENCY_CYCLES 3
#define ADC_SAR_CYCLES 12

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

#endif /* 中文说明：处理并更新ADC采样、参数和增益。 */

/************************ （C）版权所有，文件结束 ************************/
