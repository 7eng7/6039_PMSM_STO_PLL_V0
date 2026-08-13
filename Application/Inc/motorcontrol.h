/**
  ******************************************************************************
  * @file    motorcontrol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：初始化电机。 */
#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

/* 中文解读：Motor Control中间件初始化入口声明，负责把静态配置对象交给任务和UI层。 */
#include "mc_config.h"
#include "parameters_conversion.h"
#include "mc_api.h"

#ifdef __cplusplus
 extern "C" {
#endif /* 中文说明：初始化电机。 */

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCAPI
  * @{
  */

/* 中文说明：初始化电机。 */
void MX_MotorControl_Init(void);


#ifdef __cplusplus
}
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#endif /* 中文说明：处理并更新电机。 */
/************************ （C）版权所有，文件结束 ************************/
