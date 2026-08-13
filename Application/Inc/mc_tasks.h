/**
  ******************************************************************************
  * @file    mc_tasks.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __MCTASKS_H
#define __MCTASKS_H

/* 中文解读：高频FOC、中频速度/状态机、低频安全任务及电机控制初始化接口。调用频率不同，
 * 高频函数必须保持确定执行时间，慢速通信和阻塞操作只能放在低优先级上下文。 */

/* 头文件 */
#include "mc_tuning.h"
#include "mc_interface.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup MCTasks 电机功能模块
  * @brief 电机相关组件及其公共接口。
  * @{
  */

/* 中文说明：初始化电机、参数和增益。 */
void MCboot( MCI_Handle_t* pMCIList[], MCT_Handle_t* pMCTList[] );

/* 中文说明：执行并推进电机。 */
void MC_RunMotorControlTasks(void);

/* 函数 */
void MC_Scheduler(void);

/* 中文说明：执行并推进母线电压、电压、温度。 */
void TSK_SafetyTask(void);

/* 中文说明：执行并推进电机。 */
uint8_t TSK_HighFrequencyTask(void);

/* 中文说明：处理并更新PWM和占空比、ADC采样、缓冲区。 */
void TSK_DualDriveFIFOUpdate(uint8_t Motor);

/* 中文说明：执行并推进故障与保护、电机。 */
void TSK_HardwareFaultTask(void);

 /* 中文说明：处理并更新电机、数字输出。 */
void mc_lock_pins (void);
/**
  * @}
  */

/**
  * @}
  */

#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/************************ （C）版权所有，文件结束 ************************/
