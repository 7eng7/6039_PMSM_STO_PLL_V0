/**
 * @file motorcontrol.c
 * @brief Motor Control 中间件统一初始化入口。
 *
 * 中文解读：MX_MotorControl_Init() 建立电机对象、调度任务和用户界面。它只完成软件模块
 * 初始化，不代表功率级已经使能；真正输出PWM必须由状态机收到启动命令并通过安全检查。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
#include "mc_tuning.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "motorcontrol.h"

#define FIRMWARE_VERS "ST MC SDK\tVer.5.4.4"
const char s_fwVer[32] = FIRMWARE_VERS;

MCI_Handle_t* pMCI[NBR_OF_MOTORS];
MCT_Handle_t* pMCT[NBR_OF_MOTORS];
uint32_t wConfig[NBR_OF_MOTORS] = {UI_CONFIG_M1,UI_CONFIG_M2};

/**
  * @brief  MX_MotorControl_Init：初始化中断、电机，保持对象状态和控制流程一致。
  */
__WEAK void MX_MotorControl_Init(void)
{
  /* 中文说明：处理中断并更新中断。 */
  SysTick_Config(SystemCoreClock/SYS_TICK_FREQUENCY);
  /* 中文说明：初始化电机。 */
  MCboot(pMCI,pMCT);
//  mc_lock_pins();

  /* 中文说明：初始化相关数据和控制状态。 */
  UI_TaskInit(wConfig,NBR_OF_MOTORS,pMCI,pMCT,s_fwVer);
}

