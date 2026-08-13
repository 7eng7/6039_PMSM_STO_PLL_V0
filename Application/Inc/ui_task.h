/**
  ******************************************************************************
  * @file    ui_task.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：初始化相关数据和控制状态。 */
#ifndef __UITASK_H
#define __UITASK_H

/* 中文解读：用户界面任务初始化和周期处理接口，连接串口协议、DAC监视和电机控制寄存器。 */

#include "user_interface.h"
#include "dac_ui.h"

#include "motor_control_protocol.h"
#include "frame_communication_protocol.h"
#include "usart_frame_communication_protocol.h"
#include "ui_irq_handler.h"

/* 导出函数 */
void UI_TaskInit(uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t * pMCIList[],
                 MCT_Handle_t* pMCTList[],const char* s_fwVer);

void UI_DACUpdate(uint8_t bMotorNbr);
UI_Handle_t * GetDAC(void);

void UI_Scheduler(void);
MCP_Handle_t * GetMCP(void);

bool UI_IdleTimeHasElapsed(void);
void UI_SetIdleTime(uint16_t SysTickCount);
bool UI_SerialCommunicationTimeOutHasElapsed(void);
bool UI_SerialCommunicationATRTimeHasElapsed(void);
void UI_SerialCommunicationTimeOutStop(void);
void UI_SerialCommunicationTimeOutStart(void);

void UI_HandleStartStopButton_cb (void);

#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/************************ （C）版权所有，文件结束 ************************/
