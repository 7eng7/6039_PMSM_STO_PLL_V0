/**
  ******************************************************************************
  * @file    motor_control_protocol.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __MOTOR_CONTROL_PROTOCOL_H
#define __MOTOR_CONTROL_PROTOCOL_H

/* 中文解读：定义上位机通信命令、寄存器编号、错误码及帧处理接口，是协议层与MCSDK对象层
 * 之间的契约；修改编号会破坏与现有上位机的兼容性。 */

#ifdef __cplusplus
 extern "C" {
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 头文件 */
#include "frame_communication_protocol.h"

#include "user_interface.h"
#include "dac_ui.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/**
  * @addtogroup motor_control_protocol
  * @{
  */

 /**
  * @brief  当前接口：处理并更新串口通信帧、电机，保持对象状态和控制流程一致。
  */
typedef struct MCP_Handle_s
{
  UI_Handle_t _Super;     /**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/

  FCP_Handle_t *pFCP;
  FCP_SendFct_t fFcpSend;
  FCP_ReceiveFct_t fFcpReceive;
  FCP_AbortReceiveFct_t fFcpAbortReceive;
  uint8_t BufferFrame[FCP_MAX_PAYLOAD_SIZE];  /**
* @brief  MCP_Init：初始化缓冲区，保持对象状态和控制流程一致。
*/
  const char *s_fwVer;    /**
* @brief  MCP_Init：初始化相关数据和控制状态，保持对象状态和控制流程一致。
*/
  DAC_UI_Handle_t * pDAC; /**
* @brief  MCP_Init：初始化调试DAC，保持对象状态和控制流程一致。
*/
  uint8_t BufferSize;      /**
* @brief  MCP_Init：初始化缓冲区、串口通信帧，保持对象状态和控制流程一致。
*/

} MCP_Handle_t;

/* 导出类型 */

/* 中文说明：初始化串口通信帧、电机。 */
void MCP_Init( MCP_Handle_t *pHandle,
               FCP_Handle_t * pFCP,
               FCP_SendFct_t fFcpSend,
               FCP_ReceiveFct_t fFcpReceive,
               FCP_AbortReceiveFct_t fFcpAbortReceive,
               DAC_UI_Handle_t * pDAC,
               const char* s_fwVer);
void MCP_OnTimeOut(MCP_Handle_t *pHandle);

/* 中文说明：处理并更新串口通信帧。 */
void MCP_ReceivedFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size);

/* 中文说明：处理并更新串口通信帧。 */
void MCP_SentFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size);

/* 中文说明：检查并判断串口通信帧。 */
void MCP_WaitNextFrame(MCP_Handle_t *pHandle);

/* 中文说明：执行并推进故障与保护。 */
void MCP_SendOverrunMessage(MCP_Handle_t *pHandle);

/* 中文说明：处理并更新故障与保护。 */
void MCP_SendTimeoutMessage(MCP_Handle_t *pHandle);

/* 中文说明：处理并更新相关数据和控制状态。 */
void MCP_SendATRMessage(MCP_Handle_t *pHandle);

/* 中文说明：处理并更新ADC采样。 */
void MCP_SendBadCRCMessage(MCP_Handle_t *pHandle);

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
#ifdef __cplusplus
}
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#endif /* 中文说明：处理并更新串口通信帧、电机。 */

/************************ （C）版权所有，文件结束 ************************/
