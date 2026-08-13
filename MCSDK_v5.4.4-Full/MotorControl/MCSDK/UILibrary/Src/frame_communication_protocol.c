/** @file frame_communication_protocol.c
 *  @brief 通用帧协议基类。管理帧缓冲区、发送/接收状态、超时、校验和回调，具体USART驱动
 *  通过派生实现接入；上层只处理完整帧和协议错误。 */
/**
  ******************************************************************************
  * @file    frame_communication_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "user_interface.h"
#include "motor_control_protocol.h"

#include "frame_communication_protocol.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/**
  * @defgroup FCP 串口通信帧、电机功能模块
  * @brief 串口通信帧、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

__WEAK void FCP_Init( FCP_Handle_t * pHandle )
{
  pHandle->RxTimeoutCountdown = 0;

  pHandle->TxFrame.Code = 0x0;
  pHandle->TxFrame.Size = 0;
  pHandle->TxFrame.FrameCRC = 0;
  pHandle->TxFrameState = FCP_TRANSFER_IDLE;
  pHandle->TxFrameLevel = 0;

  pHandle->RxFrame.Code = 0x0;
  pHandle->RxFrame.Size = 0;
  pHandle->RxFrame.FrameCRC = 0;
  pHandle->RxFrameState = FCP_TRANSFER_IDLE;
  pHandle->RxFrameLevel = 0;
}

__WEAK void FCP_SetClient( FCP_Handle_t * pHandle,
                    struct MCP_Handle_s * pClient,
                    FCP_SentFrameCallback_t pSentFrameCb,
                    FCP_ReceivedFrameCallback_t pReceviedFrameCb,
                    FCP_RxTimeoutCallback_t pRxTimeoutCb )
{
  if ( MC_NULL != pHandle )
  {
    pHandle->ClientEntity = pClient;
    pHandle->ClientFrameSentCallback = pSentFrameCb;
    pHandle->ClientFrameReceivedCallback = pReceviedFrameCb;
    pHandle->ClientRxTimeoutCallback = pRxTimeoutCb;
  }
}

__WEAK void FCP_SetTimeout( FCP_Handle_t * pHandle, uint16_t Timeout )
{
  if ( MC_NULL != pHandle )
  {
    pHandle->RxTimeout = Timeout;
  }
}

__WEAK uint8_t FCP_CalcCRC( FCP_Frame_t * pFrame )
{
  uint8_t nCRC = 0;
  uint16_t nSum = 0;
  uint8_t idx;

  if( MC_NULL != pFrame )
  {
    nSum += pFrame->Code;
    nSum += pFrame->Size;

    for ( idx = 0; idx < pFrame->Size; idx++ )
    {
      nSum += pFrame->Buffer[idx];
    }

    nCRC = (uint8_t)(nSum & 0xFF) ; // 中文说明：检查并判断串口通信帧。
    nCRC += (uint8_t) (nSum >> 8) ; // 中文说明：检查并判断串口通信帧。
  }

  return nCRC ;
}

__WEAK uint8_t FCP_IsFrameValid( FCP_Frame_t * pFrame )
{
  if ( MC_NULL != pFrame )
    return FCP_CalcCRC(pFrame) == pFrame->Buffer[pFrame->Size];
  else
    return 0;
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

/************************ （C）版权所有，文件结束 ************************/
