/** @file usart_frame_communication_protocol.c
 *  @brief 基于USART的帧收发状态机。接收中断依次解析命令、长度、负载和校验，超时或校验失败
 *  丢弃当前帧；发送端逐字节推进并在完成时回调。状态机避免阻塞等待串口。 */
/**
  ******************************************************************************
  * @file    usart_frame_communication_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */


    /* ****************************************** */
    /* 中文说明：处理并更新相关数据和控制状态。 */
    /* ****************************************** */

/* 头文件 */
#include "usart_frame_communication_protocol.h"
#include "ui_irq_handler.h"
#include "mc_config.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/**
  * @defgroup UFCP 串口通信帧、电机功能模块
  * @brief 串口通信帧、电机相关组件及其公共接口。
  * @{
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#define UFCP_IRQ_FLAG_RX      0
#define UFCP_IRQ_FLAG_TX      1
#define UFCP_IRQ_FLAG_OVERRUN 2
#define UFCP_IRQ_FLAG_TIMEOUT 3
#define UFCP_IRQ_FLAG_ATR     4

/* 私有函数声明 */


/* 私有变量 */
static const uint16_t UFCP_Usart_Timeout_none = 0;
static const uint16_t UFCP_Usart_Timeout_start = 1;
static const uint16_t UFCP_Usart_Timeout_stop = 2;

/* 函数 */

__WEAK void UFCP_Init( UFCP_Handle_t * pHandle )
{

  /* 中文说明：初始化中断。 */
  FCP_Init( & pHandle->_Super );
}

/*
 *
 */
__WEAK void * UFCP_RX_IRQ_Handler( UFCP_Handle_t * pHandle, unsigned short rx_data )
{
  void * ret_val = (void *) & UFCP_Usart_Timeout_none;
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  if ( FCP_TRANSFER_IDLE != pBaseHandle->RxFrameState )
  {
    uint8_t rx_byte = (uint8_t) rx_data;

    switch ( pBaseHandle->RxFrameLevel )
    {
      case 0: // 中文说明：处理并更新相关数据和控制状态。
        pBaseHandle->RxFrame.Code = rx_byte;
        /* 中文说明：启动或使能相关数据和控制状态。 */
        ret_val = (void *) & UFCP_Usart_Timeout_start;

        /* 中文说明：启动或使能相关数据和控制状态。 */
        pBaseHandle->RxTimeoutCountdown = pBaseHandle->RxTimeout;
        pBaseHandle->RxFrameLevel++;
        break;

      case 1: // 中文说明：处理并更新相关数据和控制状态。
        pBaseHandle->RxFrame.Size = rx_byte;
        pBaseHandle->RxFrameLevel++;
        if ( pBaseHandle->RxFrame.Size >= FCP_MAX_PAYLOAD_SIZE)
        { /* 中文说明：处理并更新相关数据和控制状态。 */
          pBaseHandle->RxFrameLevel =0 ;
        }
        break;

      default: // 中文说明：处理并更新相关数据和控制状态。
        if ( pBaseHandle->RxFrameLevel < pBaseHandle->RxFrame.Size + FCP_HEADER_SIZE )
        {
          // 中文说明：读取并返回相关数据和控制状态。
          pBaseHandle->RxFrame.Buffer[pBaseHandle->RxFrameLevel - FCP_HEADER_SIZE] = rx_byte;
          pBaseHandle->RxFrameLevel++;
        }
        else
        {
          // 中文说明：读取并返回相关数据和控制状态。
          pBaseHandle->RxFrame.FrameCRC = rx_byte;

          /* 中文说明：停止或禁用相关数据和控制状态。 */
          ret_val = (void *) & UFCP_Usart_Timeout_stop;

          /* 中文说明：停止或禁用相关数据和控制状态。 */
          pBaseHandle->RxTimeoutCountdown = 0;
          /* 中文说明：停止或禁用中断。 */
          UART_disableInterrupt(pHandle->USARTx, UART_IT_RECV_DATA_AVAILABLE_FLAG);
          /* 中文说明：处理并更新相关数据和控制状态。 */
          pBaseHandle->RxFrameState = FCP_TRANSFER_IDLE;

          /* 中文说明：检查并判断相关数据和控制状态。 */
          if ( FCP_CalcCRC( & pBaseHandle->RxFrame ) == pBaseHandle->RxFrame.FrameCRC )
          {
            /* 中文说明：处理并更新串口通信帧。 */
            pBaseHandle->ClientFrameReceivedCallback( pBaseHandle->ClientEntity,
                                                      pBaseHandle->RxFrame.Code,
                                                      pBaseHandle->RxFrame.Buffer,
                                                      pBaseHandle->RxFrame.Size );
          }
          else
          {
            error_code = FCP_MSG_RX_BAD_CRC;
            (void) UFCP_Send( pBaseHandle, FCP_CODE_NACK, & error_code, 1 );
          }
        }
    } /* 中文说明：处理中断并更新串口通信帧、中断。 */
  } /* 中文说明：处理中断并更新状态机状态、串口通信帧、中断。 */

  return ret_val;
}

/*
 *
 */
__WEAK void UFCP_TX_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;

  if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState )
  {
    uint16_t tx_data;

    switch ( pBaseHandle->TxFrameLevel )
    {
      case 0:
        tx_data = (uint16_t) pBaseHandle->TxFrame.Code;
        break;

      case 1:
        tx_data = (uint16_t) pBaseHandle->TxFrame.Size;
        break;

      default:
        if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size + FCP_HEADER_SIZE )
        {
          tx_data = (uint16_t) pBaseHandle->TxFrame.Buffer[ pBaseHandle->TxFrameLevel - FCP_HEADER_SIZE ];
        }
        else
        {
          tx_data = (uint16_t) pBaseHandle->TxFrame.FrameCRC;
        }
    } /* 中文说明：处理并更新串口通信帧。 */

    /* 中文说明：处理并更新相关数据和控制状态。 */
    UART_writeCharNonBlocking(pHandle->USARTx, tx_data);

    if ( pBaseHandle->TxFrameLevel < pBaseHandle->TxFrame.Size + FCP_HEADER_SIZE )
    {
      pBaseHandle->TxFrameLevel++;
    }
    else
    {
      UART_disableInterrupt(pHandle->USARTx, UART_IT_TRANS_DATA_EMPTY_FLAG);
      pBaseHandle->TxFrameState = FCP_TRANSFER_IDLE;

      pBaseHandle->ClientFrameSentCallback( pBaseHandle->ClientEntity );
    }

  } /* 中文说明：处理中断并更新状态机状态、串口通信帧、中断。 */
}

/*
 *
 */
__WEAK void UFCP_OVR_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = UFCP_MSG_OVERRUN;
  (void) UFCP_Send( pBaseHandle, FCP_CODE_NACK, & error_code, 1 );

}

/*
 *
 */
__WEAK void UFCP_TIMEOUT_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = FCP_MSG_RX_TIMEOUT;
  (void) UFCP_Send( pBaseHandle, FCP_CODE_NACK, & error_code, 1 );

}

__WEAK uint8_t UFCP_Receive( FCP_Handle_t * pHandle )
{
  uint8_t ret_val;

  if ( FCP_TRANSFER_IDLE == pHandle->RxFrameState )
  {
    UFCP_Handle_t * pActualHandle = (UFCP_Handle_t *) pHandle;

    pHandle->RxFrameLevel = 0;
    pHandle->RxFrameState = FCP_TRANSFER_ONGOING;

    UART_enableInterrupt(pActualHandle->USARTx, UART_IT_RECV_DATA_AVAILABLE_FLAG);
    ret_val = FCP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

__WEAK uint8_t UFCP_Send( FCP_Handle_t * pHandle, uint8_t code, uint8_t *buffer, uint8_t size)
{
  uint8_t ret_val;

  if ( FCP_TRANSFER_IDLE == pHandle->TxFrameState )
  {
    UFCP_Handle_t * pActualHandle = (UFCP_Handle_t *) pHandle;
    uint8_t *dest = pHandle->TxFrame.Buffer;

    pHandle->TxFrame.Code = code;
    pHandle->TxFrame.Size = size;
    while ( size-- ) *dest++ = *buffer++;
    pHandle->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->TxFrame );

    pHandle->TxFrameLevel = 0;  //1
    pHandle->TxFrameState = FCP_TRANSFER_ONGOING;

    UART_enableInterrupt(pActualHandle->USARTx, UART_IT_TRANS_DATA_EMPTY_FLAG);
//    WRITE_REG(pActualHandle->USARTx->THR, pHandle->TxFrame.Code);
    ret_val = FCP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

__WEAK void UFCP_AbortReceive( FCP_Handle_t * pHandle )
{
  pHandle->RxFrameState = FCP_TRANSFER_IDLE;
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
