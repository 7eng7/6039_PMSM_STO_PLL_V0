/** @file ui_irq_handler.c
 *  @brief 用户界面中断分发器。为通信实例登记中断编号、对象句柄和回调函数；统一入口按编号
 *  找到对应对象并调用处理器，使串口协议与具体中断向量解耦。该路径不参与FOC电流环。 */
/**
  ******************************************************************************
  * @file    ui_irq_handler.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "ui_irq_handler.h"
#include "mc_type.h"


/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/**
  * @defgroup UI_IRQ_HANDLER 串口通信帧、中断功能模块
  * @brief 串口通信帧、中断相关组件及其公共接口。
  * @{
  */


/* 私有类型定义 */
/**
  * @brief  当前接口：处理中断并更新中断、参数和增益，保持对象状态和控制流程一致。
  */
typedef struct
{
    UIIRQ_Handler_t Handler;
    void *          Handle;
} UIIRQ_HandlerConfigItem_t;


/* 私有宏定义 */
/**
* @brief  当前接口：处理中断并更新中断，保持对象状态和控制流程一致。
*/
#define MAX_UI_IRQ_NUM 1


/* 变量 */
/**
* @brief  UIIRQ_SetIrqHandler：设置中断，保持对象状态和控制流程一致。
*/
static UIIRQ_HandlerConfigItem_t oUI_IRQTable[MAX_UI_IRQ_NUM];


/* 函数 */
/**
  * @brief  UIIRQ_SetIrqHandler：设置中断，保持对象状态和控制流程一致。
  */

void UIIRQ_SetIrqHandler(uint8_t bIRQAddr, UIIRQ_Handler_t Handler, void * Handle)
{
    if ( bIRQAddr < MAX_UI_IRQ_NUM )
    {
        oUI_IRQTable[ bIRQAddr ].Handler = Handler;
        oUI_IRQTable[ bIRQAddr ].Handle  = Handle;
    }
}


/**
  * @brief  UIIRQ_ExecIrqHandler：处理中断并更新中断，保持对象状态和控制流程一致。
  */
void* UIIRQ_ExecIrqHandler(uint8_t bIRQAddr, uint8_t flag, uint16_t rx_data)
{  
  void * ret_val = MC_NULL;

  if ( bIRQAddr < MAX_UI_IRQ_NUM )
  {
      ret_val = oUI_IRQTable[ bIRQAddr ].Handler( oUI_IRQTable[ bIRQAddr ].Handle, flag, rx_data );
  }

  return ret_val;
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
