/** @file digital_output.c
 *  @brief 通用数字输出抽象。把逻辑有效/无效与GPIO高低电平解耦，可供制动、使能或故障指示使用；
 *  activeHigh决定有效状态对应的物理电平。 */
/**
  ******************************************************************************
  * @file    digital_output.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "digital_output.h"
#include "mc_type.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup DigitalOutput 电机、数字输出功能模块
  * @brief 电机、数字输出相关组件及其公共接口。
  * @{
  */


/**
  * @brief  DOUT_SetOutputState：设置状态机状态、数字输出，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  OutputState_t  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void DOUT_SetOutputState( DOUT_handle_t * pHandle, DOutputState_t State )
{

  if ( State == ACTIVE )
  {
    if ( pHandle->bDOutputPolarity == DOutputActiveHigh )
    {
      GPIO_writePin(pHandle->hDOutputPin, SET);
    }
    else
    {
      GPIO_writePin(pHandle->hDOutputPin, RESET);
    }
  }
  else if ( pHandle->bDOutputPolarity == DOutputActiveHigh)
  {
    GPIO_writePin(pHandle->hDOutputPin, RESET);
  }
  else
  {
    GPIO_writePin(pHandle->hDOutputPin, SET);
  }
  pHandle->OutputState = State;
}

/**
  * @brief  DOUT_GetOutputState：读取并返回状态机状态、数字输出，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK DOutputState_t DOUT_GetOutputState( DOUT_handle_t * pHandle )
{
  return ( pHandle->OutputState );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
