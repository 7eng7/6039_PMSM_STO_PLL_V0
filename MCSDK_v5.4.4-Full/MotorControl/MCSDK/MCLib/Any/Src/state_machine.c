/** @file state_machine.c
 *  @brief 电机运行/故障状态机。限制合法状态迁移以防绕过校准和安全检查；当前故障存在时进入
 *  FAULT_NOW，条件消失后为FAULT_OVER，只有确认命令才能清历史故障并回到IDLE。 */
/**
  ******************************************************************************
  * @file    state_machine.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "state_machine.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup STATE_MACHINE 状态机状态、电机功能模块
  * @brief 状态机状态、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  STM_Init：初始化相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STM_Init( STM_Handle_t * pHandle )
{

  pHandle->bState = IDLE;
  pHandle->hFaultNow = MC_NO_FAULTS;
  pHandle->hFaultOccurred = MC_NO_FAULTS;
}

/**
  * @brief  STM_NextState：设置相电流、故障与保护、状态机状态、电机，保持对象状态和控制流程一致。
  * @param  pHanlde  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bState  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STM_NextState( STM_Handle_t * pHandle, State_t bState )
{
  /* 根据当前状态的白名单检查目标状态。非法跳转返回false且保持原状态，例如不能从FAULT_NOW
   * 直接跳RUN，也不能绕过电流偏置校准从IDLE直接进入START。 */
  bool bChangeState = false;
  State_t bCurrentState = pHandle->bState;
  State_t bNewState = bCurrentState;

  switch ( bCurrentState )
  {
    case ICLWAIT:
      if ( bState == IDLE )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
    case IDLE:
      if ( ( bState == IDLE_START ) || ( bState == IDLE_ALIGNMENT )
           || ( bState == ICLWAIT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case IDLE_ALIGNMENT:
      if ( ( bState == ANY_STOP ) || ( bState == ALIGN_CHARGE_BOOT_CAP )
           || ( bState == ALIGN_OFFSET_CALIB ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case ALIGN_CHARGE_BOOT_CAP:
      if ( ( bState == ALIGN_OFFSET_CALIB ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case ALIGN_OFFSET_CALIB:
      if ( ( bState == ALIGN_CLEAR ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case ALIGN_CLEAR:
      if ( ( bState == ALIGNMENT ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case ALIGNMENT:
      if ( bState == ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case IDLE_START:
      if ( ( bState == ANY_STOP ) || ( bState == CHARGE_BOOT_CAP ) ||
           ( bState == START ) ||
           ( bState == OFFSET_CALIB ) || ( bState == IDLE_ALIGNMENT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case CHARGE_BOOT_CAP:
      if ( ( bState == OFFSET_CALIB ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case OFFSET_CALIB:
      if ( ( bState == CLEAR ) || ( bState == ANY_STOP ) || ( bState == WAIT_STOP_MOTOR ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

     case WAIT_STOP_MOTOR:
      if ( ( bState == CLEAR ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case CLEAR:
      if ( ( bState == START ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case START:
      if ( ( bState == SWITCH_OVER ) || ( bState == ANY_STOP ) || (bState == START_RUN) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case SWITCH_OVER:
      if ( ( bState == START ) || ( bState == ANY_STOP ) || (bState == START_RUN) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case START_RUN:
      if ( ( bState == RUN ) || ( bState == ANY_STOP ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case RUN:
      if ( bState == ANY_STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case ANY_STOP:
      if ( bState == STOP )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case STOP:
      if ( bState == STOP_IDLE )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;

    case STOP_IDLE:
      if ( ( bState == IDLE ) || ( bState == ICLWAIT ) )
      {
        bNewState = bState;
        bChangeState = true;
      }
      break;
    default:
      break;
  }

  if ( bChangeState )
  {
    pHandle->bState = bNewState;
  }
  else
  {
    if ( !( ( bState == IDLE_START ) || ( bState == IDLE_ALIGNMENT )
            || ( bState == ANY_STOP ) ) )
    {
      /* 中文说明：启动或使能故障与保护、状态机状态。 */
      STM_FaultProcessing( pHandle, MC_SW_ERROR, 0u );
    }
  }

  return ( bChangeState );
}

/**
  * @brief  STM_FaultProcessing：计算并更新相电流、故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @param  pHanlde  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hSetErrors  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hResetErrors  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK State_t STM_FaultProcessing( STM_Handle_t * pHandle, uint16_t hSetErrors, uint16_t
                                   hResetErrors )
{
  /* hSetErrors按位加入当前/历史故障，hResetErrors仅清当前条件；历史故障保留用于诊断。
   * 当前故障非零时强制FAULT_NOW，全部消失后转FAULT_OVER并等待显式确认。 */
  State_t LocalState =  pHandle->bState;

  /* 中文说明：设置相电流、故障与保护。 */
  pHandle->hFaultNow = ( pHandle->hFaultNow | hSetErrors ) & ( ~hResetErrors );
  pHandle->hFaultOccurred |= hSetErrors;

  if ( LocalState == FAULT_NOW )
  {
    if ( pHandle->hFaultNow == MC_NO_FAULTS )
    {
      pHandle->bState = FAULT_OVER;  //考虑自恢复
      LocalState = FAULT_OVER;
        
    }
  }
  else
  {
    if ( pHandle->hFaultNow != MC_NO_FAULTS )
    {
      pHandle->bState = FAULT_NOW;
      LocalState = FAULT_NOW;
    }
  }

  return ( LocalState );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  STM_GetState：读取并返回相电流、状态机状态，保持对象状态和控制流程一致。
  * @param  pHanlde  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK State_t STM_GetState( STM_Handle_t * pHandle )
{
  return ( pHandle->bState );
}


/**
  * @brief  STM_FaultAcknowledged：处理并更新故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @param  pHanlde  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STM_FaultAcknowledged( STM_Handle_t * pHandle )
{
  /* 只有FAULT_OVER允许确认；确认清历史故障并回IDLE。若物理故障仍存在，状态还在FAULT_NOW，
   * 此调用必然失败，从而避免软件强行带故障启动。 */
  bool bToBeReturned = false;

  if ( pHandle->bState == FAULT_OVER )
  {
    pHandle->bState = STOP_IDLE;
    pHandle->hFaultOccurred = MC_NO_FAULTS;
    bToBeReturned = true;
  }

  return ( bToBeReturned );
}


/**
  * @brief  STM_GetFaultState：读取并返回相电流、故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @param  pHanlde  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint32_t STM_GetFaultState( STM_Handle_t * pHandle )
{
  uint32_t LocalFaultState;

  LocalFaultState = ( uint32_t )( pHandle->hFaultOccurred );
  LocalFaultState |= ( uint32_t )( pHandle->hFaultNow ) << 16;

  return LocalFaultState;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/
