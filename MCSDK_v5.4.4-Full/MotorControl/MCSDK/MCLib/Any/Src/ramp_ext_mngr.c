/** @file ramp_ext_mngr.c
 *  @brief 通用定点线性斜坡。依据起点、终点、持续时间和调用频率计算步进，并用扩展精度累加
 *  避免小增量被截断；速度/转矩平滑变化可降低冲击，时间为0表示立即到达。 */
/**
  ******************************************************************************
  * @file    ramp_ext_mngr.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "ramp_ext_mngr.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup RampExtMngr 斜坡、电机功能模块
  * @brief 斜坡、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */
/* 私有函数声明 */
uint32_t getScalingFactor( int32_t Target );

/**
  * @brief  REMNG_Init：初始化状态机状态、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void REMNG_Init( RampExtMngr_Handle_t * pHandle )
{
  pHandle->Ext = 0;
  pHandle->TargetFinal = 0;
  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
  pHandle->ScalingFactor = 1u;

#ifdef FASTDIV
  FD_Init( & ( pHandle->fd ) );
#endif

}

/**
  * @brief  REMNG_Calc：计算并更新相电流、状态机状态、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int32_t REMNG_Calc( RampExtMngr_Handle_t * pHandle )
{
  int32_t ret_val;
  int32_t current_ref;

  current_ref = pHandle->Ext;

  /* 中文说明：处理并更新斜坡。 */
  if ( pHandle->RampRemainingStep > 1u )
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
    current_ref += pHandle->IncDecAmount;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    pHandle->RampRemainingStep --;
  }
  else if ( pHandle->RampRemainingStep == 1u )
  {
    /* 中文说明：设置相关数据和控制状态。 */
    current_ref = pHandle->TargetFinal * ( int32_t )( pHandle->ScalingFactor );
    pHandle->RampRemainingStep = 0u;
  }
  else
  {
    /* 中文说明：处理并更新相关数据和控制状态。 */
  }

  pHandle->Ext = current_ref;

#ifdef FASTDIV
  ret_val = FD_FastDiv( & ( pHandle->fd ), pHandle->Ext, ( int32_t )( pHandle->ScalingFactor ) );
#else
  ret_val = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );
#endif

  return ret_val;
}

/**
  * @brief  REMNG_ExecRamp：设置状态机状态、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hTargetFinal  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool REMNG_ExecRamp( RampExtMngr_Handle_t * pHandle, int32_t TargetFinal, uint32_t Durationms )
{
  uint32_t aux;
  int32_t aux1;
  int32_t current_ref;
  bool retVal = true;

  /* 中文说明：读取并返回相电流、状态机状态。 */
#ifdef FASTDIV
  current_ref = FD_FastDiv( &( pHandle->fd ), pHandle->Ext, ( int32_t )( pHandle->ScalingFactor ) );
#else
  current_ref = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );
#endif

  if ( Durationms == 0u )
  {
    pHandle->ScalingFactor = getScalingFactor( TargetFinal );
    pHandle->Ext = TargetFinal * ( int32_t )( pHandle->ScalingFactor );
    pHandle->RampRemainingStep = 0u;
    pHandle->IncDecAmount = 0;
  }
  else
  {
    uint32_t wScalingFactor = getScalingFactor( TargetFinal - current_ref );
    uint32_t wScalingFactor2 = getScalingFactor( current_ref );
    uint32_t wScalingFactor3 = getScalingFactor( TargetFinal );
    uint32_t wScalingFactorMin;

    if ( wScalingFactor <  wScalingFactor2 )
    {
      if ( wScalingFactor < wScalingFactor3 )
      {
        wScalingFactorMin = wScalingFactor;
      }
      else
      {
        wScalingFactorMin = wScalingFactor3;
      }
    }
    else
    {
      if ( wScalingFactor2 < wScalingFactor3 )
      {
        wScalingFactorMin = wScalingFactor2;
      }
      else
      {
        wScalingFactorMin = wScalingFactor3;
      }
    }

    pHandle->ScalingFactor = wScalingFactorMin;
    pHandle->Ext = current_ref * ( int32_t )( pHandle->ScalingFactor );

    /* 中文说明：处理并更新相关数据和控制状态。 */
    pHandle->TargetFinal = TargetFinal;

    /* 中文说明：计算并更新斜坡。 */
    aux = Durationms * ( uint32_t )pHandle->FrequencyHz; /* 中文说明：检查并判断相关数据和控制状态。 */
    aux /= 1000u;
    pHandle->RampRemainingStep = aux;
    pHandle->RampRemainingStep++;

    /* 中文说明：读取并返回转矩。 */
    aux1 = ( TargetFinal - current_ref ) * ( int32_t )( pHandle->ScalingFactor );
    aux1 /= ( int32_t )( pHandle->RampRemainingStep );
    pHandle->IncDecAmount = aux1;
  }

  return retVal;
}

/**
  * @brief  REMNG_GetValue：读取并返回相电流、状态机状态、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int32_t REMNG_GetValue( RampExtMngr_Handle_t * pHandle )
{
  int32_t ret_val;
  ret_val = pHandle->Ext / ( int32_t )( pHandle->ScalingFactor );
  return ret_val;
}

/**
  * @brief  REMNG_RampCompleted：检查并判断斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool REMNG_RampCompleted( RampExtMngr_Handle_t * pHandle )
{
  bool retVal = false;
  if ( pHandle->RampRemainingStep == 0u )
  {
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  REMNG_StopRamp：停止或禁用斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void REMNG_StopRamp( RampExtMngr_Handle_t * pHandle )
{
  pHandle->RampRemainingStep = 0u;
  pHandle->IncDecAmount = 0;
}

/**
  * @brief  getScalingFactor：读取并返回相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  Target  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint32_t getScalingFactor( int32_t Target )
{
  uint8_t i;
  uint32_t TargetAbs;
  int32_t aux;

  if ( Target < 0 )
  {
    aux = -Target;
    TargetAbs = ( uint32_t )( aux );
  }
  else
  {
    TargetAbs = ( uint32_t )( Target );
  }
  for ( i = 1u; i < 32u; i++ )
  {
    uint32_t limit = ( ( uint32_t )( 1 ) << ( 31u - i ) );
    if ( TargetAbs >= limit )
    {
      break;
    }
  }
  return ( ( uint32_t )( 1u ) << ( i - 1u ) );
}

/**
 * @}
 */

/**
 * @}
 */

/************************ （C）版权所有，文件结束 ************************/
