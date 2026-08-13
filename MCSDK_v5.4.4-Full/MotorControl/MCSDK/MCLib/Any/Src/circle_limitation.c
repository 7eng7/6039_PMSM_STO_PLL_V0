/** @file circle_limitation.c
 *  @brief d-q电压矢量圆限幅。若sqrt(Vd²+Vq²)超过逆变器可实现半径，就按比例缩短矢量并保持
 *  方向，避免SVPWM占空比越界；查表和定点运算用于满足高频电流环的执行时间。 */
/**
  ******************************************************************************
  * @file    circle_limitation.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "circle_limitation.h"
#include "mc_math.h"
#include "mc_type.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup CircleLimitation 电机功能模块
  * @brief 电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

#if defined (CIRCLE_LIMITATION_VD)
__WEAK qd_t Circle_Limitation(CircleLimitation_Handle_t * pHandle, qd_t Vqd)
{
  int32_t MaxModule;
  int32_t square_q;
  int32_t square_temp;
  int32_t square_d;
  int32_t square_sum;
  int32_t square_limit;
  int32_t vd_square_limit;
  int32_t new_q;
  int32_t new_d;
  qd_t Local_Vqd=Vqd;

  MaxModule = pHandle->MaxModule;

  square_q = (int32_t)(Vqd.q) * Vqd.q;
  square_d = (int32_t)(Vqd.d) * Vqd.d;
  square_limit = MaxModule * MaxModule;
  vd_square_limit = pHandle->MaxVd * pHandle->MaxVd;
  square_sum = square_q + square_d;

  if (square_sum > square_limit)
  {
    if(square_d <= vd_square_limit)
    {
      square_temp = square_limit - square_d;
      new_q = MCM_Sqrt(square_temp);
      if(Vqd.q < 0)
      {
        new_q = -new_q;
      }
      new_d = Vqd.d;
    }
    else
    {
      new_d = pHandle->MaxVd;
      if(Vqd.d < 0)
      {
        new_d = -new_d;
      }

      square_temp = square_limit - vd_square_limit;
      new_q = MCM_Sqrt(square_temp);
      if(Vqd.q < 0)
      {
        new_q = - new_q;
      }
    }
    Local_Vqd.q = new_q;
    Local_Vqd.d = new_d;
  }
  return(Local_Vqd);
}
#else
/**
  * @brief  Circle_Limitation：检查并判断电压、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Vqd  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK qd_t Circle_Limitation( CircleLimitation_Handle_t * pHandle, qd_t Vqd )
{
  uint16_t table_element;
  uint32_t uw_temp;
  int32_t  sw_temp;
  qd_t local_vqd = Vqd;

  sw_temp = ( int32_t )( Vqd.q ) * Vqd.q +
            ( int32_t )( Vqd.d ) * Vqd.d;

  uw_temp = ( uint32_t ) sw_temp;

  /* 中文说明：处理并更新相关数据和控制状态。 */
  if ( uw_temp > ( uint32_t )( pHandle->MaxModule ) * pHandle->MaxModule )
  {

    uw_temp /= ( uint32_t )( 16777216 );

    /* 中文说明：处理并更新相关数据和控制状态。 */
    uw_temp -= pHandle->Start_index;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    table_element = pHandle->Circle_limit_table[( uint8_t )uw_temp];

    sw_temp = Vqd.q * ( int32_t )table_element;
    local_vqd.q = ( int16_t )( sw_temp / 32768 );

    sw_temp = Vqd.d * ( int32_t )( table_element );
    local_vqd.d = ( int16_t )( sw_temp / 32768 );
  }

  return ( local_vqd );
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ （C）版权所有，文件结束 ************************/

