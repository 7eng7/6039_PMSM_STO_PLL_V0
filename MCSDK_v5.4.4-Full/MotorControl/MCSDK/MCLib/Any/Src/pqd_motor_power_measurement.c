/** @file pqd_motor_power_measurement.c
 *  @brief 基于d-q量的有功功率估算。利用功率不变性计算与Vd*Id+Vq*Iq成比例的瞬时功率，再经
 *  换算和平均得到瓦特值；正负号可表示电动/回馈，精度取决于采样标定。 */
/**
  ******************************************************************************
  * @file    pqd_motor_power_measurement.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */

#include "pqd_motor_power_measurement.h"

#include "mc_type.h"


/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup motorpowermeasurement
  * @{
  */

/**
  * @defgroup pqd_motorpowermeasurement 功率、电机功能模块
  * @brief 功率、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  PQD_CalcElMotorPower：计算并更新缓冲区、功率、电机，保持对象状态和控制流程一致。
  * @param  power  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void PQD_CalcElMotorPower( PQD_MotorPowMeas_Handle_t * pHandle )
{

  int32_t wAux, wAux2, wAux3;
  qd_t Iqd = pHandle->pFOCVars->Iqd;
  qd_t Vqd = pHandle->pFOCVars->Vqd;
  wAux = ( ( int32_t )Iqd.q * ( int32_t )Vqd.q ) +
         ( ( int32_t )Iqd.d * ( int32_t )Vqd.d );
  wAux /= 65536;

  wAux2 = pHandle->wConvFact * ( int32_t )VBS_GetAvBusVoltage_V( pHandle->pVBS );
  wAux2 /= 600; /* 中文说明：处理并更新母线电压、电压。 */

  wAux3 = wAux * wAux2;
  wAux3 *= 6; /* 中文说明：处理并更新母线电压、电压。 */
  wAux3 /= 10;
  wAux3 /= 65536;

  MPM_CalcElMotorPower( &pHandle->_super, wAux3 );

}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ （C）版权所有，文件结束 ************************/
