/** @file sto_pll_speed_pos_fdbk.c
 *  @brief STO-PLL无感估速/估角。观测器用PMSM的R、L模型以及alpha-beta电压、电流估计反电势；
 *  PLL使反电势正交误差趋零，其积分量给出电角速度和角度。低速反电势弱，故速度方差、
 *  反电势一致性和连续可靠次数共同决定是否允许从虚拟角度切入闭环。 */
/**
  ******************************************************************************
  * @file    sto_pll_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 头文件 */
#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"



/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup SpeednPosFdbk
  * @{
  */

/**
  * @defgroup SpeednPosFdbk_STO 速度、位置和角度、状态机状态、观测器、反电势和PLL、电机功能模块
  * @brief 速度、位置和角度、状态机状态、观测器、反电势和PLL、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 私有宏定义 */
#define C6_COMP_CONST1  (int32_t) 1043038
#define C6_COMP_CONST2  (int32_t) 10430

/* 私有函数声明 */
static void STO_Store_Rotor_Speed( STO_PLL_Handle_t * pHandle, int16_t hRotor_Speed );
static int16_t STO_ExecutePLL( STO_PLL_Handle_t * pHandle, int16_t hBemf_alfa_est,
                               int16_t hBemf_beta_est );
static void STO_InitSpeedBuffer( STO_PLL_Handle_t * pHandle );


/**
  * @brief  STO_PLL_Init：初始化相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_Init( STO_PLL_Handle_t * pHandle )
{
  /* 初始化速度传感器基类、清空观测器/PLL状态并绑定PLL的PI调节器；不运行任何估算。 */
  int16_t htempk;
  int32_t wAux;


  pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
  pHandle->EnableDualCheck = true;

  wAux = ( int32_t )1;
  pHandle->F3POW2 = 0u;

  htempk = ( int16_t )( C6_COMP_CONST1 / ( pHandle->hF2 ) );

  while ( htempk != 0 )
  {
    htempk /= ( int16_t )2;
    wAux *= ( int32_t )2;
    pHandle->F3POW2++;
  }

  pHandle->hF3 = ( int16_t )wAux;
  wAux = ( int32_t )( pHandle->hF2 ) * pHandle->hF3;
  pHandle->hC6 = ( int16_t )( wAux / C6_COMP_CONST2 );

  STO_PLL_Clear( pHandle );

  PID_HandleInit( & pHandle->PIRegulator );

  /* 中文说明：清除并复位斜坡、观测器、反电势和PLL。 */
  pHandle->_Super.hMecAccelUnitP = 0;

  return;
}

/**
  * @brief  STO_PLL_Return：清除并复位相电流、观测器、反电势和PLL、中断，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  uint8_t  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */

__WEAK void STO_PLL_Return( STO_PLL_Handle_t * pHandle, uint8_t flag )
{
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  STO_PLL_CalcElAngle：计算并更新相电流、速度、位置和角度、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pInputVars_str  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int16_t STO_PLL_CalcElAngle( STO_PLL_Handle_t * pHandle, Observer_Inputs_t * pInputs )
{
  /* 高频观测器一步：由上周期估算电流、当前实测电流和施加电压更新alpha/beta反电势状态；
   * 再将反电势送入PLL求电角速度，并积分得到本周期电角度。所有量均按预计算系数定点缩放。 */
  int32_t wAux, wDirection;
  int32_t wIalfa_est_Next, wIbeta_est_Next;
  int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
  int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed,
          hValfa, hVbeta;


  if ( pHandle->wBemf_alfa_est > ( int32_t )( pHandle->hF2 )*INT16_MAX )
  {
    pHandle->wBemf_alfa_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_alfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_alfa_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est / pHandle->hF2 );
#else
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est >> pHandle->F2LOG );
#endif

  if ( pHandle->wBemf_beta_est > INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_beta_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est / pHandle->hF2 );
#else
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est >> pHandle->F2LOG );
#endif

  if ( pHandle->Ialfa_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ialfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }

  if ( pHandle->Ibeta_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ibeta_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

  hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifdef FULL_MISRA_C_COMPLIANCY
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.alpha;
#ifdef FULL_MISRA_C_COMPLIANCY
  hValfa = ( int16_t ) ( wAux / 65536 );
#else
  hValfa = ( int16_t ) ( wAux >> 16 );
#endif

  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.beta;
#ifdef FULL_MISRA_C_COMPLIANCY
  hVbeta = ( int16_t ) ( wAux / 65536 );
#else
  hVbeta = ( int16_t ) ( wAux >> 16 );
#endif

  /* 中文说明：处理并更新观测器、反电势和PLL。 */
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

  wAux = ( int32_t ) ( pHandle->hC1 ) * hAux;
  wIalfa_est_Next = pHandle->Ialfa_est - wAux;

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIalfa_err;
  wIalfa_est_Next += wAux;

  wAux = ( int32_t ) ( pHandle->hC5 ) * hValfa;
  wIalfa_est_Next += wAux;

  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Alfa;
  wIalfa_est_Next -= wAux;

  wAux = ( int32_t )( pHandle->hC4 ) * hIalfa_err;
  wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t ) hAux_Beta / pHandle->hF3;
#else
  wAux = ( int32_t ) hAux_Beta >> pHandle->F3POW2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = pHandle->_Super.hElSpeedDpp * wAux;
  wBemf_alfa_est_Next += wAux;

  /* 中文说明：处理并更新观测器、反电势和PLL。 */
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  wAux = ( int32_t )  ( pHandle->hC1 ) * hAux;
  wIbeta_est_Next = pHandle->Ibeta_est - wAux;

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIbeta_err;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t ) ( pHandle->hC5 ) * hVbeta;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Beta;
  wIbeta_est_Next -= wAux;

  wAux = ( int32_t )( pHandle->hC4 ) * hIbeta_err;
  wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t )hAux_Alfa / pHandle->hF3;
#else
  wAux = ( int32_t ) hAux_Alfa >> pHandle->F3POW2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = pHandle->_Super.hElSpeedDpp * wAux;
  wBemf_beta_est_Next -= wAux;

  if(pHandle->hForcedAvrSpeed_VSS >=0)
  {
    wDirection = 1;
  }
  else
  {
    wDirection = -1;
  }

  /* 中文说明：处理并更新观测器、反电势和PLL。 */
  pHandle->hBemf_alfa_est = hAux_Alfa;
  pHandle->hBemf_beta_est = hAux_Beta;

  hAux_Alfa = ( int16_t )( hAux_Alfa * wDirection );
  hAux_Beta = ( int16_t )( hAux_Beta * wDirection );

  hRotor_Speed = STO_ExecutePLL( pHandle, hAux_Alfa, -hAux_Beta );
  pHandle->_Super.InstantaneousElSpeedDpp = hRotor_Speed;

  STO_Store_Rotor_Speed( pHandle, hRotor_Speed );

  pHandle->_Super.hElAngle += hRotor_Speed;

  /* 中文说明：处理并更新相电流、观测器、反电势和PLL。 */
  pHandle->Ialfa_est = wIalfa_est_Next;
  pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

  pHandle->Ibeta_est = wIbeta_est_Next;
  pHandle->wBemf_beta_est = wBemf_beta_est_Next;

  return ( pHandle->_Super.hElAngle );
}

/**
  * @brief  STO_PLL_CalcAvrgMecSpeedUnit：读取并返回相电流、速度、状态机状态、缓冲区、观测器、反电势和PLL、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */

__WEAK bool STO_PLL_CalcAvrgMecSpeedUnit( STO_PLL_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
  /* 中频可靠性计算：对电速度缓冲区求平均和方差，换算机械速度，并组合速度范围、方差以及
   * 反电势幅值一致性判断。返回false会被上层视为无感速度反馈失效。 */
  int32_t wAvrSpeed_dpp = ( int32_t )0;
  int32_t wError, wAux, wAvrSquareSpeed, wAvrQuadraticError = 0;
  uint8_t i, bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;
  int32_t wObsBemf, wEstBemf;
  int32_t wObsBemfSq = 0, wEstBemfSq = 0;
  int32_t wEstBemfSqLo;

  bool bIs_Speed_Reliable = false, bAux = false;
  bool bIs_Bemf_Consistent = false;

  for ( i = 0u; i < bSpeedBufferSizeUnit; i++ )
  {
    wAvrSpeed_dpp += ( int32_t )( pHandle->Speed_Buffer[i] );
  }

  wAvrSpeed_dpp = wAvrSpeed_dpp / ( int16_t )bSpeedBufferSizeUnit;

  for ( i = 0u; i < bSpeedBufferSizeUnit; i++ )
  {
    wError = ( int32_t )( pHandle->Speed_Buffer[i] ) - wAvrSpeed_dpp;
    wError = ( wError * wError );
    wAvrQuadraticError += wError;
  }

  /* 中文说明：处理并更新相关数据和控制状态。 */
  wAvrQuadraticError = wAvrQuadraticError / ( int16_t )bSpeedBufferSizeUnit;

  /* 中文说明：处理并更新速度。 */
  wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
  wAvrSquareSpeed = ( wAvrSquareSpeed * ( int32_t )( pHandle->VariancePercentage )) / ( int16_t )128;

  if ( wAvrQuadraticError < wAvrSquareSpeed )
  {
    bIs_Speed_Reliable = true;
  }

  /* 中文说明：处理并更新速度。 */
  wAux = wAvrSpeed_dpp * ( int32_t )( pHandle->_Super.hMeasurementFrequency );
  wAux = wAux * ( int32_t ) ( pHandle->_Super.SpeedUnit );
  wAux = wAux / ( int32_t )( pHandle->_Super.DPPConvFactor);
  wAux = wAux / ( int16_t )( pHandle->_Super.bElToMecRatio );

  *pMecSpeedUnit = ( int16_t )wAux;
  pHandle->_Super.hAvrMecSpeedUnit = ( int16_t )wAux;

  pHandle->IsSpeedReliable = bIs_Speed_Reliable;

  /* 中文说明：检查并判断观测器、反电势和PLL。 */
  if ( pHandle->EnableDualCheck == true ) /* 中文说明：处理并更新相关数据和控制状态。 */
  {
    wAux = ( wAux < 0 ? ( -wAux ) : ( wAux ) ); /* 中文说明：处理并更新相关数据和控制状态。 */
    if ( wAux < ( int32_t )( pHandle->MaxAppPositiveMecSpeedUnit ) )
    {
      /* 中文说明：处理并更新观测器、反电势和PLL。 */
      wObsBemf = ( int32_t )( pHandle->hBemf_alfa_est );
      wObsBemfSq = wObsBemf * wObsBemf;
      wObsBemf = ( int32_t )( pHandle->hBemf_beta_est );
      wObsBemfSq += wObsBemf * wObsBemf;

      /* 中文说明：处理并更新观测器、反电势和PLL。 */
      wEstBemf = ( wAux * 32767 ) / ( int16_t )( pHandle->_Super.hMaxReliableMecSpeedUnit );
      wEstBemfSq = ( wEstBemf * ( int32_t )( pHandle->BemfConsistencyGain ) ) / 64;
      wEstBemfSq *= wEstBemf;

      /* 中文说明：处理并更新相关数据和控制状态。 */
      wEstBemfSqLo = wEstBemfSq -
                     ( wEstBemfSq / 64 ) * ( int32_t )( pHandle->BemfConsistencyCheck );

      /* 中文说明：检查并判断相关数据和控制状态。 */
      if ( wObsBemfSq > wEstBemfSqLo )
      {
        bIs_Bemf_Consistent = true;
      }
    }

    pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
    pHandle->Obs_Bemf_Level = wObsBemfSq;
    pHandle->Est_Bemf_Level = wEstBemfSq;
  }
  else
  {
    bIs_Bemf_Consistent = true;
  }

  /* 中文说明：处理并更新相关数据和控制状态。 */
  if ( pHandle->IsAlgorithmConverged == false )
  {
    bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
  }
  else
  {
    if ( ( pHandle->IsSpeedReliable == false ) || ( bIs_Bemf_Consistent == false ) )
    {
      pHandle->ReliabilityCounter++;
      if ( pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys )
      {
        pHandle->ReliabilityCounter = 0u;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        bAux = false;
      }
      else
      {
        bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
      }
    }
    else
    {
      pHandle->ReliabilityCounter = 0u;
      bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
    }
  }
  return ( bAux );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  STO_PLL_CalcAvrgElSpeedDpp：计算并更新相电流、速度、缓冲区、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_CalcAvrgElSpeedDpp( STO_PLL_Handle_t * pHandle )
{
  /* 对高频PLL速度样本滑动平均，得到每控制周期电角增量DPP，供角度预测和机械速度换算。 */

  int16_t hIndexNew = ( int16_t )pHandle->Speed_Buffer_Index;
  int16_t hIndexOld;
  int16_t hIndexOldTemp;
  int32_t wSum = pHandle->DppBufferSum;
  int32_t wAvrSpeed_dpp;
  int16_t hSpeedBufferSizedpp = ( int16_t )( pHandle->SpeedBufferSizeDpp );
  int16_t hSpeedBufferSizeUnit = ( int16_t )( pHandle->SpeedBufferSizeUnit );
  int16_t hBufferSizeDiff;

  hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

  if ( hBufferSizeDiff == 0 )
  {
    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->SpeedBufferOldestEl;
  }
  else
  {
    hIndexOldTemp = hIndexNew + hBufferSizeDiff;

    if ( hIndexOldTemp >= hSpeedBufferSizeUnit )
    {
      hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
    }
    else
    {
      hIndexOld = hIndexOldTemp;
    }

    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->Speed_Buffer[hIndexOld];
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wAvrSpeed_dpp = wSum / hSpeedBufferSizeDpp;
#else
  wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizeDppLOG;
#endif

  pHandle->_Super.hElSpeedDpp = ( int16_t )wAvrSpeed_dpp;
  pHandle->DppBufferSum = wSum;
}

/**
  * @brief  STO_PLL_Clear：初始化状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_Clear( STO_PLL_Handle_t * pHandle )
{
  pHandle->Ialfa_est = ( int32_t )0;
  pHandle->Ibeta_est = ( int32_t )0;
  pHandle->wBemf_alfa_est = ( int32_t )0;
  pHandle->wBemf_beta_est = ( int32_t )0;
  pHandle->_Super.hElAngle = ( int16_t )0;
  pHandle->_Super.hElSpeedDpp = ( int16_t )0;
  pHandle->ConsistencyCounter = 0u;
  pHandle->ReliabilityCounter = 0u;
  pHandle->IsAlgorithmConverged = false;
  pHandle->IsBemfConsistent = false;
  pHandle->Obs_Bemf_Level = ( int32_t )0;
  pHandle->Est_Bemf_Level = ( int32_t )0;
  pHandle->DppBufferSum = ( int32_t )0;
  pHandle->ForceConvergency = false;
  pHandle->ForceConvergency2 = false;

  STO_InitSpeedBuffer( pHandle );
  PID_SetIntegralTerm( & pHandle->PIRegulator, ( int32_t )0 );
}

/**
  * @brief  STO_Store_Rotor_Speed：处理中断并更新相电流、速度、缓冲区、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
inline static void STO_Store_Rotor_Speed( STO_PLL_Handle_t * pHandle, int16_t hRotor_Speed )
{

  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if ( bBuffer_index == pHandle->SpeedBufferSizeUnit )
  {
    bBuffer_index = 0u;
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];

  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
  pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
  * @brief  STO_ExecutePLL：处理中断并更新相电流、位置和角度、观测器、反电势和PLL、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
inline static int16_t STO_ExecutePLL( STO_PLL_Handle_t * pHandle, int16_t hBemf_alfa_est, int16_t
                               hBemf_beta_est )
{
  int32_t wAlfa_Sin_tmp, wBeta_Cos_tmp;
  int16_t hOutput;
  Trig_Components Local_Components;
  int16_t hAux1, hAux2;

  Local_Components = MCM_Trig_Functions( pHandle->_Super.hElAngle );

  /* 中文说明：处理并更新观测器、反电势和PLL。 */
  wAlfa_Sin_tmp = ( int32_t )( hBemf_alfa_est ) * ( int32_t )Local_Components.hSin;
  wBeta_Cos_tmp = ( int32_t )( hBemf_beta_est ) * ( int32_t )Local_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux1 = ( int16_t )( wBeta_Cos_tmp / 32768 );
#else
  hAux1 = ( int16_t )( wBeta_Cos_tmp >> 15 );
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux2 = ( int16_t )( wAlfa_Sin_tmp / 32768 );
#else
  hAux2 = ( int16_t )( wAlfa_Sin_tmp >> 15 );
#endif

  /* 中文说明：初始化速度、缓冲区、PI/PID调节器。 */
  hOutput = PI_Controller( & pHandle->PIRegulator, ( int32_t )( hAux1 ) - hAux2 );

  return ( hOutput );
}

/**
  * @brief  STO_InitSpeedBuffer：初始化相电流、速度、缓冲区，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
static void STO_InitSpeedBuffer( STO_PLL_Handle_t * pHandle )
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->SpeedBufferSizeUnit;

  /* 中文说明：处理并更新速度、缓冲区。 */
  for ( b_i = 0u; b_i < bSpeedBufferSize; b_i++ )
  {
    pHandle->Speed_Buffer[b_i] = ( int16_t )0;
  }
  pHandle->Speed_Buffer_Index = 0u;
  pHandle->SpeedBufferOldestEl = ( int16_t )0;

  return;
}

/**
  * @brief  STO_PLL_IsObserverConverged：设置相电流、速度、位置和角度、状态机状态、斜坡、观测器、反电势和PLL、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hForcedMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STO_PLL_IsObserverConverged( STO_PLL_Handle_t * pHandle, int16_t hForcedMecSpeedUnit )
{
  /* 开环切闭环判据：估算速度和强制速度必须同向，差值落入配置百分比窗口，并连续满足指定
   * 次数；任一次失败会重置连续计数，避免一次偶然匹配就切换角度源。 */
  int16_t hEstimatedSpeedUnit, hUpperThreshold, hLowerThreshold;
  int32_t wAux;
  bool bAux = false;
  int32_t wtemp;

  
  pHandle->hForcedAvrSpeed_VSS = hForcedMecSpeedUnit;
  
  
  if ( pHandle->ForceConvergency2 == true )
  {
    hForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
  }

  if ( pHandle->ForceConvergency == true )
  {
    bAux = true;
    pHandle->IsAlgorithmConverged = true;
    pHandle->_Super.bSpeedErrorNumber = 0u;
  }
  else
  {
    hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;

    wtemp = ( int32_t )hEstimatedSpeedUnit * ( int32_t )hForcedMecSpeedUnit;

    if ( wtemp > 0 )
    {
      if ( hEstimatedSpeedUnit < 0 )
      {
        hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
      }

      if ( hForcedMecSpeedUnit < 0 )
      {
        hForcedMecSpeedUnit = -hForcedMecSpeedUnit;
      }
      wAux = ( int32_t ) ( hForcedMecSpeedUnit ) * ( int16_t )pHandle->SpeedValidationBand_H;
      hUpperThreshold = ( int16_t )( wAux / ( int32_t )16 );

      wAux = ( int32_t ) ( hForcedMecSpeedUnit ) * ( int16_t )pHandle->SpeedValidationBand_L;
      hLowerThreshold = ( int16_t )( wAux / ( int32_t )16 );

      /* 中文说明：处理并更新速度。 */
      if ( pHandle->IsSpeedReliable == true )
      {
        if ( ( uint16_t )hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed )
        {
          /* 中文说明：处理并更新相关数据和控制状态。 */
          if ( hEstimatedSpeedUnit >= hLowerThreshold )
          {
            if ( hEstimatedSpeedUnit <= hUpperThreshold )
            {
              pHandle->ConsistencyCounter++;

              /* 中文说明：处理并更新相关数据和控制状态。 */
              if ( pHandle->ConsistencyCounter >=
                   pHandle->StartUpConsistThreshold )
              {

                /* 中文说明：处理并更新相关数据和控制状态。 */
                bAux = true;
                pHandle->IsAlgorithmConverged = true;
                pHandle->_Super.bSpeedErrorNumber = 0u;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0u;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0u;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0u;
        }
      }
      else
      {
        pHandle->ConsistencyCounter = 0u;
      }
    }
  }

  return ( bAux );
}

/**
  * @brief  STO_PLL_GetEstimatedBemf：读取并返回相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK alphabeta_t STO_PLL_GetEstimatedBemf( STO_PLL_Handle_t * pHandle )
{
  alphabeta_t Vaux;
  Vaux.alpha = pHandle->hBemf_alfa_est;
  Vaux.beta = pHandle->hBemf_beta_est;
  return ( Vaux );
}


/**
  * @brief  STO_PLL_GetEstimatedCurrent：读取并返回相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK alphabeta_t STO_PLL_GetEstimatedCurrent( STO_PLL_Handle_t * pHandle )
{
  alphabeta_t Iaux;

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.alpha = ( int16_t )( pHandle->Ialfa_est / ( pHandle->hF1 ) );
#else
  Iaux.alpha = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.beta = ( int16_t )( pHandle->Ibeta_est / ( pHandle->hF1 ) );
#else
  Iaux.beta = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  return ( Iaux );
}

/**
  * @brief  STO_PLL_GetObserverGains：读取并返回相电流、观测器、反电势和PLL、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  phC2  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  phC4  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_GetObserverGains( STO_PLL_Handle_t * pHandle, int16_t * phC2, int16_t * phC4 )
{
  *phC2 = pHandle->hC2;
  *phC4 = pHandle->hC4;
}


/**
  * @brief  STO_PLL_SetObserverGains：设置相电流、观测器、反电势和PLL、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wK1  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wK2  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_SetObserverGains( STO_PLL_Handle_t * pHandle, int16_t hhC1, int16_t hhC2 )
{

  pHandle->hC2 = hhC1;
  pHandle->hC4 = hhC2;
}

/**
  * @brief  STO_GetPLLGains：读取并返回相电流、观测器、反电势和PLL、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pPgain  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pIgain  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_GetPLLGains( STO_PLL_Handle_t * pHandle, int16_t * pPgain, int16_t * pIgain )
{

  *pPgain = PID_GetKP( & pHandle->PIRegulator );
  *pIgain = PID_GetKI( & pHandle->PIRegulator );
}


/**
  * @brief  STO_SetPLLGains：设置相电流、观测器、反电势和PLL、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hPgain  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hIgain  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_SetPLLGains( STO_PLL_Handle_t * pHandle, int16_t hPgain, int16_t hIgain )
{
  PID_SetKP( & pHandle->PIRegulator, hPgain );
  PID_SetKI( & pHandle->PIRegulator, hIgain );
}


/**
  * @brief  STO_PLL_SetMecAngle：设置相电流、位置和角度、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hMecAngle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_PLL_SetMecAngle( STO_PLL_Handle_t * pHandle, int16_t hMecAngle )
{
}

/**
  * @brief  STO_OTF_ResetPLL：清除并复位相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_OTF_ResetPLL( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  PID_SetIntegralTerm( &pHdl->PIRegulator, ( int32_t )0 );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  STO_ResetPLL：清除并复位相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_ResetPLL( STO_PLL_Handle_t * pHandle )
{
  PID_SetIntegralTerm( &pHandle->PIRegulator, ( int32_t )0 );
}

/**
  * @brief  STO_SetPLL：设置相电流、速度、位置和角度、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hElSpeedDpp:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hElAngle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void STO_SetPLL( STO_PLL_Handle_t * pHandle, int16_t hElSpeedDpp, int16_t hElAngle )
{
  PID_SetIntegralTerm( & pHandle->PIRegulator,
                       ( int32_t )hElSpeedDpp * ( int32_t )PID_GetKIDivisor( & pHandle->PIRegulator ) );
  pHandle->_Super.hElAngle = hElAngle;
}

/**
  * @brief  STO_PLL_GetEstimatedBemfLevel：读取并返回相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int32_t STO_PLL_GetEstimatedBemfLevel( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->Est_Bemf_Level );
}

/**
  * @brief  STO_PLL_GetObservedBemfLevel：读取并返回相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK int32_t STO_PLL_GetObservedBemfLevel( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->Obs_Bemf_Level );
}

/**
  * @brief  STO_PLL_BemfConsistencyCheckSwitch：检查并判断相电流、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bSel  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void STO_PLL_BemfConsistencyCheckSwitch( STO_PLL_Handle_t * pHandle, bool bSel )
{
  pHandle->EnableDualCheck = bSel;
}

/**
  * @brief  STO_PLL_IsBemfConsistent：检查并判断相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STO_PLL_IsBemfConsistent( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->IsBemfConsistent );
}

/**
  * @brief  STO_PLL_IsVarianceTight：检查并判断相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool STO_PLL_IsVarianceTight( const STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  return ( pHdl->IsSpeedReliable );
}

/**
  * @brief  STO_PLL_ForceConvergency1：检查并判断相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void STO_PLL_ForceConvergency1( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency = true;
}

/**
  * @brief  STO_PLL_ForceConvergency2：检查并判断相电流、状态机状态、观测器、反电势和PLL，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void STO_PLL_ForceConvergency2( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency2 = true;
}

/**
  * @brief  STO_SetMinStartUpValidSpeedUnit：设置相电流、速度，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hMinStartUpValidSpeed:  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void STO_SetMinStartUpValidSpeedUnit( STO_PLL_Handle_t * pHandle, uint16_t hMinStartUpValidSpeed )
{
  pHandle->MinStartUpValidSpeed = hMinStartUpValidSpeed;
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ （C）版权所有，文件结束 ************************/
