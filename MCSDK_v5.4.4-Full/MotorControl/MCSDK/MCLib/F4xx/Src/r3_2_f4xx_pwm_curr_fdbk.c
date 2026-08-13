/** @file r3_2_f4xx_pwm_curr_fdbk.c
 *  @brief ET6039三电阻/双ADC同步采样和互补PWM底层。文件名沿用F4xx，实际已适配SRPWM/ADC。
 *  启动时平均零电流样本求偏置；运行时按扇区读取两相、减偏置并重构第三相，采样点避开PWM
 *  边沿。开关PWM和过流上报也在此完成，是算法与PD6010B之间的关键实时边界。 */
/**
  ******************************************************************************
  * @file    r3_2_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */
/* 头文件 */
#include "r3_2_f4xx_pwm_curr_fdbk.h"
#include "mc_type.h"
#include "pwm_common.h"
#include "parameters_conversion.h"

/**
  * @defgroup r3_2_f4xx_pwm_curr_fdbk 相电流、PWM和占空比功能模块
  * @brief 相电流、PWM和占空比相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 私有宏定义 */

#define PWM_OUT_CH_ALL   PWM_U_CH | PWM_V_CH | PWM_W_CH

/* 私有函数声明 */
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters(PWMC_Handle_t * pHdl, uint16_t hCCR4Reg);
void R3_2_HFCurrentsCalibrationAB(PWMC_Handle_t * pHdl, ab_t * pStator_Currents);
void R3_2_HFCurrentsCalibrationC(PWMC_Handle_t * pHdl, ab_t * pStator_Currents);
static void R3_2_RLGetPhaseCurrents(PWMC_Handle_t * pHdl, ab_t * pStator_Currents);
static void R3_2_RLTurnOnLowSides(PWMC_Handle_t * pHdl);
static void R3_2_RLSwitchOnPWM(PWMC_Handle_t * pHdl);
// 中文说明：初始化相电流。

/**
  * @brief  R3_2_Init：初始化相电流、PWM和占空比、ADC采样、数字输出，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_Init(PWMC_R3_2_Handle_t * pHandle)
{
  /* 绑定R3_2句柄的ET6039 ADC/SRPWM资源并建立安全初态；初始化后比较输出仍被强制关闭。 */
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;
    ADC_Type * ADCx_1 = pHandle->pParams_str->ADCx_1;
    ADC_Type * ADCx_2 = pHandle->pParams_str->ADCx_2;

    /* 使能PWM */
//    SRPWMCOM_enable(PWM_OUT_CH_ALL);

//    /* ADC通道使能 */
//    ADC_enable(ADCx_1, VTEMP_VIRTUAL_CH);
//    ADC_enable(ADCx_2, VBUS_VIRTUAL_CH);
//    //使能ADC中断
//    ADC_enableInterrupt(ADCx_1, VTEMP_VIRTUAL_CH, ADC_INT_TRIGGER_EOC);
//    /* 中断功能 */
//    Interrupt_enable(SARC_INTR0_IRQn);
    pHandle->_Super.DTTest = 0u;
}

/**
  * @brief  R3_2_CurrentReadingCalibration：读取并返回相电流、电压、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  pHdl  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void R3_2_CurrentReadingCalibration(PWMC_Handle_t * pHdl)
{
  /* 电机静止且PWM无有效电压时累计多次三相ADC结果，平均值作为运放中点/ADC零偏。
   * 后续每次读相电流都先减该偏置，因此校准期间电机必须无电流且采样链路稳定。 */
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    pHandle->PhaseAOffset = 0u;
    pHandle->PhaseBOffset = 0u;
    pHandle->PhaseCOffset = 0u;


    //关闭各通道PWM,同时关闭桥臂上管的输出开关，保证没有电流流过
    R3_2_SwitchOffPWM(&pHandle->_Super);       //暂停PWM模块工作

    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);  //关闭上/下管输出，避免校准阶段执行错误的FOC
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);

    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);

    /* 中文说明：清除并复位相关数据和控制状态。 */
    pHandle->PolarizationCounter = 0u;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    /* 中文说明：处理并更新ADC采样。 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationAB;         //指向偏移相电流获取函数地址，专门读取AB相电流
    pHandle->_Super.pFctSetADCSampPointSectX = &R3_2_SetADCSampPointCalibration;  //指向偏移电流读取专用的采样点设置函数地址

    pHandle->CalibSector = SECTOR_4;      //配置扇区4为校准用扇区，读取AB相偏移电流
    /* 中文说明：处理并更新ADC采样。 */
    pHandle->_Super.Sector = SECTOR_4;    //需要在扇区_4上强制进行第一次偏移转换

    R3_2_SwitchOnPWM(&pHandle->_Super);   //使能PWM模块，开ADC触发采样

    /* 中文说明：处理并更新ADC采样。 */
    waitForPolarizationEnd(SRPWMx,
                           &pHandle->_Super.SWerror,
                           pHandle->pParams_str->RepetitionCounter,
                           &pHandle->PolarizationCounter);

    R3_2_SwitchOffPWM(&pHandle->_Super);
    /* 中文说明：处理并更新相关数据和控制状态。 */
    /* 中文说明：清除并复位相关数据和控制状态。 */
    pHandle->PolarizationCounter = 0u;

    /* 中文说明：处理并更新ADC采样。 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationC;

    pHandle->CalibSector = SECTOR_1;      //配置扇区1为校准用扇区
    /* 中文说明：处理并更新ADC采样。 */
    pHandle->_Super.Sector = SECTOR_1;    //需要在扇区_1上强制进行第一次偏移转换

    R3_2_SwitchOnPWM(&pHandle->_Super);   //使能PWM输出，但由169行可知此时CH1-3并不不输出，使能定时器1中断服务函数，CH4在中断中使能，ADC开始采样

    /* 中文说明：处理并更新ADC采样。 */
    waitForPolarizationEnd(SRPWMx,
                           &pHandle->_Super.SWerror,
                           pHandle->pParams_str->RepetitionCounter,
                           &pHandle->PolarizationCounter);                   //等待AB相电流偏移值采集完毕

    R3_2_SwitchOffPWM(&pHandle->_Super);                                      //关闭PWM输出，关闭定时器1中断，关闭ADC采样

    //ET6039数据是右对齐格式，相对ST的ADC寄存器数据右移了三位，相当于将采集到的偏移电流值缩小8倍，前面校准时采集了16次，结果相当于左移一位，使偏移电流完成uint16格式左对齐,
    pHandle->PhaseAOffset >>= 0;
    pHandle->PhaseBOffset >>= 0;
    pHandle->PhaseCOffset >>= 0;

    /* 中文说明：处理并更新ADC采样。 */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_GetPhaseCurrents;            //切换回正常相电流获取以及ADC采样
    pHandle->_Super.pFctSetADCSampPointSectX = &R3_2_SetADCSampPointSectX;    //切换回正常ADC采样设置

    /* 中文说明：处理并更新PWM和占空比。 */
    /* 中文说明：停止或禁用相关数据和控制状态。 */

    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1)); //UP-DOWN计数模式下50%占空比
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));

    /* 中文说明：启动或使能相关数据和控制状态。 */
    pHandle->_Super.Sector = SECTOR_4;

    /* 中文说明：启动或使能相关数据和控制状态。 */
    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);  //打开上/下管输出，但实际PWM模块并未使能
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);

    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
}

/**
  * @brief  R3_2_GetPhaseCurrents：读取并返回相电流、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_GetPhaseCurrents(PWMC_Handle_t * pHdl, ab_t* pStator_Currents)
{
  /* 按当前SVPWM扇区选择有效的两个ADC结果，减偏置、按符号约定饱和到int16，并利用
   * Ia+Ib+Ic=0重构缺失相。函数输出a/b字段实际承载MCSDK所需的两相定子电流。 */
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;

    int32_t wAux;
    uint16_t hReg1;
    uint16_t hReg2;
    uint8_t bSector;

    /* 中文说明：停止或禁用ADC采样。 */
//    SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);    // 进ADC中断时已关闭ADC的触发源
//    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT);

    bSector = pHandle->_Super.Sector;                                     // 获取采样扇区
    hReg1 = (uint16_t)((*(pHandle->pParams_str->ADCDataReg1[bSector]) & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk) << 4);              // 在寄存器中取ADC采样值，左移四位，左对齐
    hReg2 = (uint16_t)((*(pHandle->pParams_str->ADCDataReg2[bSector]) & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk) << 4);

    switch (bSector)
    {
    case SECTOR_4:
    case SECTOR_5:
        /* 中文说明：处理并更新相电流。 */
        /* Ia = A相零偏 - ADC转换值 */
        //在扇区4、5只采集a、b相电流
        wAux = (int32_t)(pHandle->PhaseAOffset) - (int32_t)(hReg1);        //实际电流值等于偏移量-采集值
        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux < -INT16_MAX)
        {
            pStator_Currents->a = -INT16_MAX;
        }
        else  if (wAux > INT16_MAX)
        {
            pStator_Currents->a = INT16_MAX;
        }
        else
        {
            pStator_Currents->a = (int16_t)wAux;
        }

        /* Ib = B相零偏 - ADC转换值 */
        wAux = (int32_t)(pHandle->PhaseBOffset) - (int32_t)(hReg2);

        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux < -INT16_MAX)
        {
            pStator_Currents->b = -INT16_MAX;
        }
        else  if (wAux > INT16_MAX)
        {
            pStator_Currents->b = INT16_MAX;
        }
        else
        {
            pStator_Currents->b = (int16_t)wAux;
        }
        break;

    case SECTOR_6:
    case SECTOR_1:
        /* 中文说明：处理并更新相电流。 */
        /* Ib = B相零偏 - ADC转换值 */
        wAux = (int32_t)(pHandle->PhaseBOffset) - (int32_t)(hReg1);
        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux < -INT16_MAX)
        {
            pStator_Currents->b = -INT16_MAX;
        }
        else  if (wAux > INT16_MAX)
        {
            pStator_Currents->b = INT16_MAX;
        }
        else
        {
            pStator_Currents->b = (int16_t)wAux;
        }

        /* Ic = C相零偏 - ADC转换值 */
        /* Ia = -Ic -Ib */
        wAux = (int32_t)(pHandle->PhaseCOffset) - (int32_t)(hReg2);
        wAux = -wAux - (int32_t)pStator_Currents->b;

        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux > INT16_MAX)
        {
            pStator_Currents->a = INT16_MAX;
        }
        else  if (wAux < -INT16_MAX)
        {
            pStator_Currents->a = -INT16_MAX;
        }
        else
        {
            pStator_Currents->a = (int16_t)wAux;
        }
        break;

    case SECTOR_2:
    case SECTOR_3:
        /* 中文说明：处理并更新相电流。 */
        /* Ia = A相零偏 - ADC转换值 */
        wAux = (int32_t)(pHandle->PhaseAOffset) - (int32_t)(hReg1);
        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux < -INT16_MAX)
        {
            pStator_Currents->a = -INT16_MAX;
        }
        else  if (wAux > INT16_MAX)
        {
            pStator_Currents->a = INT16_MAX;
        }
        else
        {
            pStator_Currents->a = (int16_t)wAux;
        }

        /* Ic = C相零偏 - ADC转换值 */
        /* Ib = -Ic -Ia */
        wAux = (int32_t)(pHandle->PhaseCOffset) - (int32_t)(hReg2);
        wAux = -wAux - (int32_t)pStator_Currents->a;

        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (wAux > INT16_MAX)
        {
            pStator_Currents->b = INT16_MAX;
        }
        else  if (wAux < -INT16_MAX)
        {
            pStator_Currents->b = -INT16_MAX;
        }
        else
        {
            pStator_Currents->b = (int16_t)wAux;
        }
        break;

    default:
        break;
    }

    //当前三相电流值
    pHandle->_Super.Ia = pStator_Currents->a;
    pHandle->_Super.Ib = pStator_Currents->b;
    //基尔霍夫电流定律求另外一相
    pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;
}

/**
  * @brief  R3_2_HFCurrentsCalibrationAB：设置相电流、PWM和占空比、ADC采样，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */

__WEAK void R3_2_HFCurrentsCalibrationAB(PWMC_Handle_t * pHdl, ab_t * pStator_Currents)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
// 中文说明：处理并更新相关数据和控制状态。

    /* 中文说明：停止或禁用ADC采样。 */
//  SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);
    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT | SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT);

    if (pHandle->PolarizationCounter < NB_CONVERSIONS)
    {
        pHandle->PhaseAOffset += (*pHandle->pParams_str->ADCDataReg1[pHandle->CalibSector] & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk); //取12位ADC转换数值,ET6039是右
        pHandle->PhaseBOffset += (*pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector] & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk); //对齐格式
        
        pHandle->PolarizationCounter++;
    }

    /* 中文说明：处理并更新相电流。 */
    pStator_Currents->a = 0;
    pStator_Currents->b = 0;
}

/**
  * @brief  R3_2_HFCurrentsCalibrationC：设置相电流、PWM和占空比、ADC采样，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_HFCurrentsCalibrationC(PWMC_Handle_t * pHdl, ab_t * pStator_Currents)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
// 中文说明：处理并更新相关数据和控制状态。

    /* 中文说明：停止或禁用ADC采样。 */
//  SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);
    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT | SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT);

    if (pHandle->PolarizationCounter < NB_CONVERSIONS)
    {
        pHandle->PhaseCOffset += (*pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector] & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk);  //取12位ADC转换数值
        pHandle->PolarizationCounter++;
    }

    /* 中文说明：启动或使能相电流。 */
    pStator_Currents->a = 0;
    pStator_Currents->b = 0;
}

/**
  * @brief  R3_2_TurnOnLowSides：启动或使能相电流、电压、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_TurnOnLowSides(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
// 中文说明：处理并更新PWM和占空比。

    pHandle->_Super.TurnOnLowSidesAction = true;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 0); /* 初始化U相上桥臂PWM */
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, 0);
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, 0);

    /* 中文说明：启动或使能PWM和占空比。 */
    SRPWMCOM_enable(PWM_OUT_CH_ALL);      //已在初始化阶段使能，确保PWM各模块工作

    /* 中文说明：处理并更新相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(PWM_CLK_TRIGGER, SRPWM_IT_EVT);
    while (SRPWM_getInterruptStatus(PWM_CLK_TRIGGER, SRPWM_IT_EVT) == RESET)

//        if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)      //没有用到
//        {
//            GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, SET);
//            GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, SET);
//            GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, SET);
//        }
        return;
}

/**
  * @brief  R3_2_SwitchOnPWM：启动或使能相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_SwitchOnPWM(PWMC_Handle_t * pHdl)
{
  /* 在比较寄存器和触发时序就绪后解除三相强制关闭，使六路互补PWM真正送往PD6010B。 */
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    pHandle->_Super.TurnOnLowSidesAction = false;

    /* 中文说明：设置PWM和占空比。 */
    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1)); /* UVW相上桥臂PWM配置为50%占空比 */
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod - 5u)); /*在上下计数方式的PED点，就是在脉宽中点触发ADC采样*/

    /* 使能PWM模块工作但不控制实际端口输出 */

    SRPWMCOM_enable(PWM_OUT_CH_ALL);

    /* 使能ADC触发PWM模块*/
    SRPWMCOM_enable(PWM_ADC_TRIGGER_CH);

    /* 中文说明：处理并更新PWM和占空比。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);
    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == RESET)
    { }
//        if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)       //没有用到
//        {
//            GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, RESET);
//            GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
//            GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
//        }

// 中文说明：清除并复位相关数据和控制状态。
//  SRPWM_clearInterruptStatus(SRPWMx,SRPWM_IT_EVT);;
// 中文说明：启动或使能PWM和占空比、中断。
//  SRPWM_enableEvtInterrupt(SRPWMx, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT);
//  SRPWM_enableInterrupt(SRPWMx, SRPWM_IT_EVT);
//  Interrupt_enable(EPWM_EPWMCOM_INTR0_IRQn);


    return;
}

/**
  * @brief  R3_2_SwitchOffPWM：停止或禁用相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void R3_2_SwitchOffPWM(PWMC_Handle_t * pHdl)
{
  /* 立即把三相上下桥输出置于安全状态并停止有效ADC触发，用于正常停机和所有故障路径。 */
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
// 中文说明：处理并更新PWM和占空比。

    /* 中文说明：停止或禁用相关数据和控制状态。 */
//  Interrupt_disable(EPWM_EPWMCOM_INTR0_IRQn);
    SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);  //停止ADC触发，不再进入中断服务函数，不再执行FOC LOOP


    pHandle->_Super.TurnOnLowSidesAction = false;

    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 0); /* U相上桥臂PWM输出为0，上管关断 */
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, 0);
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, 0);
    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, (pHandle->Half_PWMPeriod  - 5));  // 选择在上下计数模式中点时刻作为触发ADC的同步信号, 减去三个CLK确保可靠触发

    /* 中文说明：处理并更新PWM和占空比。 */
    SRPWM_clearInterruptStatus(PWM_CLK_TRIGGER, SRPWM_IT_EVT);
    while (SRPWM_getInterruptStatus(PWM_CLK_TRIGGER, SRPWM_IT_EVT) == RESET)
    {}
// 中文说明：清除并复位相关数据和控制状态。
//  SRPWM_clearInterruptStatus(PWM_U,SRPWM_IT_EVT);

    /* 中文说明：停止或禁用PWM和占空比。 */
    SRPWMCOM_disable(PWM_OUT_CH_ALL);

//    if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
//    {
//        GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
//    }

    /* 中文说明：执行并推进PWM和占空比。 */
//  SRPWM_clearInterruptStatus(SRPWMx,SRPWM_IT_EVT);
//  while ( SRPWM_getInterruptStatus(SRPWMx,SRPWM_IT_EVT) == 0 )
//  {}
// 中文说明：清除并复位相关数据和控制状态。
//  SRPWM_clearInterruptStatus(SRPWMx,SRPWM_IT_EVT);

    return;
}

/**
  * @brief  R3_2_WriteTIMRegisters：检查并判断相电流、PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hCCR4Reg:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters(PWMC_Handle_t * pHdl, uint16_t hCCR4Reg)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;
    uint16_t hAux;

    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, pHandle->_Super.CntPhA);
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, pHandle->_Super.CntPhB);//CH1-3在使能预装载后
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, pHandle->_Super.CntPhC);//并不会立即改变，需要等待同步加载信号
    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, hCCR4Reg);

    /* 中文说明：处理并更新相关数据和控制状态。 */
    /* 中文说明：设置相关数据和控制状态。 */
    if (READ_BIT(SRPWM1->EPWM_SADC0_MSEL, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT) || READ_BIT(SRPWM1->EPWM_SADC0_MSEL, SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT))    //这里要求触发ADC的PWM_ADC_TRIGGER在结束ADC采样后关闭，如果检测到被打开，则说明计算时间超过PWM周期了
    {
        hAux = MC_FOC_DURATION;
    }
    else
    {
        hAux = MC_NO_ERROR;
    }
    if (pHandle->_Super.SWerror == 1u)
    {
        hAux = MC_FOC_DURATION;
        pHandle->_Super.SWerror = 0u;
    }
    return hAux;
}

/**
  * @brief  R3_2_SetADCSampPointCalibration：设置相电流、PWM和占空比、ADC采样、温度，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t R3_2_SetADCSampPointCalibration(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;

    /* 中文说明：设置故障与保护。 */
    pHandle->ADCTriggerEdge = SRPWM_OUTEVT_SYNC_CMPA_UP_EVT;
    pHandle->_Super.Sector = pHandle->CalibSector;

    return R3_2_WriteTIMRegisters(&pHandle->_Super, (uint32_t)(pHandle->Half_PWMPeriod - 5u));    //减去5CLK确保ADC采样能够触发
}

/**
  * @brief  R3_2_SetADCSampPointSectX：设置相电流、PWM和占空比、ADC采样，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t R3_2_SetADCSampPointSectX(PWMC_Handle_t * pHdl)
{
  /* 根据三相占空比最小间隔判断PWM中心附近是否仍有足够稳定窗口；若没有，就把采样点移到
   * 边沿前/后并选择相应触发沿，同时配置该扇区需要读取的两相通道。 */
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;

    uint16_t hCntSmp;
    uint16_t hDeltaDuty;
    register uint16_t lowDuty = pHdl->lowDuty;
    register uint16_t midDuty = pHdl->midDuty;

    /* 中文说明：检查并判断PWM和占空比。 */
    if ((uint16_t)(pHandle->Half_PWMPeriod - lowDuty) > pHandle->pParams_str->hTafter)
    {
        /* 中文说明：处理并更新相电流、PWM和占空比、ADC采样。 */

        /* 中文说明：处理并更新相电流、ADC采样。 */
        pHandle->_Super.Sector = SECTOR_4;      //尽量在扇区4采集AB相电流

        /* 中文说明：设置PWM和占空比。 */
        hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 5u;
    }
    else
    {
        /* 中文说明：设置PWM和占空比、ADC采样。 */


        /* 中文说明：处理并更新相关数据和控制状态。 */
        hDeltaDuty = (uint16_t)(lowDuty - midDuty);

        /* 中文说明：处理并更新相关数据和控制状态。 */
        if (hDeltaDuty > (uint16_t)(pHandle->Half_PWMPeriod - lowDuty) * 2u)
        {
            /* hTbefore = 2*Ts + Tc，其中Ts为ADC采样时间，Tc为ADC转换时间。 */
            hCntSmp = lowDuty - pHandle->pParams_str->hTbefore;
        }
        else
        {
            /* hTafter = DT + max(Trise, Tnoise)，即死区加上上升时间与噪声时间中的较大值。 */
            hCntSmp = lowDuty + pHandle->pParams_str->hTafter;

            if (hCntSmp >= pHandle->Half_PWMPeriod)
            {
                /* 中文说明：检查并判断PWM和占空比、ADC采样。 */
                pHandle->ADCTriggerEdge = SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT;
                hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
            }
        }
    }

    return R3_2_WriteTIMRegisters(&pHandle->_Super, hCntSmp);
}

/**
  * @brief  当前接口：处理中断并更新相电流、PWM和占空比、中断，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */

__WEAK void *R3_2_TIMx_UP_IRQHandler(PWMC_R3_2_Handle_t * pHandle)
{
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;
    ADC_Type * ADCx_1 = pHandle->pParams_str->ADCx_1;
    ADC_Type * ADCx_2 = pHandle->pParams_str->ADCx_2;
    uint32_t ADCInjFlags;
    FLAG_A(1);
    /* 中文说明：检查并判断相关数据和控制状态。 */

    //关闭ADC0的虚拟通道的外部触发
    // SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);  // 在ADC转换结束时刻，关闭ADC触发源
    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT | SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT);

    ADC_disable(ADCx_1, (CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W));  //关闭电流采样的虚拟通道，重新设置
    ADC_disable(ADCx_2, (CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W));

    /* 中文说明：设置相电流。 */
    //下次ADC将更换采集通道，
    //根据扇区不同采集不同的电流，只需采集两相电流就可以了
    ADC_remapSingleAC(ADCx_1, pHandle->pParams_str->ADCConfig_VC1[pHandle->_Super.Sector], pHandle->pParams_str->ADCConfig1[pHandle->_Super.Sector]);   //根据扇区选择虚拟通道对应的模拟通道
    ADC_remapSingleAC(ADCx_2, pHandle->pParams_str->ADCConfig_VC2[pHandle->_Super.Sector], pHandle->pParams_str->ADCConfig2[pHandle->_Super.Sector]);

    /* 中文说明：启动或使能ADC采样。 */
    ADC_enable(ADCx_1, pHandle->pParams_str->ADCConfig_VC1[pHandle->_Super.Sector]);  //使能重新配置后的虚拟通道
    ADC_enable(ADCx_2, pHandle->pParams_str->ADCConfig_VC2[pHandle->_Super.Sector]);

    SRPWM_setADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, pHandle->ADCTriggerEdge); //设置ADC触发源

    pHandle->ADCTriggerEdge = SRPWM_OUTEVT_SYNC_CMPA_UP_EVT; //恢复初始默认值为上升沿触发
//    SRPWMCOM_enable(PWM_ADC_TRIGGER_CH);  //已经在初始化中使能
    FLAG_A(0);
    return &(pHandle->_Super.Motor);

}

/**
  * @brief  当前接口：处理中断并更新相电流、PWM和占空比、中断，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void *R3_2_BRK_IRQHandler(PWMC_R3_2_Handle_t *pHandle)
{

    if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
    {
        GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, RESET);
        GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
        GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
    }
    pHandle->OverCurrentFlag = true;

    return &(pHandle->_Super.Motor);
}

/**
  * @brief  R3_2_IsOverCurrentOccurred：检查并判断相电流、PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint16_t R3_2_IsOverCurrentOccurred(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *) pHdl;
    uint16_t retVal = MC_NO_FAULTS;
    if (pHandle->OverCurrentFlag == true)
    {
        retVal = MC_BREAK_IN;
        pHandle->OverCurrentFlag = false;
    }
    return retVal;
}

/**
  * @brief  R3_2_RLDetectionModeEnable：设置相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDuty  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void R3_2_RLDetectionModeEnable(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    if (pHandle->_Super.RLDetectionMode == false)
    {
        /* 中文说明：处理并更新相关数据和控制状态。 */
        SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
        SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
        SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 0);

        /* 中文说明：处理并更新相关数据和控制状态。 */
        if ((pHandle->pParams_str->LowSideOutputs) == LS_PWM_TIMER)
        {
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);

        }
        else if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
        {
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
        }
        else
        {
        }
        /* 中文说明：处理并更新相关数据和控制状态。 */
        SRPWMCOM_disable(PWM_W_CH);
    }

    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_RLGetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_2_RLTurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_2_RLSwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = true;
}

/**
  * @brief  R3_2_RLDetectionModeDisable：检查并判断相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void R3_2_RLDetectionModeDisable(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    if (pHandle->_Super.RLDetectionMode == true)
    {
        /* 中文说明：处理并更新相关数据和控制状态。 */
        SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);

        if ((pHandle->pParams_str->LowSideOutputs) == LS_PWM_TIMER)
        {
            SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
        }
        else if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
        {
            SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
        }
        else
        {
        }

        SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

        /* 中文说明：处理并更新相关数据和控制状态。 */
        SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);

        if ((pHandle->pParams_str->LowSideOutputs) == LS_PWM_TIMER)
        {
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
        }
        else if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
        {
            SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
        }
        else
        {
        }

        SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

        /* 中文说明：处理并更新相关数据和控制状态。 */
        SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);

        if ((pHandle->pParams_str->LowSideOutputs) == LS_PWM_TIMER)
        {
            SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_NONE);
        }
        else if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
        {
            SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
        }
        else
        {
        }

        SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);

        pHandle->_Super.pFctGetPhaseCurrents = &R3_2_GetPhaseCurrents;
        pHandle->_Super.pFctTurnOnLowSides = &R3_2_TurnOnLowSides;
        pHandle->_Super.pFctSwitchOnPwm = &R3_2_SwitchOnPWM;
        pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

        pHandle->_Super.RLDetectionMode = false;
    }
}

/**
  * @brief  R3_2_RLDetectionModeSetDuty：设置相电流、PWM和占空比、故障与保护，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDuty:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
uint16_t R3_2_RLDetectionModeSetDuty(PWMC_Handle_t * pHdl, uint16_t hDuty)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    uint32_t val;
    uint16_t hAux;

    val = ((uint32_t)(pHandle->Half_PWMPeriod) * (uint32_t)(hDuty)) >> 16;
    pHandle->_Super.CntPhA = (uint16_t)(val);

    /* 中文说明：处理并更新PWM和占空比。 */
    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, (uint32_t)pHandle->_Super.CntPhA);

    /* 中文说明：设置相关数据和控制状态。 */
    pHdl->Sector = SECTOR_4;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    /* 中文说明：设置相关数据和控制状态。 */
    if (READ_BIT(SRPWMCOM->EPWM_EN, PWM_ADC_TRIGGER_CH))
    {
        hAux = MC_FOC_DURATION;
    }
    else
    {
        hAux = MC_NO_ERROR;
    }
    if (pHandle->_Super.SWerror == 1u)
    {
        hAux = MC_FOC_DURATION;
        pHandle->_Super.SWerror = 0u;
    }
    return hAux;
}

/**
  * @brief  R3_2_RLGetPhaseCurrents：读取并返回相电流、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void R3_2_RLGetPhaseCurrents(PWMC_Handle_t * pHdl, ab_t * pStator_Currents)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    int32_t wAux;

    /* 中文说明：停止或禁用ADC采样。 */
//  SRPWMCOM_disable(PWM_ADC_TRIGGER_CH);
    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT| SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT);

    wAux = (int32_t)(pHandle->PhaseBOffset) - (int32_t)((* pHandle->pParams_str->ADCDataReg2[pHandle->_Super.Sector] & ADC0_FIL_RESULT_FIL_FIL_RESULT_Msk) << 4);

    /* 中文说明：检查并判断相关数据和控制状态。 */
    if (wAux > -INT16_MAX)
    {
        if (wAux < INT16_MAX)
        {
        }
        else
        {
            wAux = INT16_MAX;
        }
    }
    else
    {
        wAux = -INT16_MAX;
    }

    pStator_Currents->a = (int16_t)wAux;
    pStator_Currents->b = (int16_t)wAux;
}

/**
  * @brief  R3_2_RLTurnOnLowSides：启动或使能相电流、电压、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  pHdl:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
static void R3_2_RLTurnOnLowSides(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;

    /* 中文说明：处理并更新相关数据和控制状态。 */
    SRPWM_setCounterCompareValue(SRPWMx, SRPWM_COUNTER_COMPARE_A, 0u);

    /* 中文说明：清除并复位相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    /* 中文说明：处理并更新相关数据和控制状态。 */
    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == 0)
    {}
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    /* 中文说明：启动或使能PWM和占空比。 */
    SRPWMCOM_enable(SRPWM_CHANNEL_ALL);

//    if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
//    {
//        GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, SET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
//    }
    return;
}


/**
  * @brief  R3_2_RLSwitchOnPWM：启动或使能相电流、PWM和占空比，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
static void R3_2_RLSwitchOnPWM(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;


    /* 中文说明：处理并更新PWM和占空比。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);
    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == 0)
    {}
    /* 中文说明：清除并复位相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 1u);
    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, (pHandle->Half_PWMPeriod) - 5u);

    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == 0)
    {}
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);


    /* 中文说明：启动或使能PWM和占空比。 */
// 中文说明：处理并更新相关数据和控制状态。
    SRPWMCOM_enable(SRPWM_CHANNEL_ALL);

//    if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
//    {
//        GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
//    }

    /* 中文说明：清除并复位相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    /* 中文说明：启动或使能中断。 */
    SRPWM_enableEvtInterrupt(SRPWMx, SRPWM_OUTEVT_SYNC_ZERO_EVT);
    SRPWM_enableInterrupt(SRPWMx, SRPWM_IT_EVT);

    return;
}

/**
  * @brief  RLTurnOnLowSidesAndStart：启动或使能ADC采样，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void RLTurnOnLowSidesAndStart(PWMC_Handle_t * pHdl)
{
    PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
    SRPWM_Type *SRPWMx = pHandle->pParams_str->SRPWMx;;
    ADC_Type * ADCx_1 = pHandle->pParams_str->ADCx_1;
    ADC_Type * ADCx_2 = pHandle->pParams_str->ADCx_2;

    /* 中文说明：清除并复位相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == 0)
    {}
    /* 中文说明：清除并复位相关数据和控制状态。 */
    SRPWM_clearInterruptStatus(SRPWMx, SRPWM_IT_EVT);

    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 0x0u);
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, 0x0u);
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, 0x0u);

    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, (pHandle->Half_PWMPeriod - 5u));

    while (SRPWM_getInterruptStatus(SRPWMx, SRPWM_IT_EVT) == 0)
    {}

    /* 中文说明：启动或使能PWM和占空比。 */
    SRPWMCOM_enable(SRPWM_CHANNEL_ALL);

//    if ((pHandle->pParams_str->LowSideOutputs) == ES_GPIO)
//    {
// 中文说明：处理并更新相关数据和控制状态。
//        GPIO_writePin(pHandle->pParams_str->pwm_en_u_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_v_pin, RESET);
//        GPIO_writePin(pHandle->pParams_str->pwm_en_w_pin, RESET);
//    }

// 中文说明：处理并更新相关数据和控制状态。
//  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
//  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
//  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
//  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

    SRPWMCOM_enable(PWM_ADC_TRIGGER_CH);

    SRPWM_enableEvtInterrupt(SRPWMx, SRPWM_OUTEVT_SYNC_ZERO_EVT);
    SRPWM_enableInterrupt(SRPWMx, SRPWM_IT_EVT);

    return;
}


/**
  * @brief  RLSetADCSampPoint：设置ADC采样，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void RLSetADCSampPoint(PWMC_Handle_t * pHdl)
{
    /* 中文说明：读取并返回相关数据和控制状态。 */
    pHdl->Sector = SECTOR_4;

    return;
}


