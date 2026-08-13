/**
 * @file mc_tasks.c
 * @brief MCSDK 电机状态机和高/中/低频任务调度核心。
 *
 * 中文解读：本文件把启动命令转化为 IDLE、偏置校准、START、START_RUN、RUN、STOP、
 * FAULT_NOW、FAULT_OVER 等状态迁移。高频任务闭合电流环，中频任务运行速度环、
 * STO-PLL可靠性判断和开闭环切换，安全任务合并欠压、过压、过温、过流及算法故障。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

/* 头文件 */
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"
#include "r3_2_f4xx_pwm_curr_fdbk.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"

/* 私有宏定义 */
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)
/* 用户代码结束 */

#define VBUS_TEMP_ERR_MASK ( 0 | 0 | MC_OVER_TEMP)

/* 私有变量 */
FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t *oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;

NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /* 中文说明：启动或使能斜坡。 */
static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

/* 私有函数 */
void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);
void UI_Scheduler(void);

/**
  * @brief  MCboot：初始化电机、参数和增益，保持对象状态和控制流程一致。
  * @param  pMCIList  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMCTList  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void MCboot(MCI_Handle_t* pMCIList[NBR_OF_MOTORS], MCT_Handle_t* pMCTList[NBR_OF_MOTORS])
{
    /* 启动时只构造软件对象并保持PWM关闭：初始化采样驱动、传感器、PI、状态机和MCI/MCT句柄，
     * 最后置bMCBootCompleted。此函数不会自动越过IDLE，也不会忽略当前保护输入。 */
    /**************************************/
    /* 中文说明：初始化状态机状态。 */
    /**************************************/
    STM_Init(&STM[M1]);
    bMCBootCompleted = 0;
    pCLM[M1] = &CircleLimitationM1;

    /**********************************************************/
    /* 中文说明：初始化相电流、PWM和占空比。 */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);

    /******************************************************/
    /* 中文说明：初始化速度、PI/PID调节器。 */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);
    pPIDSpeed[M1] = &PIDSpeedHandle_M1;

    /******************************************************/
    /* 中文说明：初始化速度。 */
    /******************************************************/
    pSTC[M1] = &SpeednTorqCtrlM1;
    STO_PLL_Init(&STO_PLL_M1);

    /******************************************************/
    /* 中文说明：初始化速度、转矩。 */
    /******************************************************/
    STC_Init(pSTC[M1], pPIDSpeed[M1], &STO_PLL_M1._Super);

    /****************************************************/
    /* 中文说明：初始化速度。 */
    /****************************************************/
    VSS_Init(&VirtualSpeedSensorM1);

    /**************************************/
    /* 中文说明：初始化相关数据和控制状态。 */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /* 中文说明：初始化相电流、PI/PID调节器。 */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
    pPIDIq[M1] = &PIDIqHandle_M1;
    pPIDId[M1] = &PIDIdHandle_M1;

    /********************************************************/
    /* 中文说明：初始化母线电压、电压。 */
    /********************************************************/
    pBusSensorM1 = &RealBusVoltageSensorParamsM1;
    RVBS_Init(pBusSensorM1);

    /*************************************************/
    /* 中文说明：初始化功率。 */
    /*************************************************/
    pMPM[M1] = &PQD_MotorPowMeasM1;
    pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /* 中文说明：初始化温度。 */
    /*******************************************************/
    NTC_Init(&TempSensorParamsM1);                              
    pTemperatureSensor[M1] = &TempSensorParamsM1;

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
    oMCInterface[M1] = & Mci[M1];
    MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1]);
    MCI_ExecSpeedRamp(oMCInterface[M1],
                      STC_GetMecSpeedRefUnitDefault(pSTC[M1]), 0); /* 中文说明：处理并更新相关数据和控制状态。 */
    pMCIList[M1] = oMCInterface[M1];
    MCT[M1].pPIDSpeed = pPIDSpeed[M1];
    MCT[M1].pPIDIq = pPIDIq[M1];
    MCT[M1].pPIDId = pPIDId[M1];
    MCT[M1].pPIDFluxWeakening = MC_NULL; /* 中文说明：处理并更新相关数据和控制状态。 */
    MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
    MCT[M1].pRevupCtrl = &RevUpControlM1;              /* 中文说明：处理并更新相关数据和控制状态。 */
    MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &STO_PLL_M1;
    MCT[M1].pSpeedSensorAux = MC_NULL;
    MCT[M1].pSpeedSensorVirtual = &VirtualSpeedSensorM1;  /* 中文说明：处理并更新相关数据和控制状态。 */
    MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
    MCT[M1].pStateMachine = &STM[M1];
    MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
    MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
    MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* 中文说明：处理并更新相关数据和控制状态。 */
    MCT[M1].pNTCRelay = MC_NULL;             /* 中文说明：处理并更新相关数据和控制状态。 */
    MCT[M1].pMPM = (MotorPowMeas_Handle_t*)pMPM[M1];
    MCT[M1].pFW = MC_NULL;
    MCT[M1].pFF = MC_NULL;

    MCT[M1].pPosCtrl = MC_NULL;

    MCT[M1].pSCC = MC_NULL;
    MCT[M1].pOTT = MC_NULL;
    pMCTList[M1] = &MCT[M1];

    bMCBootCompleted = 1;
}

/**
  * @brief  MC_RunMotorControlTasks：设置速度、功率、中断、电机，保持对象状态和控制流程一致。
  */
__WEAK void MC_RunMotorControlTasks(void)
{
    /* 由系统节拍调用的软件任务入口：调度状态机/安全任务和UI。高频电流环不在这里，
     * 它由ADC同步中断直接调用TSK_HighFrequencyTask。 */
    if (bMCBootCompleted)
    {
        /* 中文说明：执行并推进相关数据和控制状态。 */
        MC_Scheduler();

        /* 中文说明：执行并推进相关数据和控制状态。 */
        TSK_SafetyTask();

        /* 中文说明：执行并推进相关数据和控制状态。 */
        UI_Scheduler();
    }
}

/**
  * @brief  MC_Scheduler：执行并推进相关数据和控制状态，保持对象状态和控制流程一致。
  */
__WEAK void MC_Scheduler(void)
{
    /* 多速率调度采用递减计数器：到零时执行Motor 1中频任务并重装计数值；
     * 自举电容等待和停机保持计数也在同一节拍递减，保证状态转换具有确定时间。 */

    if (bMCBootCompleted == 1)
    {
        if (hMFTaskCounterM1 > 0u)
        {
            hMFTaskCounterM1--;
        }
        else
        {
            TSK_MediumFrequencyTaskM1();
           
            hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
        }
        if (hBootCapDelayCounterM1 > 0u)
        {
            hBootCapDelayCounterM1--;
        }
        if (hStopPermanencyCounterM1 > 0u)
        {
            hStopPermanencyCounterM1--;
        }
    }
    else
    {
    }

}

/**
  * @brief  TSK_MediumFrequencyTaskM1：执行并推进速度、状态机状态、PI/PID调节器、电机，保持对象状态和控制流程一致。
  */
__WEAK void TSK_MediumFrequencyTaskM1(void)
{
    /* 中频状态机是启动/运行流程的主控：消费MCI命令，完成电流偏置校准、开环Rev-Up、
     * STO收敛检查、速度环执行、停机等待和故障状态处理。每个case只执行当前状态允许的动作。 */

    State_t StateM1;
    int16_t wAux = 0;

    (void) STO_PLL_CalcAvrgMecSpeedUnit(&STO_PLL_M1, &wAux);
    PQD_CalcElMotorPower(pMPM[M1]);

    StateM1 = STM_GetState(&STM[M1]);

    switch (StateM1)
    {
    case IDLE_START:
        RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(oMCInterface[M1]));
        R3_2_TurnOnLowSides(pwmcHandle[M1]);
        TSK_SetChargeBootCapDelayM1(CHARGE_BOOT_CAP_TICKS);
        STM_NextState(&STM[M1], CHARGE_BOOT_CAP);
        break;

    case CHARGE_BOOT_CAP:
        if (TSK_ChargeBootCapDelayHasElapsedM1())
        {
            PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);

            STM_NextState(&STM[M1], OFFSET_CALIB);
        }
        break;

    case OFFSET_CALIB:
        if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
        {
            STM_NextState(&STM[M1], CLEAR);
        }
        break;

    case CLEAR:
        /* 中文说明：处理并更新相关数据和控制状态。 */
        FOCVars[M1].bDriveInput = EXTERNAL;
        STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
        STO_PLL_Clear(&STO_PLL_M1);

        if (STM_NextState(&STM[M1], START) == true)
        {
            FOC_Clear(M1);

            R3_2_SwitchOnPWM(pwmcHandle[M1]);
        }
        break;

    case START:
    {

        /* 中文说明：处理并更新速度。 */
        int16_t hForcedMecSpeedUnit;
        qd_t IqdRef;
        bool ObserverConverged = false;

        /* 中文说明：执行并推进相关数据和控制状态。 */
        if (! RUC_Exec(&RevUpControlM1))
        {
            /* 中文说明：处理并更新相关数据和控制状态。 */
            STM_FaultProcessing(&STM[M1], MC_START_UP, 0);
        }
        else
        {
            /* 中文说明：计算并更新相电流、转矩、斜坡。 */
            IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
            IqdRef.d = FOCVars[M1].UserIdref;
            /* 中文说明：处理并更新相电流、PWM和占空比。 */
            FOCVars[M1].Iqdref = IqdRef;
        }

        (void) VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

        /* 中文说明：检查并判断观测器、反电势和PLL。 */
        if (RUC_FirstAccelerationStageReached(&RevUpControlM1) == true)
        {
            ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M1, hForcedMecSpeedUnit);
            (void) VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
        }

        if (ObserverConverged)
        {
            qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(&STO_PLL_M1._Super));

            /* 中文说明：启动或使能斜坡。 */
            REMNG_Init(pREMNG[M1]);
            REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
            REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);

            STM_NextState(&STM[M1], SWITCH_OVER);
        }
    }
    break;

    case SWITCH_OVER:
    {
        bool LoopClosed;
        int16_t hForcedMecSpeedUnit;

        if (! RUC_Exec(&RevUpControlM1))
        {
            /* 中文说明：处理并更新相关数据和控制状态。 */
            STM_FaultProcessing(&STM[M1], MC_START_UP, 0);
        }
        else
        {
            /* 中文说明：计算并更新速度、位置和角度。 */
            LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
            /* 中文说明：检查并判断斜坡。 */
            LoopClosed |= VSS_TransitionEnded(&VirtualSpeedSensorM1);

            /* 中文说明：处理并更新状态机状态。 */
            if (LoopClosed == true)
            {
#if ( PID_SPEED_INTEGRAL_INIT_DIV == 0 )
                PID_SetIntegralTerm(pPIDSpeed[M1], 0);
#else
                PID_SetIntegralTerm(pPIDSpeed[M1],
                                    (int32_t)(FOCVars[M1].Iqdref.q * PID_GetKIDivisor(pPIDSpeed[M1]) /
                                              PID_SPEED_INTEGRAL_INIT_DIV));
#endif

                STM_NextState(&STM[M1], START_RUN);
            }
        }
    }

    break;

    case START_RUN:
        /* 中文说明：处理并更新相关数据和控制状态。 */
        STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /* 中文说明：处理并更新观测器、反电势和PLL。 */
        {

            FOC_InitAdditionalMethods(M1);
            FOC_CalcCurrRef(M1);
            STM_NextState(&STM[M1], RUN);
        }
        STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]);   /* 中文说明：处理并更新相电流、速度。 */
        MCI_ExecBufferedCommands(oMCInterface[M1]);   /* 中文说明：处理并更新速度、斜坡。 */

        break;

    case RUN:

        MCI_ExecBufferedCommands(oMCInterface[M1]);
        FOC_CalcCurrRef(M1);

        break;

    case ANY_STOP:
        R3_2_SwitchOffPWM(pwmcHandle[M1]);
        FOC_Clear(M1);
        MPM_Clear((MotorPowMeas_Handle_t*) pMPM[M1]);
        TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);

        STM_NextState(&STM[M1], STOP);
        break;

    case STOP:
        if (TSK_StopPermanencyTimeHasElapsedM1())
        {
            STM_NextState(&STM[M1], STOP_IDLE);
        }
        break;

    case STOP_IDLE:
        STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);      /* 中文说明：处理并更新相关数据和控制状态。 */
        VSS_Clear(&VirtualSpeedSensorM1);   /* 中文说明：清除并复位速度。 */

        STM_NextState(&STM[M1], IDLE);
        break;

    default:
        break;
    }

}

/**
  * @brief  FOC_Clear：初始化相电流、母线电压、电压、速度、转矩、PI/PID调节器、电机，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void FOC_Clear(uint8_t bMotor)
{
    /* 清除一次运行遗留的PI积分、速度/角度估计、斜坡及电流参考，保证下一次启动从确定初值开始。 */
    /* 用户代码开始 */

    /* 用户代码结束 */
    ab_t NULL_ab = {(int16_t)0, (int16_t)0};
    qd_t NULL_qd = {(int16_t)0, (int16_t)0};
    alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

    FOCVars[bMotor].Iab = NULL_ab;
    FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
    FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
    FOCVars[bMotor].hTeref = (int16_t)0;
    FOCVars[bMotor].Vqd = NULL_qd;
    FOCVars[bMotor].Valphabeta = NULL_alphabeta;
    FOCVars[bMotor].hElAngle = (int16_t)0;

    PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
    PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

    STC_Clear(pSTC[bMotor]);

    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

}

/**
  * @brief  FOC_InitAdditionalMethods：初始化状态机状态、电机，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void FOC_InitAdditionalMethods(uint8_t bMotor)
{
    /* 用户代码开始 */

    /* 用户代码结束 */
}

/**
  * @brief  FOC_CalcCurrRef：计算并更新相电流、转矩、串口通信帧、电机、参数和增益，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void FOC_CalcCurrRef(uint8_t bMotor)
{
    /* 外环参考计算：速度模式由速度误差经PI得到Iq，转矩模式直接取得斜坡Iq；随后叠加可选
     * 前馈/弱磁结果。这里生成参考，实际Id/Iq跟踪在高频电流环完成。 */

    if (FOCVars[bMotor].bDriveInput == INTERNAL)
    {
        FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
        FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;
    }

}

/**
  * @brief  TSK_SetChargeBootCapDelayM1：初始化电机，保持对象状态和控制流程一致。
  * @param  hTickCount  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
    hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  TSK_ChargeBootCapDelayHasElapsedM1：初始化电机，保持对象状态和控制流程一致。
  * @param  none  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
    bool retVal = false;
    if (hBootCapDelayCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

/**
  * @brief  TSK_SetStopPermanencyTimeM1：设置状态机状态、电机，保持对象状态和控制流程一致。
  * @param  hTickCount  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
    hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  TSK_StopPermanencyTimeHasElapsedM1：检查并判断状态机状态、电机，保持对象状态和控制流程一致。
  * @param  none  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
    bool retVal = false;
    if (hStopPermanencyCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

#if defined (CCMRAM_ENABLED)
    #if defined (__ICCARM__)
        #pragma location = ".ccmram"
    #elif defined (__CC_ARM)
        __attribute__((section(".ccmram")))
    #endif
#endif
/**
  * @brief  TSK_HighFrequencyTask：执行并推进相电流、状态机状态、电机，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint8_t TSK_HighFrequencyTask(void)
{
    /* 每个PWM周期执行一次：采相电流 -> Clarke/Park -> Id/Iq PI -> 电压圆限幅 -> 反Park ->
     * SVPWM比较值，同时更新STO输入。返回电机号供调试DAC选择；执行必须在下一采样点前结束。 */
    uint8_t bMotorNbr = 0;
    uint16_t hFOCreturn;
    FLAG_B(1);
    uint16_t hState;  /* 中文说明：处理并更新相关数据和控制状态。 */
    Observer_Inputs_t STO_Inputs; /* 中文说明：处理并更新相关数据和控制状态。 */

    STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* 中文说明：处理并更新相关数据和控制状态。 */
    if (STM[M1].bState == SWITCH_OVER)
    {
        if (!REMNG_RampCompleted(pREMNG[M1]))
        {
            FOCVars[M1].Iqdref.q = REMNG_Calc(pREMNG[M1]);
        }
    }

    hFOCreturn = FOC_CurrControllerM1();

    if (hFOCreturn == MC_FOC_DURATION)
    {
        STM_FaultProcessing(&STM[M1], MC_FOC_DURATION, 0);
    }
    else
    {
        bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
        STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* 中文说明：处理并更新相关数据和控制状态。 */
        STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(pBusSensorM1->_Super)); /* 中文说明：处理并更新相关数据和控制状态。 */
        STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
        STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1);  /* 中文说明：处理并更新相关数据和控制状态。 */
        if (IsAccelerationStageReached == false)
        {
            STO_ResetPLL(&STO_PLL_M1);
        }
        hState = STM_GetState(&STM[M1]);
        if ((hState == START) || (hState == SWITCH_OVER) || (hState == START_RUN)) /* 中文说明：处理并更新相关数据和控制状态。 */
        {
            int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
            VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
        }

    }
    FLAG_B(0);
    return bMotorNbr;
}

#if defined (CCMRAM)
    #if defined (__ICCARM__)
        #pragma location = ".ccmram"
    #elif defined (__CC_ARM) || defined(__GNUC__)
        __attribute__((section(".ccmram")))
    #endif
#endif
/**
  * @brief  FOC_CurrControllerM1：计算并更新相电流、速度、PWM和占空比、故障与保护、PI/PID调节器、串口通信帧、电机，保持对象状态和控制流程一致。
  * @param  this  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
inline uint16_t FOC_CurrControllerM1(void)
{
    qd_t Iqd, Vqd;
    ab_t Iab;
    alphabeta_t Ialphabeta, Valphabeta;

    int16_t hElAngle;
    uint16_t hCodeError;
    SpeednPosFdbk_Handle_t *speedHandle;

    speedHandle = STC_GetSpeedSensor(pSTC[M1]);
    hElAngle = SPD_GetElAngle(speedHandle);
    hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * PARK_ANGLE_COMPENSATION_FACTOR;
    PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
    Ialphabeta = MCM_Clarke(Iab);
    Iqd = MCM_Park(Ialphabeta, hElAngle);
    Vqd.q = PI_Controller(pPIDIq[M1],
                          (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

    Vqd.d = PI_Controller(pPIDId[M1],
                          (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);

    Vqd = Circle_Limitation(pCLM[M1], Vqd);
    hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;
    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
    FOCVars[M1].Vqd = Vqd;
    FOCVars[M1].Iab = Iab;
    FOCVars[M1].Ialphabeta = Ialphabeta;
    FOCVars[M1].Iqd = Iqd;
    FOCVars[M1].Valphabeta = Valphabeta;
    FOCVars[M1].hElAngle = hElAngle;
    return (hCodeError);
}

/**
  * @brief  TSK_SafetyTask：执行并推进母线电压、电压、故障与保护、温度，保持对象状态和控制流程一致。
  */
__WEAK void TSK_SafetyTask(void)
{
    /* 低频安全入口逐电机调用具体安全策略。保护检测独立于应用启动/停止命令，任何时刻均有效。 */

    if (bMCBootCompleted == 1)
    {
        TSK_SafetyTask_PWMOFF(M1);
        /* 中文说明：执行并推进PWM和占空比、ADC采样。 */
        RCM_ExecUserConv();

    }
}

/**
  * @brief  TSK_SafetyTask_PWMOFF：执行并推进电压、PWM和占空比、电机，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
    /* 合并温度、母线电压、硬件过流和控制时序故障并送入STM_FaultProcessing。
     * FAULT_NOW/FAULT_OVER状态始终关闭PWM；故障条件消失不会自动重新启动。 */


    uint16_t CodeReturn = MC_NO_ERROR;
    uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};

    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(pTemperatureSensor[bMotor]); /* 中文说明：检查并判断故障与保护。 */
    CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[bMotor]);                    /* 中文说明：检查并判断电压、故障与保护。 */
    if (bMotor == M1)
    {
        CodeReturn |=  errMask[bMotor] & RVBS_CalcAvVbus(pBusSensorM1);
    }

    STM_FaultProcessing(&STM[bMotor], CodeReturn, ~~CodeReturn); /* 中文说明：处理并更新故障与保护。 */
    switch (STM_GetState(&STM[bMotor])) /* 中文说明：处理并更新PWM和占空比、故障与保护。 */
    {
    case FAULT_NOW:
        PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
        FOC_Clear(bMotor);
        MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);

        break;
    case FAULT_OVER:
        PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

        break;
    default:
        break;
    }

}

/**
  * @brief  当前接口：处理并更新电机，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK MCI_Handle_t *GetMCI(uint8_t bMotor)
{
    MCI_Handle_t *retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = oMCInterface[bMotor];
    }
    return retVal;
}

/**
  * @brief  TSK_HardwareFaultTask：处理中断并更新故障与保护、电机，保持对象状态和控制流程一致。
  * @param  bMotor  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK MCT_Handle_t *GetMCT(uint8_t bMotor)
{
    MCT_Handle_t *retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = &MCT[bMotor];
    }
    return retVal;
}

/**
  * @brief  TSK_HardwareFaultTask：执行并推进故障与保护、PI/PID调节器、电机，保持对象状态和控制流程一致。
  */
__WEAK void TSK_HardwareFaultTask(void)
{
    /* CPU HardFault等不可恢复异常的最后防线：对所有电机强制关PWM并登记MC_SW_ERROR。 */


    R3_2_SwitchOffPWM(pwmcHandle[M1]);
    STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);

}
/**
  * @brief  mc_lock_pins：处理并更新电机、数字输出，保持对象状态和控制流程一致。
  */
__WEAK void mc_lock_pins(void)
{

}
