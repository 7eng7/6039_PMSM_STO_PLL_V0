
/**
 * @file init_config.c
 * @brief ET6039 外设适配层：ADC、SRPWM、GPIO、串口和中断优先级配置。
 *
 * 中文解读：这是移植中最依赖硬件的文件。它把 J10 上的三相电流、母线电压和六路栅极
 * PWM 映射到 ET6039 外设，并建立“SRPWM 时基 -> ADC 同步采样 -> FOC 中断更新占空比”的
 * 实时链路。修改引脚、PWM频率、采样通道或触发沿时必须同步检查 mc_parameters.c 与
 * parameters_conversion.h，否则算法正确也会因采样相位或相序错误而失控。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
#include "init_config.h"
#include "key.h"
#include "parameters_conversion.h"
#include "et6x_dac.h"                   // 中文说明：初始化ADC采样、调试DAC。
#include "et6x_sysctrl.h"               // 中文说明：初始化ADC采样。

/**
  * @brief  user_adc_init：初始化ADC采样，保持对象状态和控制流程一致。
  * @param  None  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void user_adc_init(void)
{
    ADC_InitTypeDef stInit; 

    /* 校准ADC */
    ADC_startCalibration(ADC0);
    ADC_startCalibration(ADC1);
    ADC_startCalibration(ADC2);

    /* 初始化ADC */
    ADC_initStruct(&stInit);

    /* 配置ADC0/ADC2部分虚拟通道并行单次采样 */
    stInit.virtualChannelMask = CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W;
    stInit.triggerMode        = ADC_TRIGGER_MODE_ONE_SHOT;
    stInit.trigger            = ADC_TRIGGER_SRPWM_TRIGGER_NUM1;
    stInit.sampleTime         = 4;
    ADC_init(ADC0, &stInit);

    stInit.virtualChannelMask = CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W;
    stInit.triggerMode        = ADC_TRIGGER_MODE_ONE_SHOT;
    stInit.trigger            = ADC_TRIGGER_SRPWM_TRIGGER_NUM1;
    stInit.sampleTime         = 4;
    ADC_init(ADC2, &stInit);

    /* 配置ADC0/ADC1部分虚拟通道连续采样 */
    stInit.virtualChannelMask = VTEMP_VIRTUAL_CH;
    stInit.triggerMode        = ADC_TRIGGER_MODE_CONTINUOUS;
    stInit.trigger            = ADC_TRIGGER_SRPWM_TRIGGER_NUM1;
    stInit.sampleTime         = 4;
    ADC_init(ADC0, &stInit);

    stInit.virtualChannelMask = VBUS_VIRTUAL_CH;
    stInit.triggerMode        = ADC_TRIGGER_MODE_CONTINUOUS;
    stInit.trigger            = ADC_TRIGGER_SRPWM_TRIGGER_NUM0;
    stInit.sampleTime         = 4;
    ADC_init(ADC1, &stInit);

    /*将ADC0/1/2使用的虚拟通道映射到模拟通道*/
    ADC_remapSingleAC(ADC0, CURRENT_VIRTUAL_CHX_U, ADC_AMPU_CHX);
    ADC_remapSingleAC(ADC0, CURRENT_VIRTUAL_CHX_V, ADC_AMPV_CHX);
    ADC_remapSingleAC(ADC0, CURRENT_VIRTUAL_CHX_W, ADC_AMPW_CHX);
    ADC_remapSingleAC(ADC0, VTEMP_VIRTUAL_CH,    ADC_VTEMP_CHX);

    ADC_remapSingleAC(ADC1, VBUS_VIRTUAL_CH,     ADC_VBUS_CHX);

    ADC_remapSingleAC(ADC2, CURRENT_VIRTUAL_CHX_U, ADC_AMPU_CHX_C);
    ADC_remapSingleAC(ADC2, CURRENT_VIRTUAL_CHX_V, ADC_AMPV_CHX_C);
    ADC_remapSingleAC(ADC2, CURRENT_VIRTUAL_CHX_W, ADC_AMPW_CHX_C);
    
    /*将ADC2的C3模拟通道引出到输出管脚，对应ADC_AMPV_CHX_C*/
    SYSCTRL_selectAnalogPinMux(SYSCTL_PINMUX_A7_C3_MUX_C3);

    /* 选择虚拟通道冲突的处理方式 */
    ADC_setPriorityMode(ADC0, ADC_PRIORITY_MODE_LOW_QUEUE);
    ADC_setPriorityMode(ADC1, ADC_PRIORITY_MODE_LOW_QUEUE);
    ADC_setPriorityMode(ADC2, ADC_PRIORITY_MODE_LOW_QUEUE);

    /* 开启中断功能 */
    ADC_enableInterrupt(ADC0, VTEMP_VIRTUAL_CH, ADC_INT_TRIGGER_FILTER_RESULT);
    ADC_unmaskInterrupt(ADC0, VTEMP_VIRTUAL_CH, ADC_INT_TRIGGER_FILTER_RESULT);

    ADC_enable(ADC0, VTEMP_VIRTUAL_CH | CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W);
    ADC_enable(ADC1, VBUS_VIRTUAL_CH);
    ADC_enable(ADC2, CURRENT_VIRTUAL_CHX_U | CURRENT_VIRTUAL_CHX_V | CURRENT_VIRTUAL_CHX_W);

    /* 使能ADC中断 */
    Interrupt_register(SARC_INTR0_IRQn, ADC0_IRQHandler);


    /* 中断功能 */
    Interrupt_enable(SARC_INTR0_IRQn); 
}


void user_srpwm_init(void)
{
    SRPWM_TBInitTypeDef stTBInit;
    SRPWM_CMPInitTypeDef stCMPInit;
    SRPWM_DBInitTypeDef stDBInit;
    SRPWM_PSTPInitTypeDef stPSTPInit;

    IOC_Init_TypeDef Init;

    /* IO mux配置 */
    GPIO_initPadConfigStruct(&Init);
    Init.ds = IOC_DS_LEVEL_3;
    Init.pull = IOC_PULL_NONE;
    //上桥臂
    GPIO_setPadConfig(GPIO28, &Init);
    GPIO_setPadConfig(GPIO14, &Init);
    GPIO_setPadConfig(GPIO8,  &Init);
    GPIO_setPinConfig(GPIO28, GPIO28_MODE_SRPWM6_A);
    GPIO_setPinConfig(GPIO14, GPIO14_MODE_SRPWM7_A);
    GPIO_setPinConfig(GPIO8,  GPIO8_MODE_SRPWM4_A);
    // 下桥臂
    GPIO_setPadConfig(GPIO29, &Init);
    GPIO_setPadConfig(GPIO15, &Init);
    GPIO_setPadConfig(GPIO9,  &Init);
    GPIO_setPinConfig(GPIO29, GPIO29_MODE_SRPWM6_B);
    GPIO_setPinConfig(GPIO15, GPIO15_MODE_SRPWM7_B);
    GPIO_setPinConfig(GPIO9,  GPIO9_MODE_SRPWM4_B);

    /* TB初始化 */
    SRPWM_initTBStruct(&stTBInit);
    stTBInit.period    = ((PWM_PERIOD_CYCLES) / 2);           
    stTBInit.countMode = SRPWM_COUNTMODE_UPDOWN;              
    SRPWM_initTB(PWM_CLK_TRIGGER, &stTBInit);                  
    stTBInit.phase0        = 3;                               
    stTBInit.sync0CountDir = SRPWM_SYNC_DIR_UP;
    stTBInit.syncIn0  = SRPWM_TBCOUNTER_SYNCEVT_SYNC_IN0;     
    stTBInit.waitSync0TimerEn = ENABLE;
    SRPWM_initTB(PWM_U, &stTBInit);
    SRPWM_initTB(PWM_V, &stTBInit);
    SRPWM_initTB(PWM_W, &stTBInit);
    SRPWM_initTB(PWM_ADC_TRIGGER, &stTBInit);

    /* CMP初始化 */
    SRPWM_initCMPStruct(&stCMPInit);
    stCMPInit.cmpShadowLoadSel = SRPWM_SYNCEVTLOAD1_ZERO_EVT;   
    stCMPInit.cmpShadowLoadEn  = ENABLE;
    SRPWM_initCMP(PWM_U, &stCMPInit);
    SRPWM_initCMP(PWM_V, &stCMPInit);
    SRPWM_initCMP(PWM_W, &stCMPInit);
    SRPWM_initCMP(PWM_ADC_TRIGGER, &stCMPInit);
    SRPWM_initCMP(PWM_CLK_TRIGGER, &stCMPInit);


    /* PG初始化 */
    SRPWM_setPG(PWM_U, SRPWM_A, SRPWM_PG_EVT_ZERO, SRPWM_PG_SET);
    SRPWM_setPG(PWM_U, SRPWM_A, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_U, SRPWM_A, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_SET);
    SRPWM_setPG(PWM_U, SRPWM_B, SRPWM_PG_EVT_ZERO, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_U, SRPWM_B, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_SET);
    SRPWM_setPG(PWM_U, SRPWM_B, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_CLEAR);

    SRPWM_setPG(PWM_V, SRPWM_A, SRPWM_PG_EVT_ZERO, SRPWM_PG_SET);
    SRPWM_setPG(PWM_V, SRPWM_A, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_V, SRPWM_A, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_SET);
    SRPWM_setPG(PWM_V, SRPWM_B, SRPWM_PG_EVT_ZERO, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_V, SRPWM_B, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_SET);
    SRPWM_setPG(PWM_V, SRPWM_B, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_CLEAR);

    SRPWM_setPG(PWM_W, SRPWM_A, SRPWM_PG_EVT_ZERO, SRPWM_PG_SET);
    SRPWM_setPG(PWM_W, SRPWM_A, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_W, SRPWM_A, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_SET);
    SRPWM_setPG(PWM_W, SRPWM_B, SRPWM_PG_EVT_ZERO, SRPWM_PG_CLEAR);
    SRPWM_setPG(PWM_W, SRPWM_B, SRPWM_PG_EVT_CMPA_UP, SRPWM_PG_SET);
    SRPWM_setPG(PWM_W, SRPWM_B, SRPWM_PG_EVT_CMPA_DOWN, SRPWM_PG_CLEAR);

    /* 选择事件输出 */
    SRPWM_selectSyncOut(PWM_CLK_TRIGGER, SRPWM_ALLEVT_ZERO_EVT, SRPWM_SYNC0);
    SRPWMCOM_selectSyncToSRPWM(SRPWMCOM_SRPWM0_SYNCOUT0, SRPWM_SYNC0, PWM_U_CH);
    SRPWMCOM_selectSyncToSRPWM(SRPWMCOM_SRPWM0_SYNCOUT0, SRPWM_SYNC0, PWM_V_CH);
    SRPWMCOM_selectSyncToSRPWM(SRPWMCOM_SRPWM0_SYNCOUT0, SRPWM_SYNC0, PWM_W_CH);
    SRPWMCOM_selectSyncToSRPWM(SRPWMCOM_SRPWM0_SYNCOUT0, SRPWM_SYNC0, PWM_ADC_TRIGGER_CH);


    /* DB初始化 */
    SRPWM_initDBStruct(&stDBInit);
    stDBInit.a.risingDelay          = 100;           //设置桥臂上下管驱动死区时间 5ns*100 = 500ns, 300ns会有过流保护
    stDBInit.b.fallingDelay         = 100;
    stDBInit.a.selComplementChannel = DISABLE;
    stDBInit.a.selRisingDelay       = ENABLE;
    stDBInit.a.selFallingDelay      = DISABLE;
    stDBInit.a.selOutputState       = SRPWM_DB_OUTPUT_STATE_POS_OR_NEG_AFTER;
    stDBInit.b.selComplementChannel = ENABLE;
    stDBInit.b.selRisingDelay       = DISABLE;
    stDBInit.b.selFallingDelay      = ENABLE;
    stDBInit.b.selOutputState       = SRPWM_DB_OUTPUT_STATE_POS_OR_NEG_INVERT;

    SRPWM_initDB(PWM_U, &stDBInit);
    SRPWM_initDB(PWM_V, &stDBInit);
    SRPWM_initDB(PWM_W, &stDBInit);

    /* PSTP后级保护功能初始化 */
    // 防止上下桥臂同时为高
    SRPWM_initPSTPStruct(&stPSTPInit);
    stPSTPInit.mutexConditionAB = SRPWMCOM_A_HIGH_B_HIGH;
    stPSTPInit.mutexEn          = ENABLE;
    stPSTPInit.b.mutexOutState  = SRPWM_PSTP_MUTEX_OUTSTATE_SET;
    stPSTPInit.a.mutexOutState  = SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR;
    SRPWM_initPSTP(PWM_U, &stPSTPInit);
    SRPWM_initPSTP(PWM_V, &stPSTPInit);
    SRPWM_initPSTP(PWM_W, &stPSTPInit);

    //去除高精度；
    SRPWM_disableHRPWMBypass(PWM_U);
    SRPWM_disableHRPWMBypass(PWM_V);
    SRPWM_disableHRPWMBypass(PWM_W);
    SRPWM_disableHRPWMBypass(PWM_ADC_TRIGGER);
    SRPWM_disableHRPWMBypass(PWM_CLK_TRIGGER);

    // 初始化占空比为0
    SRPWM_setCounterCompareValue(PWM_U, SRPWM_COUNTER_COMPARE_A, 0); /* U相上桥臂PWM */
    SRPWM_setCounterCompareValue(PWM_V, SRPWM_COUNTER_COMPARE_A, 0);
    SRPWM_setCounterCompareValue(PWM_W, SRPWM_COUNTER_COMPARE_A, 0);

    SRPWM_setCounterCompareValue(PWM_ADC_TRIGGER, SRPWM_COUNTER_COMPARE_A, (stTBInit.period  - 5)); // 选择在上下计数模式中点时刻作为采样FOC电流的同步信号, 减去五个CLK确保可靠触发

    SRPWM_setCounterCompareValue(PWM_CLK_TRIGGER, SRPWM_COUNTER_COMPARE_A, (stTBInit.period  - 5)); 

    /* 选择事件输出 */
    SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT | SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT); 
    SRPWM_setADCTriggerSource(PWM_CLK_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT); 

    /* 选择给ADC的触发信号 */

    SRPWMCOM_configADCTriggerMux(SRPWMCOM_ADC_SRPWM1_SOC_A, SRPWM_ADC_TRIGGER_NUM1);       
    SRPWMCOM_configADCTriggerMux(SRPWMCOM_ADC_SRPWM0_SOC_A, SRPWM_ADC_TRIGGER_NUM0);         

    /* 中断功能 */
    Interrupt_register(EPWM_EPWMCOM_INTR6_IRQn, PWM_U_IRQHandler); 
    SRPWM_enableEvtInterrupt(PWM_U, SRPWM_OUTEVT_SYNC_ZERO_EVT);
    SRPWM_enableInterrupt(PWM_U, SRPWM_IT_EVT);
    Interrupt_enable(EPWM_EPWMCOM_INTR6_IRQn);    

    SRPWM_enableEvtInterrupt(PWM_CLK_TRIGGER, SRPWM_OUTEVT_SYNC_ZERO_EVT);   
    SRPWM_enableInterrupt(PWM_CLK_TRIGGER, SRPWM_IT_EVT);

    /* 使能指定模块 */
    SRPWMCOM_enable(PWM_CLK_TRIGGER_CH | PWM_ADC_TRIGGER_CH);  


    /* 初始化阶段先禁止端口SRPWM_A&B输出 */
    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_A, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);    
    
    
    SRPWMCOM_forcePSTPOutState(PWM_U_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_V_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);
    SRPWMCOM_forcePSTPOutState(PWM_W_CH, SRPWM_B, SRPWM_PSTP_MUTEX_OUTSTATE_CLEAR);

}


/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
/* 中文说明：处理并更新相关数据和控制状态。 */
int fputc(int ch, FILE *f)
{
    UNUSED(f);
    UART_writeCharBlockingNonFIFO(UART0, (uint8_t)ch);
    return ch;
}

/**
 * @brief       串口X初始化函数
 * @param       无
 * @retval      无
 */
void user_uart_init(void)
{
    IOC_Init_TypeDef Init;
    GPIO_initPadConfigStruct(&Init);

    // 中文说明：处理并更新串口通信帧。
    GPIO_setPadConfig(GPIO2, &Init);
    GPIO_setPadConfig(GPIO25, &Init);
    GPIO_setPinConfig(GPIO2, GPIO2_MODE_UART0_TX);
    GPIO_setPinConfig(GPIO25, GPIO25_MODE_UART0_RX);
    UART_performSoftwareReset(UART0);
    UART_setConfig(UART0, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UART_enableFIFO(UART0);
    // 中文说明：初始化中断。
    Interrupt_register(UART0_IRQn, USER_UART_IRQHandler);
    UART_enableInterrupt(UART0, UART_IT_RECV_DATA_AVAILABLE_FLAG);

    Interrupt_enable(UART0_IRQn);

}

/**
 * @brief       中断优先级配置函数
 * @param       无
 * @retval      无
 */
void MX_NVIC_Init(void)
{
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(EPWM_EPWMCOM_INTR6_IRQn, 0);     //PWM周期更新
    NVIC_SetPriority(SARC_INTR0_IRQn, 0);             //和电流采样和FOC计算为最高优先级
    NVIC_SetPriority(UART0_IRQn, 2);                  

}

void sd_gpio_init(void)
{
    IOC_Init_TypeDef init;
    /* 配置IOC */
    GPIO_initPadConfigStruct(&init);
    init.dir = IOC_DIR_OUT;
    init.pull = IOC_PULL_UP;
    init.ds = IOC_DS_LEVEL_3;
    GPIO_setPadConfig(GPIO27, &init);
    GPIO_setPinConfig(GPIO27, GPIO27_MODE_GPIO27);
    GPIO_setPortDirectionMode(GPIO27, GPIO_DIR_OUT);
    GPIO_writePin(GPIO27, SET);
}
