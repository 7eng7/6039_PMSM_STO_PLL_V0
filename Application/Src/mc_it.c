/**
 * @file mc_it.c
 * @brief 电机控制实时中断入口。
 *
 * 中文解读：ADC/PWM中断构成 FOC 的硬实时调度器。高频中断读取相电流并执行坐标变换、
 * PI电流调节和SVPWM；PWM更新中断维护采样扇区和下一周期触发点。阻塞、printf或较长
 * 计算都不应放入这些中断，否则会产生 MC_FOC_DURATION 或占空比更新延迟。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

/* 头文件 */
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "et6039.h"
#include "et6x_srpwm.h"
#include "et6x_uart.h"


/* 私有宏定义 */
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)

void HardFault_Handler(void);
void SysTick_Handler(void);


/**
  * @brief  ADC0_IRQHandler：处理中断并更新ADC采样、中断，保持对象状态和控制流程一致。
  * @param  None  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void ADC0_IRQHandler(void)
{

    /* VTEMP虚拟通道的滤波结果中断被用作本工程高频FOC节拍。先清触发/中断标志，
     * 防止同一ADC事件重复进入，再运行一次完整电流环并更新可选调试DAC。 */
    if (ADC_getInterruptStatus(ADC0, VTEMP_VIRTUAL_CH, ADC_INT_TRIGGER_FILTER_RESULT))
    {
        SRPWM_clearADCTriggerSource(PWM_ADC_TRIGGER, SRPWM_SOC_A, SRPWM_OUTEVT_SYNC_CMPA_UP_EVT | SRPWM_OUTEVT_SYNC_CMPA_DOWN_EVT);
        ADC_clearInterruptStatus(ADC0, VTEMP_VIRTUAL_CH, ADC_INT_TRIGGER_FILTER_RESULT);

        UI_DACUpdate(TSK_HighFrequencyTask());  

    }
}

/**
  * @brief  PWM_U_IRQHandler：处理中断并更新PWM和占空比、中断、电机，保持对象状态和控制流程一致。
  * @param  None  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void PWM_U_IRQHandler(void)
{

    /* U相SRPWM零点/更新事件：为下一PWM周期准备扇区相关ADC采样配置。 */
    if (SRPWM_getInterruptStatus(PWM_U, SRPWM_IT_EVT) == SET)
    {
        SRPWM_clearInterruptStatus(PWM_U, SRPWM_IT_EVT);

        R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);    

    }

}

/* 中文说明：处理中断并更新串口通信帧、中断。 */
/**
  * @brief  USER_UART_IRQHandler：处理中断并更新串口通信帧、中断，保持对象状态和控制流程一致。
  * @param  None  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void USER_UART_IRQHandler(void)
{
    uint32_t temp_status = 0;
    temp_status = UART_getInterruptStatus(pUSART.USARTx);


    /* 收到一个字符后推进无阻塞帧解析状态机；返回1/2分别表示启动/停止帧超时计时。 */
    if (temp_status == UART_IT_STA_RECV_AVI)
    {
        uint16_t retVal;

        retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART, UART_readCharNonBlocking(pUSART.USARTx))); /* 中文说明：处理并更新相关数据和控制状态。 */
        if (retVal == 1)
        {
            UI_SerialCommunicationTimeOutStart();
        }
        if (retVal == 2)
        {
            UI_SerialCommunicationTimeOutStop();
        }
    }

    /* 字符超时意味着一帧中断或接收异常，复位协议接收状态，避免半帧污染下一帧。 */
    if (temp_status == UART_IT_STA_CHAR_TIMEOUT)
    {
        UFCP_OVR_IRQ_Handler(&pUSART);
        UI_SerialCommunicationTimeOutStop();
    }
}

/**
  * @brief  HardFault_Handler：处理中断并更新故障与保护，保持对象状态和控制流程一致。
  * @param  None  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void HardFault_Handler(void)
{

    /* CPU硬Fault时立即关闭电机PWM并把状态机置为硬件故障；此后不再尝试恢复电机。 */
    TSK_HardwareFaultTask();

    /* 中文说明：处理并更新故障与保护。 */
    while (1)
    {
        uint32_t temp_status = 0;
        temp_status = UART_getInterruptStatus(pUSART.USARTx);
        UART_clearInterruptStatus(UART0, UART_IT_STA_RECV_AVI | UART_IT_STA_THR | UART_IT_STA_CHAR_TIMEOUT);
        if ((temp_status & UART_IT_STA_CHAR_TIMEOUT) != 0) /* 中文说明：处理并更新故障与保护。 */
        {
            /* 中文说明：处理并更新相关数据和控制状态。 */
            UFCP_OVR_IRQ_Handler(&pUSART);
            UI_SerialCommunicationTimeOutStop();
        }

        if ((temp_status & UART_IT_STA_THR) != 0)
        {
            UFCP_TX_IRQ_Handler(&pUSART);
        }

        if ((temp_status & UART_IT_STA_RECV_AVI) != 0) /* 中文说明：处理并更新相关数据和控制状态。 */
        {
            uint16_t retVal;
            retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART, UART_readCharBlockingNonFIFO(pUSART.USARTx)));
            if (retVal == 1)
            {
                UI_SerialCommunicationTimeOutStart();
            }
            if (retVal == 2)
            {
                UI_SerialCommunicationTimeOutStop();
            }
        }
        else
        {
        }
    }

}

__IO uint32_t uwTick;
uint32_t uwTickPrio   = (1UL << __NVIC_PRIO_BITS); /* 中文说明：处理中断并更新相关数据和控制状态。 */
uint16_t uwTickFreq = 1U;  /* 中文说明：处理中断并更新相关数据和控制状态。 */

void SysTick_Handler(void)
{

    static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;

//    FLAG_A(1);

    /* 将系统SysTick分频为1 ms软件时基，供MCSDK中/低频任务、斜坡和通信超时使用。 */
    if (SystickDividerCounter == SYSTICK_DIVIDER)
    {
        uwTick += uwTickFreq;
        SystickDividerCounter = 0;
    }
    SystickDividerCounter ++;

    MC_RunMotorControlTasks();
    while (!UART_isSpaceAvailableNonFIFO(pUSART.USARTx))
    {
    }
    UFCP_TX_IRQ_Handler(&pUSART);

//    FLAG_A(0);

}
