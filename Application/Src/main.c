
/**
 * @file main.c
 * @brief 电机控制应用入口。
 *
 * 中文解读：本文件完成 ET6039 内核、GPIO、ADC、SRPWM、串口和电机控制中间件的初始化，
 * 随后进入前台循环。FOC 的电流环并不在 while(1) 中执行，而是由 PWM/ADC 中断以 16 kHz
 * 周期调用；前台循环只处理按键、目标转速斜坡和状态指示。MC_StartMotor1() 仅向状态机
 * 提交启动请求，真正的偏置校准、开环启动和闭环切换由 mc_tasks.c 完成。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

#include "main.h"
#include "motorcontrol.h"
/* 私有头文件 */
/* 用户代码开始 */
#include "key.h"
#include "mc_api.h"
/* 用户代码结束 */

/* 用户代码开始 */
#define PI      3.1415
/* 用户代码结束 */

#ifdef USE_FULL_ASSERT
/**
  * @brief  assert_failed：处理并更新故障与保护，保持对象状态和控制流程一致。
  * @param  file:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  line:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__attribute__((noreturn)) void assert_failed(uint8_t *file, uint32_t line)
{
    /* 中文说明：处理并更新参数和增益。 */
    printf("Wrong parameters value: file %s on line %d", file, line);
    UNUSED(file);
    UNUSED(line);

    /* 中文说明：处理并更新相关数据和控制状态。 */
    while (1)
    {
    }
}
#endif

/**
  * @brief  main：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
int main(void)
{
    /* hSetSpeed使用机械转速rpm保存，调用MCSDK前再换算为内部速度单位。 */
    uint8_t key = 0, i = 0;
    int16_t hSetSpeed = DEFAULT_TARGET_SPEED_RPM;
    int16_t hMecSpeedUnit = 0;
    int32_t actualSpeedRpm = 0;
    /* 使能指令缓存并刷新系统时钟变量；FOC高频代码会从I-Cache降低取指等待。 */
    SCB_EnableICache();
    SystemCoreClockUpdate();
    /* 将ET6039中断向量表安装到中断控制器，后续外设才能注册自己的IRQ入口。 */
    Interrupt_initVectorTable();

    /* 板级外设初始化顺序：指示/保护GPIO -> 同步ADC -> 六路PWM -> 通信 -> 按键。 */
    led_init();
    Flag_init();
    user_adc_init();
    user_srpwm_init();
    user_uart_init();
    sd_gpio_init();
    key_init();
    /* 连接MCSDK对象并初始化状态机；MX_NVIC_Init设置控制任务所需的中断优先级。 */
    MX_MotorControl_Init();
    MX_NVIC_Init();
    printf("\r\nET6039 PMSM speed monitor ready, baud=115200.\r\n");
    /* 这里只提交启动请求。若此时母线未上电，安全任务会产生欠压并拒绝实际启动。 */
    MC_StartMotor1();

    while (1)
    {
        /* 非连续模式消抖：一次完整按下只返回一次KEYx_PRES。 */
        key = key_scan(0);
            
        if(key == KEY1_PRES)
        {
        MC_StartMotor1();    /* 若状态允许则启动；已运行时重复请求不会重新初始化FOC。 */
        hSetSpeed += 300;
            /* 跳过+300 rpm，使跨过零速时直接到+600 rpm，避开STO低速不可观区。 */
            if(hSetSpeed == 300) hSetSpeed += 300;
        if(hSetSpeed >= 3000) hSetSpeed = 3000;
        if(hSetSpeed == 0) MC_StopMotor1();
        }
        
        else if(key == KEY2_PRES)
        {
        MC_StartMotor1();
        hSetSpeed -= 300;
            /* 同理跳过-300 rpm；当前逻辑通过改变目标符号请求反转。 */
            if(hSetSpeed == -300) hSetSpeed -=300;
        if(hSetSpeed <= -3000) hSetSpeed = -3000;
        if(hSetSpeed == 0) MC_StopMotor1();            
        }
         
        /* duration=0表示立即更新速度参考；速度PI和电流环仍会受各自输出限幅约束。 */
        MC_ProgramSpeedRampMotor1(hSetSpeed*SPEED_UNIT/_RPM,0);

        /*
         * MCSDK返回以SPEED_UNIT表示的平均机械转速。先扩展到32位再换算，避免乘法溢出。
         * 正值表示正转，负值表示反转；STO尚未收敛或电机停止时读数通常接近0。
         */
        hMecSpeedUnit = MC_GetMecSpeedAverageMotor1();
        actualSpeedRpm = ((int32_t)hMecSpeedUnit * (int32_t)_RPM) /
                         (int32_t)SPEED_UNIT;
        //printf("speed_rpm=%ld\r\n", (long)actualSpeedRpm);
        printf("%ld,%ld\r\n", (long)actualSpeedRpm,(long)hSetSpeed);

        /* 前台循环及转速串口输出约10 Hz；硬实时FOC由中断独立执行。 */
        DWT_delay_ms(100);
        LED0_TOGGLE();

    }

}

