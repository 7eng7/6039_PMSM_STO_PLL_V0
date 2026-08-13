/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-14 12:08:09
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-08-13 01:29:55
 * @FilePath: \6039_PMSM_STO_PLL_V0\Application\Inc\init_config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
#ifndef __INIT_CONFIG_H
#define __INIT_CONFIG_H

/* 中文解读：ET6039底层资源映射集中定义，声明ADC虚拟通道、SRPWM实例/通道及外设初始化接口。
 * 这里的通道编号必须与J10原理图、init_config.c和mc_parameters.c三者一致。 */

// 物理通道
#define ADC_VBUS_CHX                        ADC_ANALOG_CHANNEL2                                         /* 电源电压采集ADC通道 */ 
#define ADC_VTEMP_CHX                       ADC_ANALOG_CHANNEL9                                         /* 驱动板温度采集ADC通道 */ 
#define ADC_AMPU_CHX                        ADC_ANALOG_CHANNEL14                                        /* U相电流采集ADC0通道 */ 
#define ADC_AMPV_CHX                        ADC_ANALOG_CHANNEL7                                         /* V相电流采集ADC0通道 */ 
#define ADC_AMPW_CHX                        ADC_ANALOG_CHANNEL12                                        /* W相电流采集ADC0通道 */ 
#define ADC_AMPU_CHX_C						ADC_ANALOG_CHANNEL4                                         /* U相电流采集ADC2通道 */
#define ADC_AMPV_CHX_C						ADC_ANALOG_CHANNEL3                                         /* V相电流采集ADC2通道 */
#define ADC_AMPW_CHX_C						ADC_ANALOG_CHANNEL5                                			/* W相电流采集ADC2通道 */



#define ADC_CH_NUM                          8                                                           /* ADC 通道数量 */

// 虚拟通道
// 三相电流采集每次都使用两个固定的虚拟通道
#define CURRENT_VIRTUAL_CHX_U     ADC_VIRTUAL_CHANNEL0
#define CURRENT_VIRTUAL_CHX_V     ADC_VIRTUAL_CHANNEL1
#define CURRENT_VIRTUAL_CHX_W	  ADC_VIRTUAL_CHANNEL2
#define VBUS_VIRTUAL_CH           ADC_VIRTUAL_CHANNEL3
#define VTEMP_VIRTUAL_CH          ADC_VIRTUAL_CHANNEL4

// U V W 三相SRPWM
//#define     SRPWM_PERIOD        200000/10       // 10KHz  改周期的同时需要修改PWM_FREQUENCY
#define     PWM_U               SRPWM6
#define     PWM_V               SRPWM7
#define     PWM_W               SRPWM4
#define     PWM_U_CH            SRPWM_CHANNEL_6
#define     PWM_V_CH            SRPWM_CHANNEL_7
#define     PWM_W_CH            SRPWM_CHANNEL_4
// ADC触发
#define     PWM_ADC_TRIGGER     SRPWM1
#define     PWM_ADC_TRIGGER_CH  SRPWM_CHANNEL_1
// PWM时基，用来判断周期更新
#define     PWM_CLK_TRIGGER     SRPWM0
#define     PWM_CLK_TRIGGER_CH  SRPWM_CHANNEL_0


void user_adc_init(void);
void user_srpwm_init(void);
void user_uart_init(void);
void sd_gpio_init(void);
void sa_gpio_init(void);

void ADC0_IRQHandler(void);
void USER_UART_IRQHandler(void);
void PWM_U_IRQHandler(void);
void MX_NVIC_Init(void);
#endif
