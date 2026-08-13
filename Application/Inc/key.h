/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

#ifndef __KEY_H
#define __KEY_H

/* 中文解读：按键驱动接口及按下事件编码。key_scan(mode)负责消抖；mode决定是否允许连按，
 * 返回值仅表示用户事件，不代表电机启动命令已经由状态机执行。 */

#include "et6x.h"

#define KEY1_PIN        GPIO13   
#define KEY2_PIN        GPIO12
#define KEY1_PIN_MODE   GPIO13_MODE_GPIO13
#define KEY2_PIN_MODE   GPIO12_MODE_GPIO12

#define KEY1            GPIO_readPin(KEY1_PIN)     /* 读取KEY1引脚 */
#define KEY2            GPIO_readPin(KEY2_PIN)     /* 读取KEY2引脚 */

#define KEY1_PRES    1              /* KEY1按下 */
#define KEY2_PRES    2              /* KEY2按下 */

#define LED_PIN         GPIO34
#define LED_PIN_MODE    GPIO34_MODE_GPIO34 

#define FLAG_PIN_A      GPIO0
#define FLAG_PIN_B      GPIO1

#define FLAG_PIN_MODE_A GPIO0_MODE_GPIO0
#define FLAG_PIN_MODE_B GPIO1_MODE_GPIO1

/* 引脚电平设置 */
#define LED0(x)    (x ? GPIO_writePin(LED_PIN, SET) : GPIO_writePin(LED_PIN, RESET))
#define FLAG_A(x)    (x ? GPIO_writePin(FLAG_PIN_A, SET) : GPIO_writePin(FLAG_PIN_A, RESET))
#define FLAG_B(x)    (x ? GPIO_writePin(FLAG_PIN_B, SET) : GPIO_writePin(FLAG_PIN_B, RESET))

/* 取反定义 */
#define LED0_TOGGLE()   (GPIO_togglePin(LED_PIN))      /* LED0 = !LED0 */
#define FLAG_A_TOGGLE()   (GPIO_togglePin(FLAG_PIN_A))
#define FLAG_B_TOGGLE()   (GPIO_togglePin(FLAG_PIN_B))


void led_init(void);
void key_init(void);                /* 按键初始化函数 */
void Flag_init(void);
uint8_t key_scan(uint8_t mode);     /* 按键扫描函数 */

#endif





