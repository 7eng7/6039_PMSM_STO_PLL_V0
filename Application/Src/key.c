/**
 * @file key.c
 * @brief 控制板按键GPIO初始化、消抖和按键事件识别。
 *
 * 中文解读：机械触点会在动作瞬间抖动，本模块通过延时和状态锁存把持续电平转换为单次
 * KEYx_PRES事件。按键只改变应用目标，不应直接在这里操作功率PWM。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
#include "key.h"
#include "et6x_gpio.h"
#include "et6x_cortex.h"

/**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    IOC_Init_TypeDef init;
    GPIO_initPadConfigStruct(&init);

    init.dir = IOC_DIR_IN_OUT;
    /* 中文说明：设置相关数据和控制状态。 */
    init.pull = IOC_PULL_NONE;

    GPIO_setPadConfig(KEY1_PIN, &init);
    GPIO_setPadConfig(KEY2_PIN, &init);
    GPIO_setPinConfig(KEY1_PIN, KEY1_PIN_MODE); /* 中文说明：处理并更新数字输出。 */
    GPIO_setPinConfig(KEY2_PIN, KEY2_PIN_MODE); /* 中文说明：处理并更新数字输出。 */

    /* 中文说明：启动或使能数字输出。 */
    GPIO_setPortDirectionMode(KEY1_PIN, GPIO_DIR_IN);
    GPIO_setPortDirectionMode(KEY2_PIN, GPIO_DIR_IN);

}

/**
 * @brief       按键扫描函数
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;                      /* 按键按松开标志 */
    uint8_t keyval = 0;

    if (mode) key_up = 1;                           /* 支持连按 */

    if (key_up && (KEY1 == 0 || KEY2 == 0))         /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
        DWT_delay_ms(10);                           /* 去抖动10ms */
        key_up = 0;

        if (KEY1 == 0)  keyval = KEY1_PRES;

        if (KEY2 == 0)  keyval = KEY2_PRES;

    }
    else if (KEY1 == 1 && KEY2 == 1)                /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}


/**
 * @brief       初始化LED相关IO口,
 * @param       无
 * @retval      无
 */
void led_init(void)
{

    IOC_Init_TypeDef init;
    /* 中文说明：设置数字输出。 */
    GPIO_initPadConfigStruct(&init);
    /* 中文说明：初始化相关数据和控制状态。 */
    init.dir = IOC_DIR_OUT;
    /* 中文说明：初始化相关数据和控制状态。 */
    init.pull = IOC_PULL_NONE;
    init.ds = IOC_DS_LEVEL_3;
    GPIO_setPadConfig(LED_PIN, &init);
    GPIO_setPinConfig(LED_PIN, LED_PIN_MODE); /* 中文说明：初始化数字输出。 */
    /* 中文说明：初始化数字输出。 */
    GPIO_setPortDirectionMode(LED_PIN, GPIO_DIR_OUT);
    LED0(1);
}

/**
 * @brief       初始化FLAG相关IO口,用于指示程序运行的状态
 * @param       无
 * @retval      无
 */
void Flag_init(void)
{

    IOC_Init_TypeDef init;
    /* 中文说明：设置数字输出。 */
    GPIO_initPadConfigStruct(&init);
    /* 中文说明：设置相关数据和控制状态。 */
    init.dir = IOC_DIR_OUT;
    /* 中文说明：设置相关数据和控制状态。 */
    init.pull = IOC_PULL_DOWN;
    init.ds = IOC_DS_LEVEL_3;
    GPIO_setPadConfig(FLAG_PIN_A, &init);
    GPIO_setPadConfig(FLAG_PIN_B, &init);

    GPIO_setPinConfig(FLAG_PIN_A, FLAG_PIN_MODE_A); /* 中文说明：处理并更新数字输出。 */
    GPIO_setPinConfig(FLAG_PIN_B, FLAG_PIN_MODE_B);

    /* 中文说明：启动或使能数字输出。 */
    GPIO_setPortDirectionMode(FLAG_PIN_A, GPIO_DIR_OUT);
    GPIO_setPortDirectionMode(FLAG_PIN_B, GPIO_DIR_OUT);
    FLAG_A(0);
    FLAG_B(0);
}
















