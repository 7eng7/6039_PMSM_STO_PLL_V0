/**
 * @file dac_ui.c
 * @brief 调试DAC用户界面适配。
 *
 * 中文解读：把选定的内部控制变量转换为调试DAC输出，便于示波器观察电流、速度和角度。
 * 当前硬件若未连接实际DAC，该模块仍可作为MCSDK UI接口占位，不参与电机闭环。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

/* 头文件 */
#include "dac_common_ui.h"
#include "dac_ui.h"
#include "et6x_dac.h"                   // 中文说明：初始化调试DAC。

#define DACOFF 32768    // DAC的偏移量，原始值32768

/**
  * @defgroup DAC_UserInterface 调试DAC功能模块
  * @brief 调试DAC相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/**
  * @brief  DAC_Init：初始化调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void DAC_Init(UI_Handle_t *pHandle)
{
    DAC_TriggerConfig config;
    /* 1. 初始化DAC1/2 */
    config.shadowLoadEn = DISABLE;
    config.triggerMask = UNMASKED;
    config.triggerInvertEn = DISABLE;
    config.triggerEn = ENABLE;
    config.triggerDelay = 0;
    /* 中文说明：处理并更新调试DAC。 */
    DAC_configTriggerMode(DAC0, &config);
    DAC_setReferenceVoltage(DAC0, DAC_REFSEL_INNER_LDO);
    DAC_setTriggerSource(DAC0, DAC_TRIGGER_SEL_SW);
    DAC_enableOutput(DAC0);
    /* 中文说明：执行并推进调试DAC。 */
    DAC_configTriggerMode(DAC1, &config);
    DAC_setReferenceVoltage(DAC1, DAC_REFSEL_INNER_LDO);
    DAC_setTriggerSource(DAC1, DAC_TRIGGER_SEL_SW);
    DAC_enableOutput(DAC1);

}

/**
  * @brief  DAC_Exec：执行并推进调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void DAC_Exec(UI_Handle_t *pHandle)
{
    DAC_UI_Handle_t *pDacHandle = (DAC_UI_Handle_t *)pHandle;
    MC_Protocol_REG_t bCh_var;

    bCh_var = pDacHandle->bChannel_variable[DAC_CH0];
    DAC_setValue(DAC0,  (DACOFF + (int16_t)UI_GetReg(pHandle, bCh_var, MC_NULL)) >> 4 );

    bCh_var = pDacHandle->bChannel_variable[DAC_CH1];
    DAC_setValue(DAC1,  (DACOFF + (int16_t)UI_GetReg(pHandle, bCh_var, MC_NULL)) >> 4 );

}
