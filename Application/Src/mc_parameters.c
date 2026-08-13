/**
 * @file mc_parameters.c
 * @brief 三电阻电流采样与 PWM/ADC 对应关系的常量配置。
 *
 * 中文解读：R3_2_ParamsM1 描述不同扇区应采哪两相电流、ADC虚拟通道与结果寄存器的对应、
 * 采样窗口裕量以及同步用的SRPWM实例。FOC只需两相电流，第三相由 Ia+Ib+Ic=0 重构；
 * 扇区切换时选择未被PWM边沿遮挡的两相。本表错误会直接表现为电流符号或相序异常。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
/* 头文件 */
#include "main.h"
#include "parameters_conversion.h"
#include "r3_2_f4xx_pwm_curr_fdbk.h"

#define FREQ_RATIO 1                /* 中文说明：处理并更新相关数据和控制状态。 */
#define FREQ_RELATION HIGHEST_FREQ  /* 中文说明：处理并更新相关数据和控制状态。 */

#define CURRENT_U         0
#define CURRENT_V         1
#define CURRENT_W         2

           
/**
  * @brief  当前接口：处理并更新相电流、电机、参数和增益，保持对象状态和控制流程一致。
  */
const R3_2_Params_t R3_2_ParamsM1 =
{
  .Tw                       = MAX_TWAIT,
  .bFreqRatio               = FREQ_RATIO,
  .bIsHigherFreqTim         = FREQ_RELATION,

/* 中文说明：初始化相电流、ADC采样。 */
  .ADCx_1                  = ADC0,
  .ADCx_2                  = ADC2,

/* 中文说明：处理并更新PWM和占空比、参数和增益。 */
  .SRPWMx                    =	PWM_CLK_TRIGGER,
  .RepetitionCounter         =	REP_COUNTER,
  .hTafter                    =	TW_AFTER,
  .hTbefore                   =	TW_BEFORE,

/* 中文说明：初始化PWM和占空比。 */
  .LowSideOutputs             =	(LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,

  .ADCConfig1 = {   ADC_AMPV_CHX    //扇区1--ADC0采样对应的模拟通道
                   ,ADC_AMPU_CHX	//扇区2
                   ,ADC_AMPU_CHX	//扇区3
                   ,ADC_AMPU_CHX	//扇区4
                   ,ADC_AMPU_CHX	//扇区5
                   ,ADC_AMPV_CHX	//扇区6
                  },

  .ADCConfig2 = {   ADC_AMPW_CHX_C	//扇区1--ADC2采样对应的模拟通道
                   ,ADC_AMPW_CHX_C	//扇区2
                   ,ADC_AMPW_CHX_C	//扇区3
                   ,ADC_AMPV_CHX_C	//扇区4
                   ,ADC_AMPV_CHX_C	//扇区5
                   ,ADC_AMPW_CHX_C	//扇区6
                  },
 
 .ADCConfig_VC1 = { CURRENT_VIRTUAL_CHX_V	//扇区1--ADC0采样对应的虚拟通道
	               ,CURRENT_VIRTUAL_CHX_U
				   ,CURRENT_VIRTUAL_CHX_U
				   ,CURRENT_VIRTUAL_CHX_U
				   ,CURRENT_VIRTUAL_CHX_U
				   ,CURRENT_VIRTUAL_CHX_V
                  },
				  
 .ADCConfig_VC2 = { CURRENT_VIRTUAL_CHX_W	//扇区1--ADC2采样对应的虚拟通道
	               ,CURRENT_VIRTUAL_CHX_W
				   ,CURRENT_VIRTUAL_CHX_W
				   ,CURRENT_VIRTUAL_CHX_V
				   ,CURRENT_VIRTUAL_CHX_V
				   ,CURRENT_VIRTUAL_CHX_W
                  },
				  				 				  
  .ADCDataReg1 = {
                   &ADC0->FIL_RESULT[CURRENT_V].FIL,	//ADC0扇区1采样的电流值
                   &ADC0->FIL_RESULT[CURRENT_U].FIL,	//ADC0扇区2
                   &ADC0->FIL_RESULT[CURRENT_U].FIL,
                   &ADC0->FIL_RESULT[CURRENT_U].FIL,
                   &ADC0->FIL_RESULT[CURRENT_U].FIL,
                   &ADC0->FIL_RESULT[CURRENT_V].FIL,    //ADC0扇区6采样的电流值
                  },
  .ADCDataReg2 = {
                   &ADC2->FIL_RESULT[CURRENT_W].FIL,	//ADC2扇区1采样的电流值
                   &ADC2->FIL_RESULT[CURRENT_W].FIL,	//ADC2扇区2
                   &ADC2->FIL_RESULT[CURRENT_W].FIL,
                   &ADC2->FIL_RESULT[CURRENT_V].FIL,
                   &ADC2->FIL_RESULT[CURRENT_V].FIL,
                   &ADC2->FIL_RESULT[CURRENT_W].FIL,	//ADC2扇区6采样的电流值
                  },

/* 中文说明：初始化PWM和占空比。 */
  .EmergencyStop                =	(FunctionalState) ENABLE,
};

