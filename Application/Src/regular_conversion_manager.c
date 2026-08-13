/**
 * @file regular_conversion_manager.c
 * @brief 非电流环ADC规则转换管理器。
 *
 * 中文解读：电流采样具有严格的PWM同步时序，不能被温度、电位器等低速采样打断。本模块
 * 对普通ADC请求进行登记、排队和结果回调，使其在允许的时间窗口执行，是实时电流采样与
 * 慢速模拟量测量之间的仲裁层。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
/* 头文件 */
#include "regular_conversion_manager.h"
#include "mc_config.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup RCM 相电流、母线电压、电压、ADC采样、故障与保护、温度、状态机状态、中断、电机、参数和增益功能模块
  * @brief 相电流、母线电压、电压、ADC采样、故障与保护、温度、状态机状态、中断、电机、参数和增益相关组件及其公共接口。
  * @{
  */

/* 私有类型定义 */

typedef struct
{
  RCM_exec_cb_t cb;
  void * data;
} RCM_callback_t;

/* 私有宏定义 */
/**
  * @brief  当前接口：处理并更新母线电压、电压、ADC采样、故障与保护、温度、电机，保持对象状态和控制流程一致。
  */
#define RCM_MAX_CONV  4

/* 变量 */

RegConv_t * RCM_handle_array [RCM_MAX_CONV];
RCM_callback_t RCM_CB_array [RCM_MAX_CONV];

uint8_t RCM_UserConvHandle;
uint16_t RCM_UserConvValue;
RCM_UserConvState_t RCM_UserConvState;

/* 私有函数声明 */

/* 私有函数 */

/**
  * @brief  RCM_RegisterRegConv_WithCB：检查并判断ADC采样、参数和增益，保持对象状态和控制流程一致。
  * @param  regConv  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  fctCB  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  data  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
uint8_t RCM_RegisterRegConv_WithCB (RegConv_t * regConv, RCM_exec_cb_t fctCB, void *data)
{
  uint8_t handle;
  handle = RCM_RegisterRegConv(regConv);
  if (handle < RCM_MAX_CONV)
  {
    RCM_CB_array [handle].cb = fctCB;
    RCM_CB_array [handle].data = data;
  }
  return handle;
}

/**
  * @brief  RCM_RegisterRegConv：检查并判断ADC采样、参数和增益，保持对象状态和控制流程一致。
  * @param  regConv  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
uint8_t RCM_RegisterRegConv(RegConv_t * regConv)
{
  uint8_t handle=255;
  uint8_t i=0;

    /* 中文说明：处理并更新ADC采样。 */
    while (i < RCM_MAX_CONV)										//搜索空余的常规注册通道
    {
      if (  RCM_handle_array [i] == 0 && handle > RCM_MAX_CONV)
      {
        handle = i; /* 中文说明：检查并判断相关数据和控制状态。 */
      }
      /* 中文说明：处理并更新故障与保护。 */
      if (  RCM_handle_array [i] != 0 )
      {
        if ((RCM_handle_array [i]->channel == regConv->channel) &&
            (RCM_handle_array [i]->regADC == regConv->regADC))
          {
           handle =i; /* 中文说明：处理并更新相关数据和控制状态。 */
           i = RCM_MAX_CONV; /* 中文说明：处理并更新相关数据和控制状态。 */
          }
      }
      i++;
    }
    if (handle < RCM_MAX_CONV )										//注册搜索到的通道并完成注册
    {
      RCM_handle_array [handle] = regConv;
      RCM_CB_array [handle].cb = NULL; /* 中文说明：处理并更新相关数据和控制状态。 */

      ADC_clearInterruptStatus(regConv->regADC, regConv->channel, ADC_INT_TRIGGER_FILTER_RESULT);  //对注册的常规ADC采样信号清标志

      ADC_enable(regConv->regADC, regConv->channel);                                               //使能相关通道

    }
    else
    {
      /* 中文说明：设置故障与保护。 */
    }
  return handle;
}

/* 中文说明：读取并返回相电流、ADC采样。 */
uint16_t RCM_ExecRegularConv (uint8_t handle)
{
  uint16_t retVal;

  ADC_clearFilterStatus(RCM_handle_array[handle]->regADC, RCM_handle_array[handle]->channel);

  /* 中文说明：处理并更新ADC采样。 */
  while ( ADC_getFilterResultStatus(RCM_handle_array[handle]->regADC, RCM_handle_array[handle]->channel) == 0u ) {}
  retVal = (ADC_getFilterResult( RCM_handle_array[handle]->regADC, RCM_handle_array[handle]->channel) << 4 );       //将ET6039采样的数据左对齐

  return retVal;
}

/**
  * @brief  RCM_RequestUserConv：检查并判断ADC采样、状态机状态，保持对象状态和控制流程一致。
  * @param  handle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
bool RCM_RequestUserConv(uint8_t handle)
{
  bool retVal = false;
  if (RCM_UserConvState == RCM_USERCONV_IDLE)
  {
    RCM_UserConvHandle = handle;
    /* 中文说明：读取并返回相关数据和控制状态。 */
    RCM_UserConvState = RCM_USERCONV_REQUESTED;
    retVal = true;
  }
  return retVal;
}

/**
  * @brief  RCM_GetUserConv：读取并返回ADC采样、故障与保护、状态机状态，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
uint16_t RCM_GetUserConv(void)
{
  uint16_t hRetVal = 0xFFFFu;
  if (RCM_UserConvState == RCM_USERCONV_EOC)
  {
    hRetVal = RCM_UserConvValue;
    RCM_UserConvState = RCM_USERCONV_IDLE;
  }
  return hRetVal;
}

/* 中文说明：执行并推进相电流、ADC采样。 */
void RCM_ExecUserConv ()
{
  if (RCM_UserConvState == RCM_USERCONV_REQUESTED)
  {
    RCM_UserConvValue = RCM_ExecRegularConv (RCM_UserConvHandle);
    RCM_UserConvState = RCM_USERCONV_EOC;
    if (RCM_CB_array [RCM_UserConvHandle].cb != NULL)
    {
      RCM_UserConvState = RCM_USERCONV_IDLE;
      RCM_CB_array [RCM_UserConvHandle].cb (RCM_UserConvHandle, RCM_UserConvValue ,RCM_CB_array [RCM_UserConvHandle].data);
    }
  }
}

/**
  * @brief  RCM_GetUserConvState：读取并返回ADC采样、状态机状态，保持对象状态和控制流程一致。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
RCM_UserConvState_t RCM_GetUserConvState(void)
{
  return RCM_UserConvState;
}


