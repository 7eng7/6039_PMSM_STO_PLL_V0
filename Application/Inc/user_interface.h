/**
  ******************************************************************************
  * @file    user_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __USERINTERFACE_H
#define __USERINTERFACE_H

/* 中文解读：UI对象、可访问寄存器和通用读写接口定义。该层允许同一通信协议选择不同电机，
 * 并通过对象句柄间接访问控制器，避免协议代码直接依赖全局变量。 */

#ifdef __cplusplus
 extern "C" {
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 头文件 */
#include "mc_type.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "state_machine.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @addtogroup MCUI
  * @{
  */

/* 导出类型 */
/**
  * @brief  当前接口：处理并更新串口通信帧、电机，保持对象状态和控制流程一致。
  */
typedef enum
{
  MC_PROTOCOL_REG_TARGET_MOTOR,          /* 0   */
  MC_PROTOCOL_REG_FLAGS,                 /* 1   */
  MC_PROTOCOL_REG_STATUS,                /* 2   */
  MC_PROTOCOL_REG_CONTROL_MODE,          /* 3   */
  MC_PROTOCOL_REG_SPEED_REF,             /* 4   */
  MC_PROTOCOL_REG_SPEED_KP,              /* 5   */
  MC_PROTOCOL_REG_SPEED_KI,              /* 6   */
  MC_PROTOCOL_REG_SPEED_KD,              /* 7   */
  MC_PROTOCOL_REG_TORQUE_REF,            /* 8   */
  MC_PROTOCOL_REG_TORQUE_KP,             /* 9   */
  MC_PROTOCOL_REG_TORQUE_KI,             /* 10  */
  MC_PROTOCOL_REG_TORQUE_KD,             /* 11  */
  MC_PROTOCOL_REG_FLUX_REF,              /* 12  */
  MC_PROTOCOL_REG_FLUX_KP,               /* 13  */
  MC_PROTOCOL_REG_FLUX_KI,               /* 14  */
  MC_PROTOCOL_REG_FLUX_KD,               /* 15  */
  MC_PROTOCOL_REG_OBSERVER_C1,           /* 16  */
  MC_PROTOCOL_REG_OBSERVER_C2,           /* 17  */
  MC_PROTOCOL_REG_OBSERVER_CR_C1,        /* 18  */
  MC_PROTOCOL_REG_OBSERVER_CR_C2,        /* 19  */
  MC_PROTOCOL_REG_PLL_KI,                /* 20  */
  MC_PROTOCOL_REG_PLL_KP,                /* 21  */
  MC_PROTOCOL_REG_FLUXWK_KP,             /* 22  */
  MC_PROTOCOL_REG_FLUXWK_KI,             /* 23  */
  MC_PROTOCOL_REG_FLUXWK_BUS,            /* 24  */
  MC_PROTOCOL_REG_BUS_VOLTAGE,           /* 25  */
  MC_PROTOCOL_REG_HEATS_TEMP,            /* 26  */
  MC_PROTOCOL_REG_MOTOR_POWER,           /* 27  */
  MC_PROTOCOL_REG_DAC_OUT1,              /* 28  */
  MC_PROTOCOL_REG_DAC_OUT2,              /* 29  */
  MC_PROTOCOL_REG_SPEED_MEAS,            /* 30  */
  MC_PROTOCOL_REG_TORQUE_MEAS,           /* 31  */
  MC_PROTOCOL_REG_FLUX_MEAS,             /* 32  */
  MC_PROTOCOL_REG_FLUXWK_BUS_MEAS,       /* 33  */
  MC_PROTOCOL_REG_RUC_STAGE_NBR,         /* 34  */
  MC_PROTOCOL_REG_I_A,                   /* 35  */
  MC_PROTOCOL_REG_I_B,                   /* 36  */
  MC_PROTOCOL_REG_I_ALPHA,               /* 37  */
  MC_PROTOCOL_REG_I_BETA,                /* 38  */
  MC_PROTOCOL_REG_I_Q,                   /* 39  */
  MC_PROTOCOL_REG_I_D,                   /* 40  */
  MC_PROTOCOL_REG_I_Q_REF,               /* 41  */
  MC_PROTOCOL_REG_I_D_REF,               /* 42  */
  MC_PROTOCOL_REG_V_Q,                   /* 43  */
  MC_PROTOCOL_REG_V_D,                   /* 44  */
  MC_PROTOCOL_REG_V_ALPHA,               /* 45  */
  MC_PROTOCOL_REG_V_BETA,                /* 46  */
  MC_PROTOCOL_REG_MEAS_EL_ANGLE,         /* 47  */
  MC_PROTOCOL_REG_MEAS_ROT_SPEED,        /* 48  */
  MC_PROTOCOL_REG_OBS_EL_ANGLE,          /* 49  */
  MC_PROTOCOL_REG_OBS_ROT_SPEED,         /* 50  */
  MC_PROTOCOL_REG_OBS_I_ALPHA,           /* 51  */
  MC_PROTOCOL_REG_OBS_I_BETA,            /* 52  */
  MC_PROTOCOL_REG_OBS_BEMF_ALPHA,        /* 53  */
  MC_PROTOCOL_REG_OBS_BEMF_BETA,         /* 54  */
  MC_PROTOCOL_REG_OBS_CR_EL_ANGLE,       /* 55  */
  MC_PROTOCOL_REG_OBS_CR_ROT_SPEED,      /* 56  */
  MC_PROTOCOL_REG_OBS_CR_I_ALPHA,        /* 57  */
  MC_PROTOCOL_REG_OBS_CR_I_BETA,         /* 58  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_ALPHA,     /* 59  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_BETA,      /* 60  */
  MC_PROTOCOL_REG_DAC_USER1,             /* 61  */
  MC_PROTOCOL_REG_DAC_USER2,             /* 62  */
  MC_PROTOCOL_REG_MAX_APP_SPEED,         /* 63  */
  MC_PROTOCOL_REG_MIN_APP_SPEED,         /* 64  */
  MC_PROTOCOL_REG_IQ_SPEEDMODE,          /* 65  */
  MC_PROTOCOL_REG_EST_BEMF_LEVEL,        /* 66  */
  MC_PROTOCOL_REG_OBS_BEMF_LEVEL,        /* 67  */
  MC_PROTOCOL_REG_EST_CR_BEMF_LEVEL,     /* 68  */
  MC_PROTOCOL_REG_OBS_CR_BEMF_LEVEL,     /* 69  */
  MC_PROTOCOL_REG_FF_1Q,                 /* 70  */
  MC_PROTOCOL_REG_FF_1D,                 /* 71  */
  MC_PROTOCOL_REG_FF_2,                  /* 72  */
  MC_PROTOCOL_REG_FF_VQ,                 /* 73  */
  MC_PROTOCOL_REG_FF_VD,                 /* 74  */
  MC_PROTOCOL_REG_FF_VQ_PIOUT,           /* 75  */
  MC_PROTOCOL_REG_FF_VD_PIOUT,           /* 76  */
  MC_PROTOCOL_REG_PFC_STATUS,            /* 77  */
  MC_PROTOCOL_REG_PFC_FAULTS,            /* 78  */
  MC_PROTOCOL_REG_PFC_DCBUS_REF,         /* 79  */
  MC_PROTOCOL_REG_PFC_DCBUS_MEAS,        /* 80  */
  MC_PROTOCOL_REG_PFC_ACBUS_FREQ,        /* 81  */
  MC_PROTOCOL_REG_PFC_ACBUS_RMS,         /* 82  */
  MC_PROTOCOL_REG_PFC_I_KP,              /* 83  */
  MC_PROTOCOL_REG_PFC_I_KI,              /* 84  */
  MC_PROTOCOL_REG_PFC_I_KD,              /* 85  */
  MC_PROTOCOL_REG_PFC_V_KP,              /* 86  */
  MC_PROTOCOL_REG_PFC_V_KI,              /* 87  */
  MC_PROTOCOL_REG_PFC_V_KD,              /* 88  */
  MC_PROTOCOL_REG_PFC_STARTUP_DURATION,  /* 89  */
  MC_PROTOCOL_REG_PFC_ENABLED,           /* 90  */
  MC_PROTOCOL_REG_RAMP_FINAL_SPEED,      /* 91  */
  MC_PROTOCOL_REG_RAMP_DURATION,         /* 92  */
  MC_PROTOCOL_REG_HFI_EL_ANGLE,          /* 93  */
  MC_PROTOCOL_REG_HFI_ROT_SPEED,         /* 94  */
  MC_PROTOCOL_REG_HFI_CURRENT,           /* 95  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_PLL,      /* 96  */
  MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF, /* 97  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KP,         /* 98  */
  MC_PROTOCOL_REG_HFI_PI_PLL_KI,         /* 99  */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KP,       /* 100 */
  MC_PROTOCOL_REG_HFI_PI_TRACK_KI,       /* 101 */
  MC_PROTOCOL_REG_SC_CHECK,              /* 102 */
  MC_PROTOCOL_REG_SC_STATE,              /* 103 */
  MC_PROTOCOL_REG_SC_RS,                 /* 104 */
  MC_PROTOCOL_REG_SC_LS,                 /* 105 */
  MC_PROTOCOL_REG_SC_KE,                 /* 106 */
  MC_PROTOCOL_REG_SC_VBUS,               /* 107 */
  MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED,  /* 108 */
  MC_PROTOCOL_REG_SC_STEPS,              /* 109 */
  MC_PROTOCOL_REG_SPEED_KP_DIV,          /* 110 */
  MC_PROTOCOL_REG_SPEED_KI_DIV,          /* 111 */
  MC_PROTOCOL_REG_UID,                   /* 112 */
  MC_PROTOCOL_REG_HWTYPE,                /* 113 */
  MC_PROTOCOL_REG_CTRBDID,               /* 114 */
  MC_PROTOCOL_REG_PWBDID,                /* 115 */
  MC_PROTOCOL_REG_SC_PP,                 /* 116 */
  MC_PROTOCOL_REG_SC_CURRENT,            /* 117 */
  MC_PROTOCOL_REG_SC_SPDBANDWIDTH,       /* 118 */
  MC_PROTOCOL_REG_SC_LDLQRATIO,          /* 119 */
  MC_PROTOCOL_REG_SC_NOMINAL_SPEED,      /* 120 */
  MC_PROTOCOL_REG_SC_CURRBANDWIDTH,      /* 121 */
  MC_PROTOCOL_REG_SC_J,                  /* 122 */
  MC_PROTOCOL_REG_SC_F,                  /* 123 */
  MC_PROTOCOL_REG_SC_MAX_CURRENT,        /* 124 */
  MC_PROTOCOL_REG_SC_STARTUP_SPEED,      /* 125 */
  MC_PROTOCOL_REG_SC_STARTUP_ACC,        /* 126 */
  MC_PROTOCOL_REG_SC_PWM_FREQUENCY,      /* 127 */
  MC_PROTOCOL_REG_SC_FOC_REP_RATE,       /* 128 */
  MC_PROTOCOL_REG_PWBDID2,               /* 129 */
  MC_PROTOCOL_REG_SC_COMPLETED,          /* 130 */
  MC_PROTOCOL_REG_CURRENT_POSITION,      /* 131 */
  MC_PROTOCOL_REG_TARGET_POSITION,       /* 132 */
  MC_PROTOCOL_REG_MOVE_DURATION,         /* 133 */
  MC_PROTOCOL_REG_POSITION_KP,           /* 134 */
  MC_PROTOCOL_REG_POSITION_KI,           /* 135 */
  MC_PROTOCOL_REG_POSITION_KD,           /* 136 */
  MC_PROTOCOL_REG_UNDEFINED
} MC_Protocol_REG_t;

/**
* @brief  当前接口：处理并更新调试DAC，保持对象状态和控制流程一致。
*/
typedef enum
{
  DAC_CH0,
  DAC_CH1,
// 中文说明：处理并更新调试DAC。
// 中文说明：处理并更新调试DAC。
// 中文说明：处理并更新调试DAC。
} DAC_Channel_t;

/**
* @brief  当前接口：处理并更新调试DAC，保持对象状态和控制流程一致。
*/
typedef enum
{
  DAC_USER1,
  DAC_USER2
} DAC_UserChannel_t;

/* 中文说明：设置相关数据和控制状态。 */
#define UI_SCODE_NONE     0x0u
#define UI_SCODE_HALL     0x1u /* 中文说明：处理并更新相关数据和控制状态。 */
#define UI_SCODE_ENC      0x2u /* 中文说明：处理并更新相关数据和控制状态。 */
#define UI_SCODE_STO_PLL  0x9u /* 中文说明：处理并更新状态机状态、观测器、反电势和PLL。 */
#define UI_SCODE_STO_CR   0xAu /* 中文说明：处理并更新状态机状态、观测器、反电势和PLL。 */
#define UI_SCODE_HFINJ    0xBu /* 中文说明：处理并更新相关数据和控制状态。 */

#define UI_CFGOPT_NONE            0x00000000u /* 中文说明：启动或使能相关数据和控制状态。 */
#define UI_CFGOPT_FW              0x00000001u /* 中文说明：启动或使能相关数据和控制状态。 */
#define UI_CFGOPT_SPEED_KD        0x00000002u /* 中文说明：启动或使能速度、PI/PID调节器。 */
#define UI_CFGOPT_Iq_KD           0x00000004u /* 中文说明：启动或使能PI/PID调节器。 */
#define UI_CFGOPT_Id_KD           0x00000008u /* 中文说明：启动或使能PI/PID调节器。 */
#define UI_CFGOPT_DAC             0x00000010u /* 中文说明：启动或使能调试DAC。 */
#define UI_CFGOPT_SETIDINSPDMODE  0x00000020u /* 中文说明：启动或使能速度。 */
#define UI_CFGOPT_PLLTUNING       0x00000040u /* 中文说明：启动或使能观测器、反电势和PLL。 */
#define UI_CFGOPT_PFC             0x00000080u /* 中文说明：启动或使能相关数据和控制状态。 */

#define UI_CFGOPT_PFC_I_KD        0x00000100u /* 中文说明：启动或使能相电流、PI/PID调节器。 */
#define UI_CFGOPT_PFC_V_KD        0x00000200u /* 中文说明：启动或使能电压、PI/PID调节器。 */

#define MC_PROTOCOL_CODE_SET_REG          0x01
#define MC_PROTOCOL_CODE_GET_REG          0x02
#define MC_PROTOCOL_CODE_EXECUTE_CMD      0x03
#define MC_PROTOCOL_CODE_STORE_TOADDR     0x04
#define MC_PROTOCOL_CODE_LOAD_FROMADDR    0x05
#define MC_PROTOCOL_CODE_GET_BOARD_INFO   0x06
#define MC_PROTOCOL_CODE_SET_SPEED_RAMP   0x07
#define MC_PROTOCOL_CODE_GET_REVUP_DATA   0x08
#define MC_PROTOCOL_CODE_SET_REVUP_DATA   0x09
#define MC_PROTOCOL_CODE_SET_CURRENT_REF  0x0A
#define MC_PROTOCOL_CODE_GET_MP_INFO      0x0B
#define MC_PROTOCOL_CODE_GET_FW_VERSION   0x0C
#define MC_PROTOCOL_CODE_SET_TORQUE_RAMP  0x0D
#define MC_PROTOCOL_CODE_SET_POSITION_CMD 0x12

#define MC_PROTOCOL_CMD_START_MOTOR       0x01
#define MC_PROTOCOL_CMD_STOP_MOTOR        0x02
#define MC_PROTOCOL_CMD_STOP_RAMP         0x03
#define MC_PROTOCOL_CMD_RESET             0x04
#define MC_PROTOCOL_CMD_PING              0x05
#define MC_PROTOCOL_CMD_START_STOP        0x06
#define MC_PROTOCOL_CMD_FAULT_ACK         0x07
#define MC_PROTOCOL_CMD_ENCODER_ALIGN     0x08
#define MC_PROTOCOL_CMD_IQDREF_CLEAR      0x09
#define MC_PROTOCOL_CMD_PFC_ENABLE        0x0A
#define MC_PROTOCOL_CMD_PFC_DISABLE       0x0B
#define MC_PROTOCOL_CMD_PFC_FAULT_ACK     0x0C
#define MC_PROTOCOL_CMD_SC_START          0x0D
#define MC_PROTOCOL_CMD_SC_STOP           0x0E

#define CTRBDID 0
#define PWBDID 0
#define MC_UID 883328122

/**
  * @brief  当前接口：处理并更新参数和增益，保持对象状态和控制流程一致。
  */
typedef const void UserInterfaceParams_t, *pUserInterfaceParams_t;

/**
  * @brief  void：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
typedef struct UI_Handle UI_Handle_t;

/**
  * @brief  void：执行并推进PWM和占空比，保持对象状态和控制流程一致。
  */
typedef void (*UI_Generic_Cb_t)( UI_Handle_t *pHandle);

/**
  * @brief  void：执行并推进中断，保持对象状态和控制流程一致。
  */
typedef void* (*UI_IrqHandler_Cb_t)( void *pHandle, unsigned char flag, unsigned short rx_data);

/**
  * @brief  void：执行并推进调试DAC，保持对象状态和控制流程一致。
  */
typedef void (*UI_DACSetChannelConfig_Cb_t)( UI_Handle_t *pHandle, DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable);

/**
  * @brief  MC_Protocol_REG_t：执行并推进串口通信帧、调试DAC，保持对象状态和控制流程一致。
  */
typedef MC_Protocol_REG_t (*UI_DACGetChannelConfig_Cb_t)( UI_Handle_t *pHandle, DAC_Channel_t bChannel);

/**
  * @brief  void：执行并推进调试DAC，保持对象状态和控制流程一致。
  */
typedef void (*UI_DACSetUserChannelValue_Cb_t)( UI_Handle_t *pHandle, uint8_t bUserChNumber, int16_t hValue);

/**
  * @brief  int16_t：执行并推进调试DAC，保持对象状态和控制流程一致。
  */
typedef int16_t (*UI_DACGetUserChannelValue_Cb_t)( UI_Handle_t *pHandle, uint8_t bUserChNumber);

/**
  * @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
struct UI_Handle
{

  UI_IrqHandler_Cb_t pFctIrqHandler;

  /* 函数 */
  UI_DACSetChannelConfig_Cb_t pFctDACSetChannelConfig;
  UI_DACGetChannelConfig_Cb_t pFctDACGetChannelConfig;
  UI_DACSetUserChannelValue_Cb_t pFctDACSetUserChannelValue;
  UI_DACGetUserChannelValue_Cb_t pFctDACGetUserChannelValue;

  UI_Generic_Cb_t pFct_DACInit;
  UI_Generic_Cb_t pFct_DACExec;
  uint8_t bDriveNum;      /* 中文说明：处理并更新相关数据和控制状态。 */
  MCI_Handle_t** pMCI;             /* 中文说明：初始化相关数据和控制状态。 */
  MCT_Handle_t** pMCT;             /* 中文说明：初始化相关数据和控制状态。 */
  uint32_t* pUICfg;       /* 中文说明：初始化相关数据和控制状态。 */
  uint8_t bSelectedDrive; /* 中文说明：初始化相电流。 */
};

/**
  * @brief  UI_Init：初始化相关数据和控制状态，保持对象状态和控制流程一致。
  */
void UI_Init(UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg);

/**
  * @brief  UI_SelectMC：设置相关数据和控制状态，保持对象状态和控制流程一致。
  */
bool UI_SelectMC(UI_Handle_t *pHandle,uint8_t bSelectMC);

/**
  * @brief  UI_GetSelectedMC：设置相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
uint8_t UI_GetSelectedMC(UI_Handle_t *pHandle);

/**
  * @brief  UI_GetSelectedMCConfig：设置相电流，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
uint32_t UI_GetSelectedMCConfig(UI_Handle_t *pHandle);

/**
  * @brief  UI_SetReg：设置串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bRegID  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wValue  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
bool UI_SetReg(UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, int32_t wValue);

/**
  * @brief  UI_GetReg：设置相电流、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bRegID  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  success  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
int32_t UI_GetReg(UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, bool * success );

/**
  * @brief  UI_ExecCmd：注册串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bCmdID  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
bool UI_ExecCmd(UI_Handle_t *pHandle, uint8_t bCmdID);

/* 中文说明：执行并推进速度、斜坡。 */
bool UI_ExecSpeedRamp(UI_Handle_t *pHandle, int32_t wFinalMecSpeedUnit, uint16_t hDurationms);

/**
  * @brief  UI_ExecTorqueRamp：设置转矩、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hTargetFinal  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
bool UI_ExecTorqueRamp(UI_Handle_t *pHandle, int16_t hTargetFinal, uint16_t hDurationms);

/* 中文说明：读取并返回相关数据和控制状态。 */
bool UI_GetRevupData(UI_Handle_t *pHandle, uint8_t bStage, uint16_t* pDurationms,
                     int16_t* pFinalMecSpeedUnit, int16_t* pFinalTorque );

/**
* @brief  UI_SetRevupData：设置相关数据和控制状态，保持对象状态和控制流程一致。
*/
bool UI_SetRevupData(UI_Handle_t *pHandle, uint8_t bStage, uint16_t hDurationms,
                     int16_t hFinalMecSpeedUnit, int16_t hFinalTorque );

/**
  * @brief  UI_SetCurrentReferences：设置相电流、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hIqRef  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hIdRef  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_SetCurrentReferences(UI_Handle_t *pHandle, int16_t hIqRef, int16_t hIdRef);

/**
  * @brief  UI_GetMPInfo：读取并返回相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  stepList  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMPInfo  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
bool UI_GetMPInfo(pMPInfo_t stepList, pMPInfo_t MPInfo);

/**
  * @brief  UI_DACInit：初始化调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_DACInit(UI_Handle_t *pHandle);

/**
  * @brief  UI_DACExec：执行并推进调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_DACExec(UI_Handle_t *pHandle);

/**
  * @brief  UI_SetDAC：设置串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bVariable  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_SetDAC(UI_Handle_t *pHandle, DAC_Channel_t bChannel,
                         MC_Protocol_REG_t bVariable);

/**
  * @brief  UI_GetDAC：读取并返回相电流、串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
MC_Protocol_REG_t UI_GetDAC(UI_Handle_t *pHandle, DAC_Channel_t bChannel);

/**
  * @brief  UI_SetUserDAC：设置调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bUserChNumber  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hValue  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_SetUserDAC(UI_Handle_t *pHandle, DAC_UserChannel_t bUserChNumber, int16_t hValue);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/************************ （C）版权所有，文件结束 ************************/
