/**
 * @file motor_control_protocol.c
 * @brief 电机参数监视/调试通信协议的寄存器访问层。
 *
 * 中文解读：把通信帧中的寄存器编号映射到速度、转矩、PI参数、母线电压和故障字，并执行
 * 长度、类型与可写性检查。该文件服务于上位机调试，不参与FOC实时闭环。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/
/* 头文件 */
#include "user_interface.h"
#include "motor_control_protocol.h"

/**
  * @defgroup motor_control_protocol 串口通信帧、电机功能模块
  * @brief 串口通信帧、电机相关组件及其公共接口。
  * @todo 根据工程需要继续补充该模块文档。
  * @{
  */

/* 私有宏定义 */

#define ACK_NOERROR 0xF0
#define ACK_ERROR   0xFF
#define ATR_FRAME_START 0xE0

#define MC_PROTOCOL_CODE_NONE        0x00

/* 中文说明：处理并更新故障与保护。 */
typedef enum ERROR_CODE_e
{
    ERROR_NONE = 0,             /**
* @brief  当前接口：处理并更新故障与保护，保持对象状态和控制流程一致。
*/
    ERROR_BAD_FRAME_ID,         /**
* @brief  当前接口：处理并更新串口通信帧，保持对象状态和控制流程一致。
*/
    ERROR_CODE_SET_READ_ONLY,   /**
* @brief  当前接口：读取并返回相关数据和控制状态，保持对象状态和控制流程一致。
*/
    ERROR_CODE_GET_WRITE_ONLY,  /**
* @brief  当前接口：读取并返回相关数据和控制状态，保持对象状态和控制流程一致。
*/
    ERROR_CODE_NO_TARGET_DRIVE, /**
* @brief  当前接口：处理并更新电机，保持对象状态和控制流程一致。
*/
    ERROR_CODE_WRONG_SET,       /**
* @brief  当前接口：处理并更新串口通信帧，保持对象状态和控制流程一致。
*/
    ERROR_CODE_CMD_ID,          /**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
    ERROR_CODE_WRONG_CMD,       /**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
    ERROR_CODE_OVERRUN,         /**
* @brief  当前接口：处理并更新速度、故障与保护、串口通信帧，保持对象状态和控制流程一致。
*/
    ERROR_CODE_TIMEOUT,         /**
* @brief  当前接口：处理并更新故障与保护、串口通信帧，保持对象状态和控制流程一致。
*/
    ERROR_CODE_BAD_CRC,         /**
* @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
*/
    ERROR_BAD_MOTOR_SELECTED,   /**
* @brief  当前接口：处理并更新电机，保持对象状态和控制流程一致。
*/
    ERROR_MP_NOT_ENABLED        /**
* @brief  当前接口：处理并更新电机，保持对象状态和控制流程一致。
*/
} ERROR_CODE;

MPInfo_t MPInfo = {0, 0};

/**
  * @brief  MCP_Init：初始化串口通信帧、调试DAC、参数和增益，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pFCP  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  fFcpSend  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  fFcpReceive  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  fFcpAbortReceive  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pDAC  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  s_fwVer  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_Init(MCP_Handle_t *pHandle,
                     FCP_Handle_t *pFCP,
                     FCP_SendFct_t fFcpSend,
                     FCP_ReceiveFct_t fFcpReceive,
                     FCP_AbortReceiveFct_t fFcpAbortReceive,
                     DAC_UI_Handle_t *pDAC,
                     const char *s_fwVer)
{
    pHandle->pFCP = pFCP;
    pHandle->pDAC = pDAC;
    pHandle->s_fwVer = s_fwVer;
    FCP_SetClient(pFCP, pHandle,
                  (FCP_SentFrameCallback_t) & MCP_SentFrame,
                  (FCP_ReceivedFrameCallback_t) & MCP_ReceivedFrame,
                  (FCP_RxTimeoutCallback_t) & MCP_OnTimeOut);
    pHandle->fFcpSend = fFcpSend;
    pHandle->fFcpReceive = fFcpReceive;
    pHandle->fFcpAbortReceive = fFcpAbortReceive;

    MCP_WaitNextFrame(pHandle);
}

/**
  * @brief  MCP_OnTimeOut：设置相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_OnTimeOut(MCP_Handle_t *pHandle)
{
    MCP_WaitNextFrame(pHandle);
}

/**
  * @brief  MCP_WaitNextFrame：检查并判断串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_WaitNextFrame(MCP_Handle_t *pHandle)
{
    pHandle->fFcpAbortReceive(pHandle->pFCP);
    pHandle->BufferSize = FCP_MAX_PAYLOAD_SIZE;
    pHandle->fFcpReceive(pHandle->pFCP);
}

/**
  * @brief  MCP_SentFrame：处理并更新缓冲区、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Code  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  buffer  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Size  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_SentFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
    MCP_WaitNextFrame(pHandle);
}

/**
  * @brief  MCP_ReceivedFrame：处理并更新缓冲区、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Code  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  buffer  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  Size  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_ReceivedFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
    bool RequireAck = true;
    bool bNoError = false; // 中文说明：处理并更新故障与保护。
    uint8_t bErrorCode;

    /* 中文说明：处理并更新串口通信帧、电机。 */
    uint8_t bMotorSelection = (Code & 0xE0) >> 5; /* 中文说明：处理并更新相关数据和控制状态。 */
    if (bMotorSelection != 0)
    {
        if (UI_SetReg(&pHandle->_Super, MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1))
        {
            Code &= 0x1F; /* 中文说明：处理并更新相关数据和控制状态。 */

            /* 中文说明：处理并更新调试DAC、电机。 */
            if (pHandle->pDAC)
            {
                UI_SetReg(&pHandle->pDAC->_Super, MC_PROTOCOL_REG_TARGET_MOTOR, bMotorSelection - 1);
            }
        }
        else
        {
            Code = MC_PROTOCOL_CODE_NONE; /* 中文说明：处理并更新故障与保护。 */
            bErrorCode = ERROR_BAD_MOTOR_SELECTED;
        }
    }

    switch (Code)
    {
    case MC_PROTOCOL_CODE_SET_REG:
    {
        MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
        bErrorCode = ERROR_CODE_WRONG_SET;

        switch (bRegID)
        {
        case MC_PROTOCOL_REG_TARGET_MOTOR:
        {
            /* 中文说明：处理并更新相关数据和控制状态。 */
            int32_t wValue = (int32_t)(buffer[1]);

            UI_SetReg(&pHandle->pDAC->_Super, bRegID, wValue);
            bNoError = UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;
        case MC_PROTOCOL_REG_CONTROL_MODE:
        case MC_PROTOCOL_REG_SC_PP:
        {
            /* 变量 */
            bNoError = UI_SetReg(&pHandle->_Super, bRegID, (int32_t)(buffer[1]));
        }
        break;

        case MC_PROTOCOL_REG_DAC_OUT1:
        {
            UI_SetDAC(&pHandle->pDAC->_Super, DAC_CH0, (MC_Protocol_REG_t)(buffer[1]));
            bNoError = true; /* 中文说明：检查并判断相关数据和控制状态。 */
        }
        break;

        case MC_PROTOCOL_REG_DAC_OUT2:
        {
            UI_SetDAC(&pHandle->pDAC->_Super, DAC_CH1, (MC_Protocol_REG_t)(buffer[1]));
            bNoError = true; /* 中文说明：检查并判断相关数据和控制状态。 */
        }
        break;

        case MC_PROTOCOL_REG_TORQUE_REF:
        case MC_PROTOCOL_REG_FLUX_REF:
        case MC_PROTOCOL_REG_SPEED_KP:
        case MC_PROTOCOL_REG_SPEED_KI:
        case MC_PROTOCOL_REG_SPEED_KD:
        case MC_PROTOCOL_REG_TORQUE_KP:
        case MC_PROTOCOL_REG_TORQUE_KI:
        case MC_PROTOCOL_REG_TORQUE_KD:
        case MC_PROTOCOL_REG_FLUX_KP:
        case MC_PROTOCOL_REG_FLUX_KI:
        case MC_PROTOCOL_REG_FLUX_KD:
        case MC_PROTOCOL_REG_PLL_KI:
        case MC_PROTOCOL_REG_PLL_KP:
        case MC_PROTOCOL_REG_FLUXWK_KP:
        case MC_PROTOCOL_REG_FLUXWK_KI:
        case MC_PROTOCOL_REG_FLUXWK_BUS:
        case MC_PROTOCOL_REG_IQ_SPEEDMODE:
        case MC_PROTOCOL_REG_PFC_DCBUS_REF:
        case MC_PROTOCOL_REG_PFC_I_KP:
        case MC_PROTOCOL_REG_PFC_I_KI:
        case MC_PROTOCOL_REG_PFC_I_KD:
        case MC_PROTOCOL_REG_PFC_V_KP:
        case MC_PROTOCOL_REG_PFC_V_KI:
        case MC_PROTOCOL_REG_PFC_V_KD:
        case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
        case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
        case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
        case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
        case MC_PROTOCOL_REG_POSITION_KP:
        case MC_PROTOCOL_REG_POSITION_KI:
        case MC_PROTOCOL_REG_POSITION_KD:
        {
            /* 变量 */
            int32_t wValue = buffer[1] + (buffer[2] << 8);
            bNoError = UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;

        case MC_PROTOCOL_REG_OBSERVER_C1:
        case MC_PROTOCOL_REG_OBSERVER_C2:
        case MC_PROTOCOL_REG_FF_1Q:
        case MC_PROTOCOL_REG_FF_1D:
        case MC_PROTOCOL_REG_FF_2:
        case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
        case MC_PROTOCOL_REG_SC_CURRENT:
        case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
        case MC_PROTOCOL_REG_SC_LDLQRATIO:
        case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
        case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
        case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
        case MC_PROTOCOL_REG_SC_STARTUP_ACC:
        {
            /* 变量 */
            int32_t wValue = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
            bNoError = UI_SetReg(&pHandle->_Super, bRegID, wValue);
        }
        break;

        default:
        {
            bErrorCode = ERROR_CODE_SET_READ_ONLY;
        }
        break;
        }
    }
    break;

    case MC_PROTOCOL_CODE_GET_REG:
    {
        MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buffer[0];
        bErrorCode = ERROR_CODE_GET_WRITE_ONLY;

        switch (bRegID)
        {
        case MC_PROTOCOL_REG_TARGET_MOTOR:
        case MC_PROTOCOL_REG_STATUS:
        case MC_PROTOCOL_REG_CONTROL_MODE:
        case MC_PROTOCOL_REG_RUC_STAGE_NBR:
        case MC_PROTOCOL_REG_PFC_STATUS:
        case MC_PROTOCOL_REG_PFC_ENABLED:
        case MC_PROTOCOL_REG_SC_CHECK:
        case MC_PROTOCOL_REG_SC_STATE:
        case MC_PROTOCOL_REG_SC_STEPS:
        case MC_PROTOCOL_REG_SC_PP:
        case MC_PROTOCOL_REG_SC_FOC_REP_RATE:
        case MC_PROTOCOL_REG_SC_COMPLETED:
        {
            /* 变量 */
            int32_t value = UI_GetReg(&pHandle->_Super, bRegID, &bNoError);
            if (bNoError == true)
            {
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 1);
                RequireAck = false;
            }
        }
        break;

        case MC_PROTOCOL_REG_DAC_OUT1:
        {
            if (pHandle->pDAC)
            {
                MC_Protocol_REG_t value = UI_GetDAC(&pHandle->pDAC->_Super, DAC_CH0);
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 1);
                bNoError = true;
                RequireAck = false;
            }
        }
        break;

        case MC_PROTOCOL_REG_DAC_OUT2:
        {
            if (pHandle->pDAC)
            {
                MC_Protocol_REG_t value = UI_GetDAC(&pHandle->pDAC->_Super, DAC_CH1);
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 1);
                bNoError = true;
            }
        }
        break;

        case MC_PROTOCOL_REG_SPEED_KP:
        case MC_PROTOCOL_REG_SPEED_KP_DIV:
        case MC_PROTOCOL_REG_SPEED_KI:
        case MC_PROTOCOL_REG_SPEED_KI_DIV:
        case MC_PROTOCOL_REG_SPEED_KD:
        case MC_PROTOCOL_REG_TORQUE_REF:
        case MC_PROTOCOL_REG_TORQUE_KP:
        case MC_PROTOCOL_REG_TORQUE_KI:
        case MC_PROTOCOL_REG_TORQUE_KD:
        case MC_PROTOCOL_REG_FLUX_REF:
        case MC_PROTOCOL_REG_FLUX_KP:
        case MC_PROTOCOL_REG_FLUX_KI:
        case MC_PROTOCOL_REG_FLUX_KD:
        case MC_PROTOCOL_REG_OBSERVER_C1:
        case MC_PROTOCOL_REG_OBSERVER_C2:
        case MC_PROTOCOL_REG_OBSERVER_CR_C1:
        case MC_PROTOCOL_REG_OBSERVER_CR_C2:
        case MC_PROTOCOL_REG_PLL_KP:
        case MC_PROTOCOL_REG_PLL_KI:
        case MC_PROTOCOL_REG_FLUXWK_KP:
        case MC_PROTOCOL_REG_FLUXWK_KI:
        case MC_PROTOCOL_REG_FLUXWK_BUS:
        case MC_PROTOCOL_REG_BUS_VOLTAGE:
        case MC_PROTOCOL_REG_HEATS_TEMP:
        case MC_PROTOCOL_REG_MOTOR_POWER:
        case MC_PROTOCOL_REG_TORQUE_MEAS:
        case MC_PROTOCOL_REG_FLUX_MEAS:
        case MC_PROTOCOL_REG_FLUXWK_BUS_MEAS:
        case MC_PROTOCOL_REG_IQ_SPEEDMODE:
        case MC_PROTOCOL_REG_FF_VQ:
        case MC_PROTOCOL_REG_FF_VD:
        case MC_PROTOCOL_REG_FF_VQ_PIOUT:
        case MC_PROTOCOL_REG_FF_VD_PIOUT:
        case MC_PROTOCOL_REG_PFC_DCBUS_REF:
        case MC_PROTOCOL_REG_PFC_DCBUS_MEAS:
        case MC_PROTOCOL_REG_PFC_ACBUS_FREQ:
        case MC_PROTOCOL_REG_PFC_ACBUS_RMS:
        case MC_PROTOCOL_REG_PFC_I_KP:
        case MC_PROTOCOL_REG_PFC_I_KI:
        case MC_PROTOCOL_REG_PFC_I_KD:
        case MC_PROTOCOL_REG_PFC_V_KP:
        case MC_PROTOCOL_REG_PFC_V_KI:
        case MC_PROTOCOL_REG_PFC_V_KD:
        case MC_PROTOCOL_REG_PFC_STARTUP_DURATION:
        case MC_PROTOCOL_REG_HFI_EL_ANGLE:
        case MC_PROTOCOL_REG_HFI_ROT_SPEED:
        case MC_PROTOCOL_REG_HFI_CURRENT:
        case MC_PROTOCOL_REG_HFI_INIT_ANG_PLL:
        case MC_PROTOCOL_REG_HFI_INIT_ANG_SAT_DIFF:
        case MC_PROTOCOL_REG_HFI_PI_TRACK_KP:
        case MC_PROTOCOL_REG_HFI_PI_TRACK_KI:
        case MC_PROTOCOL_REG_CTRBDID:
        case MC_PROTOCOL_REG_PWBDID:
        case MC_PROTOCOL_REG_POSITION_KP:
        case MC_PROTOCOL_REG_POSITION_KI:
        case MC_PROTOCOL_REG_POSITION_KD:
        {
            int32_t value = UI_GetReg(&pHandle->_Super, bRegID, &bNoError);
            if (bNoError == true)
            {
                /* 变量 */
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 2);
                RequireAck = false;
            }
        }
        break;

        case MC_PROTOCOL_REG_FLAGS:
        case MC_PROTOCOL_REG_SPEED_REF:
        case MC_PROTOCOL_REG_SPEED_MEAS:
        case MC_PROTOCOL_REG_FF_1Q:
        case MC_PROTOCOL_REG_FF_1D:
        case MC_PROTOCOL_REG_FF_2:
        case MC_PROTOCOL_REG_PFC_FAULTS:
        case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
        case MC_PROTOCOL_REG_SC_RS:
        case MC_PROTOCOL_REG_SC_LS:
        case MC_PROTOCOL_REG_SC_KE:
        case MC_PROTOCOL_REG_SC_VBUS:
        case MC_PROTOCOL_REG_SC_MEAS_NOMINALSPEED:
        case MC_PROTOCOL_REG_SC_CURRENT:
        case MC_PROTOCOL_REG_SC_SPDBANDWIDTH:
        case MC_PROTOCOL_REG_SC_LDLQRATIO:
        case MC_PROTOCOL_REG_SC_NOMINAL_SPEED:
        case MC_PROTOCOL_REG_SC_CURRBANDWIDTH:
        case MC_PROTOCOL_REG_SC_J:
        case MC_PROTOCOL_REG_SC_F:
        case MC_PROTOCOL_REG_SC_MAX_CURRENT:
        case MC_PROTOCOL_REG_SC_STARTUP_SPEED:
        case MC_PROTOCOL_REG_SC_STARTUP_ACC:
        case MC_PROTOCOL_REG_SC_PWM_FREQUENCY:
        case MC_PROTOCOL_REG_UID:
        {
            int32_t value = UI_GetReg(&pHandle->_Super, bRegID, &bNoError);
            if (bNoError == true)
            {
                /* 变量 */
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 4);
                RequireAck = false;
            }
        }
        break;

        case MC_PROTOCOL_REG_CURRENT_POSITION:
        case MC_PROTOCOL_REG_TARGET_POSITION:
        case MC_PROTOCOL_REG_MOVE_DURATION:
        {
            int32_t value = UI_GetReg(&pHandle->_Super, bRegID, &bNoError);
            if (bNoError == true)
            {
                /* 变量 */
                pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, (uint8_t*)(&value), 4);
                RequireAck = false;
            }
        }
        break;

        default:
            bErrorCode = ERROR_CODE_GET_WRITE_ONLY;
            break;
        }
    }
    break;

    case MC_PROTOCOL_CODE_EXECUTE_CMD:
    {
        uint8_t bCmdID = buffer[0];
        bErrorCode = ERROR_CODE_WRONG_CMD;
        bNoError = UI_ExecCmd(&pHandle->_Super, bCmdID);
    }
    break;

    case MC_PROTOCOL_CODE_GET_BOARD_INFO:
    {
        /* 中文说明：处理并更新相关数据和控制状态。 */
        unsigned char i;
        uint8_t outBuff[32];
        for (i = 0; i < 32; i++)
        {
            outBuff[i] = 0;
        }
        for (i = 0; (i < 29) && (pHandle->s_fwVer[i] != '\t'); i++)
        {
            outBuff[3 + i] = pHandle->s_fwVer[i];
        }
        outBuff[0] = pHandle->s_fwVer[i + 5];
        outBuff[1] = pHandle->s_fwVer[i + 7];
        outBuff[2] = pHandle->s_fwVer[i + 9];
        pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, outBuff, 32);
        bNoError = true;
    }
    break;

    case MC_PROTOCOL_CODE_SET_SPEED_RAMP:
    {
        uint16_t duration = buffer[4] + (buffer[5] << 8);
        int32_t rpm = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
        bNoError = UI_ExecSpeedRamp(&pHandle->_Super, rpm, duration);
    }
    break;
    case MC_PROTOCOL_CODE_SET_POSITION_CMD:
    {
    }
    break;

    case MC_PROTOCOL_CODE_SET_TORQUE_RAMP:
    {
        uint16_t duration = buffer[4] + (buffer[5] << 8);
        int32_t torque = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
        bNoError = UI_ExecTorqueRamp(&pHandle->_Super, torque, duration);
    }
    break;

    case MC_PROTOCOL_CODE_GET_REVUP_DATA:
    {
        uint8_t outBuff[8];
        uint16_t Durationms;
        int16_t FinalMecSpeedUnit;
        int16_t FinalTorque;
        int32_t rpm;
        UI_GetRevupData(&pHandle->_Super, buffer[0], &Durationms, &FinalMecSpeedUnit, &FinalTorque);
        rpm = (FinalMecSpeedUnit * _RPM) / SPEED_UNIT;
        outBuff[0] = (uint8_t)(rpm);
        outBuff[1] = (uint8_t)(rpm >> 8);
        outBuff[2] = (uint8_t)(rpm >> 16);
        outBuff[3] = (uint8_t)(rpm >> 24);
        outBuff[4] = (uint8_t)(FinalTorque);
        outBuff[5] = (uint8_t)(FinalTorque >> 8);
        outBuff[6] = (uint8_t)(Durationms);
        outBuff[7] = (uint8_t)(Durationms >> 8);
        pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, outBuff, 8);
    }
    break;

    case MC_PROTOCOL_CODE_SET_REVUP_DATA:
    {
        uint8_t bStage;
        uint16_t hDurationms;
        int16_t hFinalMecSpeedUnit;
        int16_t hFinalTorque;
        int32_t rpm;
        bStage = buffer[0];
        hDurationms = buffer[7] + (buffer[8] << 8);
        rpm = buffer[1] + (buffer[2] << 8) + (buffer[3] << 16) + (buffer[4] << 24);
        hFinalMecSpeedUnit = (rpm * SPEED_UNIT) / _RPM ;
        hFinalTorque = buffer[5] + (buffer[6] << 8);
        bNoError = UI_SetRevupData(&pHandle->_Super, bStage, hDurationms, hFinalMecSpeedUnit, hFinalTorque);
    }
    break;

    case MC_PROTOCOL_CODE_SET_CURRENT_REF:
    {
        int16_t hIqRef;
        int16_t hIdRef;
        hIqRef = buffer[0] + (buffer[1] << 8);
        hIdRef = buffer[2] + (buffer[3] << 8);
        UI_SetCurrentReferences(&pHandle->_Super, hIqRef, hIdRef);
        bNoError = true;
    }
    break;

    case MC_PROTOCOL_CODE_GET_MP_INFO:
    {
        MPInfo_t stepList;
        stepList.data = buffer;
        stepList.len = Size;
        bErrorCode = ERROR_MP_NOT_ENABLED;
        bNoError = UI_GetMPInfo(&stepList, &MPInfo);

        if (bNoError)
        {
            pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, MPInfo.data, MPInfo.len);
            RequireAck = false;
        }
    }
    break;

    case MC_PROTOCOL_CODE_GET_FW_VERSION:
    {
        /* 中文说明：读取并返回相关数据和控制状态。 */
        unsigned char i;
        uint8_t outBuff[32];
        for (i = 0; (i < 32) && (pHandle->s_fwVer[i] != 0); i++)
        {
            outBuff[i] = pHandle->s_fwVer[i];
        }

        for (; i < 32; i++)
        {
            outBuff[i] = 0;
        }

        pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, outBuff, 32);
        bNoError = true;
    }
    break;

    case MC_PROTOCOL_CODE_NONE:
    {
    }
    break;

    default:
    {
        bErrorCode = ERROR_BAD_FRAME_ID;
    }
    break;
    }

    if (RequireAck)
    {
        if (bNoError)
        {
            pHandle->fFcpSend(pHandle->pFCP, ACK_NOERROR, MC_NULL, 0);
        }
        else
        {
            pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
        }
    }
}

/**
  * @brief  MCP_SendOverrunMessage：执行并推进速度、故障与保护、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_SendOverrunMessage(MCP_Handle_t *pHandle)
{
    uint8_t bErrorCode = ERROR_CODE_OVERRUN;
    pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
}

/**
  * @brief  MCP_SendTimeoutMessage：处理并更新故障与保护，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_SendTimeoutMessage(MCP_Handle_t *pHandle)
{
    uint8_t bErrorCode = ERROR_CODE_TIMEOUT;
    pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
}

/**
  * @brief  MCP_SendATRMessage：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_SendATRMessage(MCP_Handle_t *pHandle)
{
    uint32_t wUID = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_UID, MC_NULL);
    unsigned char i;
    uint8_t bFWX;
    uint8_t bFWY;
    uint8_t bFWZ;
    uint16_t hCBDID = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_CTRBDID, MC_NULL);
    uint16_t hPBDID = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_PWBDID, MC_NULL);
    uint16_t hPBDID2 = UI_GetReg(&pHandle->_Super, MC_PROTOCOL_REG_PWBDID2, MC_NULL);
    uint8_t buff[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for (i = 0; (i < 29) && (pHandle->s_fwVer[i] != '\t'); i++);

    bFWX = pHandle->s_fwVer[i + 5];
    bFWY = pHandle->s_fwVer[i + 7];
    bFWZ = pHandle->s_fwVer[i + 9];

    // 中文说明：处理并更新相关数据和控制状态。
    buff[0] = (uint8_t)(wUID & 0xff);
    buff[1] = (uint8_t)((wUID >> 8)  & 0xff);
    buff[2] = (uint8_t)((wUID >> 16) & 0xff);
    buff[3] = (uint8_t)((wUID >> 24) & 0xff);
    buff[4] = bFWX;
    buff[5] = bFWY;
    buff[6] = bFWZ;
    buff[7] = (uint8_t)(hCBDID);
    buff[8] = (uint8_t)(hCBDID >> 8);
    buff[9] = (uint8_t)(hPBDID);
    buff[10] = (uint8_t)(hPBDID >> 8);
    buff[11] = (uint8_t)(hPBDID2);
    buff[12] = (uint8_t)(hPBDID2 >> 8);

    pHandle->fFcpSend(pHandle->pFCP, ATR_FRAME_START, buff, 13);
}

/**
  * @brief  MCP_SendBadCRCMessage：处理并更新ADC采样，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  */
__WEAK void MCP_SendBadCRCMessage(MCP_Handle_t *pHandle)
{
    uint8_t bErrorCode = ERROR_CODE_BAD_CRC;
    pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
}


