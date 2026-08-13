/**
 * @file user_interface.c
 * @brief MCSDK 通用用户界面寄存器读写实现。
 *
 * 中文解读：根据电机号和寄存器ID读取/设置状态机、速度传感器、PI控制器及功率测量对象。
 * 大量switch分支是协议寄存器到内部对象的显式映射；此层用于观测与调参，不在高频FOC路径中。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

#include "mc_type.h"
#include "mc_math.h"
#include "mc_config.h"
#include "user_interface.h"

/**
  * @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup MCUI 串口通信帧、调试DAC、电机功能模块
  * @brief 串口通信帧、调试DAC、电机相关组件及其公共接口。
  * @{
  */

/**
  * @brief  UI_Init：初始化相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bMCNum  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMCI  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMCT  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pUICfg  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void UI_Init(UI_Handle_t *pHandle, uint8_t bMCNum, MCI_Handle_t ** pMCI, MCT_Handle_t** pMCT, uint32_t* pUICfg)
{
  pHandle->bDriveNum = bMCNum;
  pHandle->pMCI = pMCI;
  pHandle->pMCT = pMCT;
  pHandle->bSelectedDrive = 0u;
  pHandle->pUICfg = pUICfg;
}

/**
  * @brief  UI_SelectMC：设置相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bSelectMC:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_SelectMC(UI_Handle_t *pHandle,uint8_t bSelectMC)
{
  bool retVal = true;
  if (bSelectMC  >= pHandle->bDriveNum)
  {
    retVal = false;
  }
  else
  {
    pHandle->bSelectedDrive = bSelectMC;
  }
  return retVal;
}

/**
  * @brief  UI_GetSelectedMC：设置相电流，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint8_t UI_GetSelectedMC(UI_Handle_t *pHandle)
{
  return (pHandle->bSelectedDrive);
}

/**
  * @brief  UI_GetSelectedMCConfig：设置相电流，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK uint32_t UI_GetSelectedMCConfig(UI_Handle_t *pHandle)
{
  return pHandle->pUICfg[pHandle->bSelectedDrive];
}

/**
  * @brief  UI_SetReg：设置相电流、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bRegID:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wValue:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_SetReg(UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, int32_t wValue)
{
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
  MCT_Handle_t * pMCT = pHandle->pMCT[pHandle->bSelectedDrive];

  bool retVal = true;
  switch (bRegID)
  {
  case MC_PROTOCOL_REG_TARGET_MOTOR:
    {
      retVal = UI_SelectMC(pHandle,(uint8_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      MCI_ExecSpeedRamp(pMCI,(int16_t)((wValue*SPEED_UNIT)/_RPM),0);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KP:
    {
      PID_SetKP(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KI:
    {
      PID_SetKI(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_SPEED_KD:
    {
      PID_SetKD(pMCT->pPIDSpeed,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_CONTROL_MODE:
    {
      if ((STC_Modality_t)wValue == STC_TORQUE_MODE)
      {
        MCI_ExecTorqueRamp(pMCI, MCI_GetTeref(pMCI),0);
      }
      if ((STC_Modality_t)wValue == STC_SPEED_MODE)
      {
        MCI_ExecSpeedRamp(pMCI, MCI_GetMecSpeedRefUnit(pMCI),0);
      }
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_REF:
    {
      qd_t currComp;
      currComp = MCI_GetIqdref(pMCI);
      currComp.q = (int16_t)wValue;
      MCI_SetCurrentReferences(pMCI,currComp);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KP:
    {
      PID_SetKP(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KI:
    {
      PID_SetKI(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_TORQUE_KD:
    {
      PID_SetKD(pMCT->pPIDIq,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_REF:
    {
      qd_t currComp;
      currComp = MCI_GetIqdref(pMCI);
      currComp.d = (int16_t)wValue;
      MCI_SetCurrentReferences(pMCI,currComp);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KP:
    {
      PID_SetKP(pMCT->pPIDId,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KI:
    {
      PID_SetKI(pMCT->pPIDId,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_FLUX_KD:
    {
      PID_SetKD(pMCT->pPIDId,(int16_t)wValue);
    }
    break;

  case MC_PROTOCOL_REG_OBSERVER_C1:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
        STO_PLL_SetObserverGains((STO_PLL_Handle_t*)pSPD,(int16_t)wValue,hC2);
      }
    }
    break;

  case MC_PROTOCOL_REG_OBSERVER_C2:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
        STO_PLL_SetObserverGains((STO_PLL_Handle_t*)pSPD,hC1,(int16_t)wValue);
      }
    }
    break;

  case MC_PROTOCOL_REG_PLL_KI:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
        STO_SetPLLGains((STO_PLL_Handle_t*)pSPD,hPgain,(int16_t)wValue);
      }
    }
    break;

  case MC_PROTOCOL_REG_PLL_KP:
	{
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
        STO_SetPLLGains((STO_PLL_Handle_t*)pSPD,(int16_t)wValue,hIgain);
      }
    }
    break;

  case MC_PROTOCOL_REG_IQ_SPEEDMODE:
    {
      MCI_SetIdref(pMCI,(int16_t)wValue);
    }
    break;

  default:
    retVal = false;
    break;
  }

  return retVal;
}

/* 中文说明：读取并返回相关数据和控制状态。 */
__WEAK int32_t UI_GetReg(UI_Handle_t *pHandle, MC_Protocol_REG_t bRegID, bool * success)
{
  MCT_Handle_t* pMCT = pHandle->pMCT[pHandle->bSelectedDrive];
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  int32_t bRetVal = 0;

  if ( success != (bool *) 0 )
  {
    *success = true;
  }

  switch (bRegID)
  {
    case MC_PROTOCOL_REG_TARGET_MOTOR:
    {
      bRetVal = (int32_t)UI_GetSelectedMC(pHandle);
    }
    break;

    case MC_PROTOCOL_REG_FLAGS:
    {
      bRetVal = (int32_t)STM_GetFaultState(pMCT->pStateMachine);
    }
	break;

    case MC_PROTOCOL_REG_STATUS:
    {
      bRetVal = (int32_t)STM_GetState(pMCT->pStateMachine);
    }
	break;

    case MC_PROTOCOL_REG_SPEED_REF:
    {
      bRetVal = (int32_t)((MCI_GetMecSpeedRefUnit(pMCI)*_RPM)/SPEED_UNIT);
    }
    break;

    case MC_PROTOCOL_REG_SPEED_KP:
    {
      bRetVal = (int32_t)PID_GetKP(pMCT->pPIDSpeed);
    }
    break;

    case MC_PROTOCOL_REG_SPEED_KI:
    {
      bRetVal = (int32_t)PID_GetKI(pMCT->pPIDSpeed);
    }
    break;

    case MC_PROTOCOL_REG_SPEED_KD:
    {
      bRetVal = (int32_t)PID_GetKD(pMCT->pPIDSpeed);
    }
    break;

    case MC_PROTOCOL_REG_BUS_VOLTAGE:
    {
      bRetVal = (int32_t)VBS_GetAvBusVoltage_V(pMCT->pBusVoltageSensor);
    }
    break;

    case MC_PROTOCOL_REG_HEATS_TEMP:
    {
      bRetVal = (int32_t)NTC_GetAvTemp_C(pMCT->pTemperatureSensor);
    }
    break;

    case MC_PROTOCOL_REG_SPEED_MEAS:
    {
      bRetVal = (int32_t)((MCI_GetAvrgMecSpeedUnit(pMCI) * _RPM)/SPEED_UNIT);
    }
    break;

    case MC_PROTOCOL_REG_UID:
    {
      bRetVal = (int32_t)(MC_UID);
    }
    break;

    case MC_PROTOCOL_REG_CTRBDID:
    {
      bRetVal = CTRBDID;
    }
    break;

    case MC_PROTOCOL_REG_PWBDID:
    {
      bRetVal = PWBDID;
    }
    break;

    case MC_PROTOCOL_REG_PWBDID2:
    {
      bRetVal = (uint32_t) 0;
    }
    break;

    case MC_PROTOCOL_REG_TORQUE_REF:
    {
      qd_t currComp;
      currComp = MCI_GetIqdref(pMCI);
      bRetVal = (int32_t)currComp.q;
    }
    break;

    case MC_PROTOCOL_REG_FLUX_REF:
    {
      qd_t currComp;
      currComp = MCI_GetIqdref(pMCI);
      bRetVal = (int32_t)currComp.d;
    }
    break;

    case MC_PROTOCOL_REG_CONTROL_MODE:
    {
      bRetVal = (int32_t)MCI_GetControlMode(pMCI);
    }
    break;

    case MC_PROTOCOL_REG_RAMP_FINAL_SPEED:
    {
      if (MCI_GetControlMode(pMCI) == STC_SPEED_MODE)
      {
      bRetVal = (int32_t)((MCI_GetLastRampFinalSpeed(pMCI) * _RPM)/SPEED_UNIT) ;
      }
      else
      {
      bRetVal = (int32_t)((MCI_GetMecSpeedRefUnit(pMCI) * _RPM)/SPEED_UNIT) ;
      }
    }
    break;

    case MC_PROTOCOL_REG_SPEED_KP_DIV:
    {
      bRetVal = (int32_t)PID_GetKPDivisor(pMCT->pPIDSpeed);
    }
    break;

    case MC_PROTOCOL_REG_SPEED_KI_DIV:
    {
      bRetVal = (int32_t)PID_GetKIDivisor(pMCT->pPIDSpeed);
    }
    break;

    case MC_PROTOCOL_REG_TORQUE_KP:
    {
      bRetVal = (int32_t)PID_GetKP(pMCT->pPIDIq);
    }
    break;

    case MC_PROTOCOL_REG_TORQUE_KI:
    {
      bRetVal = (int32_t)PID_GetKI(pMCT->pPIDIq);
    }
    break;

    case MC_PROTOCOL_REG_TORQUE_KD:
    {
      bRetVal = (int32_t)PID_GetKD(pMCT->pPIDIq);
    }
    break;

    case MC_PROTOCOL_REG_IQ_SPEEDMODE:
    {
      qd_t currComp;
      currComp = MCI_GetIqdref(pMCI);
      bRetVal = (int32_t)currComp.d;
    }
    break;

    case MC_PROTOCOL_REG_FLUX_KP:
    {
      bRetVal = (int32_t)PID_GetKP(pMCT->pPIDId);
    }
    break;

    case MC_PROTOCOL_REG_FLUX_KI:
    {
      bRetVal = (int32_t)PID_GetKI(pMCT->pPIDId);
    }
    break;

    case MC_PROTOCOL_REG_FLUX_KD:
    {
      bRetVal = (int32_t)PID_GetKD(pMCT->pPIDId);
    }
    break;

    case MC_PROTOCOL_REG_OBSERVER_C1:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC1;
    }
    break;

    case MC_PROTOCOL_REG_OBSERVER_C2:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hC1,hC2;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_PLL_GetObserverGains((STO_PLL_Handle_t*)pSPD,&hC1,&hC2);
      }
      bRetVal = (int32_t)hC2;
    }
    break;

    case MC_PROTOCOL_REG_OBS_EL_ANGLE:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = SPD_GetElAngle(pSPD);
      }
    }
    break;

    case MC_PROTOCOL_REG_PLL_KP:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
      }
      bRetVal = (int32_t)hPgain;
    }
    break;

    case MC_PROTOCOL_REG_PLL_KI:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      int16_t hPgain, hIgain;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        STO_GetPLLGains((STO_PLL_Handle_t*)pSPD,&hPgain,&hIgain);
      }
      bRetVal = (int32_t)hIgain;
    }
    break;

    case MC_PROTOCOL_REG_OBS_ROT_SPEED:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = SPD_GetS16Speed(pSPD);
      }
    }
    break;

    case MC_PROTOCOL_REG_OBS_I_ALPHA:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetEstimatedCurrent((STO_PLL_Handle_t*)pSPD).alpha;
      }
    }
    break;

    case MC_PROTOCOL_REG_OBS_I_BETA:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetEstimatedCurrent((STO_PLL_Handle_t*)pSPD).beta;
      }
    }
    break;

    case MC_PROTOCOL_REG_OBS_BEMF_ALPHA:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD =  pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetEstimatedBemf((STO_PLL_Handle_t*)pSPD).alpha;
      }
    }
    break;

    case MC_PROTOCOL_REG_OBS_BEMF_BETA:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
       pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetEstimatedBemf((STO_PLL_Handle_t*)pSPD).beta;
      }
    }
    break;

    case MC_PROTOCOL_REG_EST_BEMF_LEVEL:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetEstimatedBemfLevel((STO_PLL_Handle_t*)pSPD) >> 16;
      }
    }
    break;

    case MC_PROTOCOL_REG_OBS_BEMF_LEVEL:
    {
      uint32_t hUICfg = pHandle->pUICfg[pHandle->bSelectedDrive];
      SpeednPosFdbk_Handle_t* pSPD = MC_NULL;
      if (MAIN_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorMain;
      }
      if (AUX_SCFG_VALUE(hUICfg) == UI_SCODE_STO_PLL)
      {
        pSPD = pMCT->pSpeedSensorAux;
      }
      if (pSPD != MC_NULL)
      {
        bRetVal = STO_PLL_GetObservedBemfLevel((STO_PLL_Handle_t*)pSPD) >> 16;
      }
    }
    break;

    case MC_PROTOCOL_REG_MOTOR_POWER:
    {
      bRetVal = MPM_GetAvrgElMotorPowerW(pMCT->pMPM);
    }
    break;

    case MC_PROTOCOL_REG_DAC_OUT1:
    {
      MC_Protocol_REG_t value = UI_GetDAC(pHandle, DAC_CH0);
      bRetVal = value;
    }
    break;

    case MC_PROTOCOL_REG_DAC_OUT2:
    {
      MC_Protocol_REG_t value = UI_GetDAC(pHandle, DAC_CH1);
      bRetVal = value;
    }
    break;

    case MC_PROTOCOL_REG_DAC_USER1:
    {
      if (pHandle->pFctDACGetUserChannelValue)
      {
        bRetVal = (int32_t) pHandle->pFctDACGetUserChannelValue(pHandle, 0);
      }
      else
      {
        bRetVal = (uint32_t) 0;
      }
    }
    break;

    case MC_PROTOCOL_REG_DAC_USER2:
    {
      if (pHandle->pFctDACGetUserChannelValue)
      {
        bRetVal = (int32_t) pHandle->pFctDACGetUserChannelValue(pHandle, 1);
      }
      else
      {
        bRetVal = (uint32_t) 0;
      }
    }
    break;

    case MC_PROTOCOL_REG_MAX_APP_SPEED:
    {
      bRetVal = (STC_GetMaxAppPositiveMecSpeedUnit(pMCT->pSpeednTorqueCtrl) * _RPM)/SPEED_UNIT ;
    }
    break;

    case MC_PROTOCOL_REG_MIN_APP_SPEED:
    {
      bRetVal = (STC_GetMinAppNegativeMecSpeedUnit(pMCT->pSpeednTorqueCtrl)  * _RPM)/SPEED_UNIT ;
    }
    break;

    case MC_PROTOCOL_REG_TORQUE_MEAS:
    case MC_PROTOCOL_REG_I_Q:
    {
      bRetVal = MCI_GetIqd(pMCI).q;
    }
    break;

    case MC_PROTOCOL_REG_FLUX_MEAS:
    case MC_PROTOCOL_REG_I_D:
    {
      bRetVal = MCI_GetIqd(pMCI).d;
    }
    break;

    case MC_PROTOCOL_REG_RUC_STAGE_NBR:
    {
      if (pMCT->pRevupCtrl)
      {
        bRetVal = (int32_t)RUC_GetNumberOfPhases(pMCT->pRevupCtrl);
      }
      else
      {
        bRetVal = (uint32_t) 0;
      }
    }
    break;

    case MC_PROTOCOL_REG_I_A:
    {
      bRetVal = MCI_GetIab(pMCI).a;
    }
    break;

    case MC_PROTOCOL_REG_I_B:
    {
      bRetVal = MCI_GetIab(pMCI).b;
    }
    break;

    case MC_PROTOCOL_REG_I_ALPHA:
    {
      bRetVal = MCI_GetIalphabeta(pMCI).alpha;
    }
    break;

    case MC_PROTOCOL_REG_I_BETA:
    {
      bRetVal = MCI_GetIalphabeta(pMCI).beta;
    }
    break;

    case MC_PROTOCOL_REG_I_Q_REF:
    {
      bRetVal = MCI_GetIqdref(pMCI).q;
    }
    break;

    case MC_PROTOCOL_REG_I_D_REF:
    {
      bRetVal = MCI_GetIqdref(pMCI).d;
    }
    break;

    case MC_PROTOCOL_REG_V_Q:
    {
      bRetVal = MCI_GetVqd(pMCI).q;
    }
    break;

    case MC_PROTOCOL_REG_V_D:
    {
      bRetVal = MCI_GetVqd(pMCI).d;
    }
    break;

   case MC_PROTOCOL_REG_V_ALPHA:
    {
      bRetVal = MCI_GetValphabeta(pMCI).alpha;
    }
    break;

    case MC_PROTOCOL_REG_V_BETA:
    {
      bRetVal = MCI_GetValphabeta(pMCI).beta;
    }
    break;

    default:
	{
      if ( success != (bool *) 0 )
      {
        *success = false;
      }
	}
    break;
  }
  return bRetVal;
}

/**
  * @brief  UI_ExecCmd：执行并推进串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bCmdID:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_ExecCmd(UI_Handle_t *pHandle, uint8_t bCmdID)
{
  bool retVal = true;

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  switch (bCmdID)
  {
  case MC_PROTOCOL_CMD_START_MOTOR:
    {
      /* 中文说明：启动或使能电机。 */
      MCI_StartMotor(pMCI);
    }
    break;

  case MC_PROTOCOL_CMD_STOP_MOTOR:
  case MC_PROTOCOL_CMD_SC_STOP:
    {
      /* 中文说明：停止或禁用电机。 */
      MCI_StopMotor(pMCI);
    }
    break;

  case MC_PROTOCOL_CMD_STOP_RAMP:
    {
      if (MCI_GetSTMState(pMCI) == RUN)
      {
        MCI_StopRamp(pMCI);
      }
    }
    break;

  case MC_PROTOCOL_CMD_PING:
    {
	  /* 中文说明：处理并更新相关数据和控制状态。 */
    }
    break;

  case MC_PROTOCOL_CMD_START_STOP:
    {
      /* 中文说明：启动或使能状态机状态。 */
      if (MCI_GetSTMState(pMCI) == IDLE)
      {
        MCI_StartMotor(pMCI);
      }
      else
      {
        MCI_StopMotor(pMCI);
      }
    }
    break;

  case MC_PROTOCOL_CMD_RESET:
    {
	  /* 中文说明：处理并更新相关数据和控制状态。 */
    }
    break;

  case MC_PROTOCOL_CMD_FAULT_ACK:
    {
      MCI_FaultAcknowledged(pMCI);
    }
    break;

  case MC_PROTOCOL_CMD_ENCODER_ALIGN:
    {
      MCI_EncoderAlign(pMCI);
    }
    break;

  case MC_PROTOCOL_CMD_IQDREF_CLEAR:
    {
      MCI_Clear_Iqdref(pMCI);
    }
    break;

  default:
    {
    retVal = false;
	}
    break;
  }
  return retVal;
}

/**
  * @brief  UI_ExecSpeedRamp：设置速度、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  wFinalMecSpeedUnit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_ExecSpeedRamp(UI_Handle_t *pHandle, int32_t wFinalMecSpeedUnit, uint16_t hDurationms)
{
  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  /* 中文说明：处理并更新斜坡。 */
  MCI_ExecSpeedRamp(pMCI,(int16_t)((wFinalMecSpeedUnit*SPEED_UNIT)/_RPM),hDurationms);
  return true;
}
/**
  * @brief  UI_ExecTorqueRamp：设置转矩、斜坡，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hTargetFinal:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_ExecTorqueRamp(UI_Handle_t *pHandle, int16_t hTargetFinal, uint16_t hDurationms)
{

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];

  /* 中文说明：处理并更新斜坡。 */
  MCI_ExecTorqueRamp(pMCI,hTargetFinal,hDurationms);
  return true;
}

/**
  * @brief  UI_GetRevupData：读取并返回相电流、速度、转矩、电机，保持对象状态和控制流程一致。
  * @param  pHandle  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bStage  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pDurationms  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pFinalMecSpeedUnit  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pFinalTorque  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_GetRevupData(UI_Handle_t *pHandle, uint8_t bStage, uint16_t* pDurationms,
                     int16_t* pFinalMecSpeedUnit, int16_t* pFinalTorque )
{
  bool hRetVal = true;

  RevUpCtrl_Handle_t *pRevupCtrl = pHandle->pMCT[pHandle->bSelectedDrive]->pRevupCtrl;
  if (pRevupCtrl)
  {
    *pDurationms = RUC_GetPhaseDurationms(pRevupCtrl, bStage);
    *pFinalMecSpeedUnit = RUC_GetPhaseFinalMecSpeedUnit(pRevupCtrl, bStage);
    *pFinalTorque = RUC_GetPhaseFinalTorque(pRevupCtrl, bStage);
  }
  else
  {
    hRetVal = false;
  }
  return hRetVal;
}

/**
  * @brief  UI_SetRevupData：设置相电流、速度、转矩、电机，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bStage:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hDurationms:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalMecSpeedUnit:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hFinalTorque:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_SetRevupData(UI_Handle_t *pHandle, uint8_t bStage, uint16_t hDurationms,
                     int16_t hFinalMecSpeedUnit, int16_t hFinalTorque )
{
  RevUpCtrl_Handle_t *pRevupCtrl = pHandle->pMCT[pHandle->bSelectedDrive]->pRevupCtrl;
  RUC_SetPhaseDurationms(pRevupCtrl, bStage, hDurationms);
  RUC_SetPhaseFinalMecSpeedUnit(pRevupCtrl, bStage, hFinalMecSpeedUnit);
  RUC_SetPhaseFinalTorque(pRevupCtrl, bStage, hFinalTorque);
  return true;
}

/**
  * @brief  UI_SetCurrentReferences：设置相电流、串口通信帧，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hIqRef:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hIdRef:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  * @note   调用时应满足该模块的状态、时序和量纲约束。
  */
__WEAK void UI_SetCurrentReferences(UI_Handle_t *pHandle, int16_t hIqRef, int16_t hIdRef)
{

  MCI_Handle_t * pMCI = pHandle->pMCI[pHandle->bSelectedDrive];
  qd_t currComp;
  currComp.q = hIqRef;
  currComp.d = hIdRef;
  MCI_SetCurrentReferences(pMCI,currComp);
}

/**
  * @brief  UI_GetMPInfo：读取并返回相关数据和控制状态，保持对象状态和控制流程一致。
  * @param  stepList:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  pMPInfo:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK bool UI_GetMPInfo(pMPInfo_t stepList, pMPInfo_t pMPInfo)
{
    return false;
}

/**
  * @brief  UI_DACInit：初始化调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void UI_DACInit(UI_Handle_t *pHandle)
{
  if (pHandle->pFct_DACInit)
  {
	  pHandle->pFct_DACInit(pHandle);
  }
}

/**
  * @brief  UI_DACExec：执行并推进调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_DACExec(UI_Handle_t *pHandle)
{
  if (pHandle->pFct_DACExec)
  {
    pHandle->pFct_DACExec(pHandle);
  }
}

/**
  * @brief  UI_SetDAC：设置串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bVariable:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
void UI_SetDAC(UI_Handle_t *pHandle, DAC_Channel_t bChannel,
                         MC_Protocol_REG_t bVariable)
{
  if (pHandle->pFctDACSetChannelConfig)
  {
	  pHandle->pFctDACSetChannelConfig(pHandle, bChannel, bVariable);
  }
}

/**
  * @brief  UI_GetDAC：读取并返回相电流、串口通信帧、调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bChannel:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK MC_Protocol_REG_t UI_GetDAC(UI_Handle_t *pHandle, DAC_Channel_t bChannel)
{
  MC_Protocol_REG_t retVal = MC_PROTOCOL_REG_UNDEFINED;
  if (pHandle->pFctDACGetChannelConfig)
  {
    retVal = pHandle->pFctDACGetChannelConfig(pHandle, bChannel);
  }
  return retVal;
}

/**
  * @brief  UI_SetUserDAC：设置调试DAC，保持对象状态和控制流程一致。
  * @param  pHandle:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  bUserChNumber:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @param  hValue:  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
__WEAK void UI_SetUserDAC(UI_Handle_t *pHandle, DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  if (pHandle->pFctDACSetUserChannelValue)
  {
	  pHandle->pFctDACSetUserChannelValue(pHandle, bUserChNumber, hValue);
  }
}
