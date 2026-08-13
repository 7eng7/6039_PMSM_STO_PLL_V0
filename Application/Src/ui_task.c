/**
 * @file ui_task.c
 * @brief 电机控制用户界面与串行监视任务。
 *
 * 中文解读：初始化通信对象、DAC监视通道和周期性UI处理，将底层串口帧协议连接到
 * motor_control_protocol.c。它运行在非实时任务中，不应影响电流环时序。
 */
/*
* 版权所有。允许在保留原版权声明、条件和免责声明的前提下，以源代码或二进制形式重新发布和使用。
* 本软件按“现状”提供，不作任何明示或暗示担保；版权方及贡献者不承担因使用本软件产生的损失。
*/

#include "ui_task.h"
#include "mc_config.h"
#include "parameters_conversion.h"
#include "mc_api.h"

#define OPT_DACX  0x20 /* 中文说明：初始化调试DAC。 */

DAC_UI_Handle_t * pDAC = MC_NULL;
extern DAC_UI_Handle_t DAC_UI_Params;

MCP_Handle_t * pMCP = MC_NULL;
MCP_Handle_t MCP_UI_Params;

static volatile uint16_t  bUITaskCounter;
static volatile uint16_t  bCOMTimeoutCounter;
static volatile uint16_t  bCOMATRTimeCounter = SERIALCOM_ATR_TIME_TICKS;

void UI_TaskInit( uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t* pMCIList[],
                  MCT_Handle_t* pMCTList[],const char* s_fwVer )
{
      pDAC = &DAC_UI_Params;
      pDAC->_Super = UI_Params;

      UI_Init( &pDAC->_Super, bMCNum, pMCIList, pMCTList, pUICfg ); /* 中文说明：处理并更新相关数据和控制状态。 */
      UI_DACInit( &pDAC->_Super ); /* 中文说明：处理并更新调试DAC。 */
      UI_SetDAC( &pDAC->_Super, DAC_CH0, MC_PROTOCOL_REG_V_ALPHA );
      UI_SetDAC( &pDAC->_Super, DAC_CH1, MC_PROTOCOL_REG_OBS_EL_ANGLE );
	
//      UI_SetDAC( &pDAC->_Super, DAC_CH0, MC_PROTOCOL_REG_MEAS_EL_ANGLE );
//      UI_SetDAC( &pDAC->_Super, DAC_CH1, MC_PROTOCOL_REG_V_ALPHA );	
	

    pMCP = &MCP_UI_Params;
    pMCP->_Super = UI_Params;

    UFCP_Init( & pUSART );
    MCP_Init(pMCP, (FCP_Handle_t *) & pUSART, & UFCP_Send, & UFCP_Receive, & UFCP_AbortReceive, pDAC, s_fwVer);
    UI_Init( &pMCP->_Super, bMCNum, pMCIList, pMCTList, pUICfg ); /* 中文说明：初始化相关数据和控制状态。 */

}

__WEAK void UI_Scheduler(void)
{
  if(bUITaskCounter > 0u)
  {
    bUITaskCounter--;
  }

  if(bCOMTimeoutCounter > 1u)
  {
    bCOMTimeoutCounter--;
  }

  if(bCOMATRTimeCounter > 1u)
  {
    bCOMATRTimeCounter--;
  }
}

__WEAK void UI_DACUpdate(uint8_t bMotorNbr)
{
  if (UI_GetSelectedMC(&pDAC->_Super) == bMotorNbr)
  {
    UI_DACExec(&pDAC->_Super); /* 中文说明：设置调试DAC。 */
  }
}

__WEAK void MC_SetDAC(DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable)
{
  UI_SetDAC(&pDAC->_Super, bChannel, bVariable);
}

__WEAK void MC_SetUserDAC(DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
  UI_SetUserDAC(&pDAC->_Super, bUserChNumber, hValue);
}

__WEAK UI_Handle_t * GetDAC(void)
{
  return &pDAC->_Super;
}

__WEAK MCP_Handle_t * GetMCP(void)
{
  return pMCP;
}

__WEAK bool UI_IdleTimeHasElapsed(void)
{
  bool retVal = false;
  if (bUITaskCounter == 0u)
  {
    retVal = true;
  }
  return (retVal);
}

__WEAK void UI_SetIdleTime(uint16_t SysTickCount)
{
  bUITaskCounter = SysTickCount;
}

__WEAK bool UI_SerialCommunicationTimeOutHasElapsed(void)
{
  bool retVal = false;
  if (bCOMTimeoutCounter == 1u)
  {
    bCOMTimeoutCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

__WEAK bool UI_SerialCommunicationATRTimeHasElapsed(void)
{
  bool retVal = false;
  if (bCOMATRTimeCounter == 1u)
  {
    bCOMATRTimeCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

__WEAK void UI_SerialCommunicationTimeOutStop(void)
{
  bCOMTimeoutCounter = 0u;
}

__WEAK void UI_SerialCommunicationTimeOutStart(void)
{
  bCOMTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}

__WEAK void UI_HandleStartStopButton_cb (void)
{
  if (MC_GetSTMStateMotor1() == IDLE)
  {
    /* 中文说明：处理并更新斜坡、电机、参数和增益。 */
    MC_StartMotor1();
  }
  else
  {
    MC_StopMotor1();
  }
}


