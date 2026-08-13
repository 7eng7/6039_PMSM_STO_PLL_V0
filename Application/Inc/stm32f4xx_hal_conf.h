/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   本文件实现其所属电机控制模块的接口、状态维护和算法流程。
  * @attention
  *          本软件依据原版权方许可证提供，使用、复制和再发布应遵守该许可证；
  *          软件按“现状”提供，不作任何明示或暗示担保。
  ******************************************************************************
  */

/* 中文说明：处理并更新相关数据和控制状态。 */
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

/* 中文解读：MCSDK遗留的STM32 HAL兼容配置头。ET6039工程保留它主要为满足旧版MCSDK包含关系，
 * 实际外设初始化以ET6039驱动和init_config.c为准。 */

#ifdef __cplusplus
 extern "C" {
#endif

/* 导出类型 */
/* 导出常量 */

/* ########################## 模块选择 ###################################### */
/**
  * @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
#define HAL_MODULE_ENABLED

  #define HAL_ADC_MODULE_ENABLED
/* #define HAL_CRYP_MODULE_ENABLED   */
/* #define HAL_CAN_MODULE_ENABLED   */
/* #define HAL_CRC_MODULE_ENABLED   */
/* #define HAL_CAN_LEGACY_MODULE_ENABLED   */
/* #define HAL_CRYP_MODULE_ENABLED   */
#define HAL_DAC_MODULE_ENABLED
/* #define HAL_DCMI_MODULE_ENABLED   */
/* #define HAL_DMA2D_MODULE_ENABLED   */
/* #define HAL_ETH_MODULE_ENABLED   */
/* #define HAL_NAND_MODULE_ENABLED   */
/* #define HAL_NOR_MODULE_ENABLED   */
/* #define HAL_PCCARD_MODULE_ENABLED   */
/* #define HAL_SRAM_MODULE_ENABLED   */
/* #define HAL_SDRAM_MODULE_ENABLED   */
/* #define HAL_HASH_MODULE_ENABLED   */
/* #define HAL_I2C_MODULE_ENABLED   */
/* #define HAL_I2S_MODULE_ENABLED   */
/* #define HAL_IWDG_MODULE_ENABLED   */
/* #define HAL_LTDC_MODULE_ENABLED   */
/* #define HAL_RNG_MODULE_ENABLED   */
/* #define HAL_RTC_MODULE_ENABLED   */
/* #define HAL_SAI_MODULE_ENABLED   */
/* #define HAL_SD_MODULE_ENABLED   */
/* #define HAL_MMC_MODULE_ENABLED   */
/* #define HAL_SPI_MODULE_ENABLED   */
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
/* #define HAL_USART_MODULE_ENABLED   */
/* #define HAL_IRDA_MODULE_ENABLED   */
/* #define HAL_SMARTCARD_MODULE_ENABLED   */
/* #define HAL_SMBUS_MODULE_ENABLED   */
/* #define HAL_WWDG_MODULE_ENABLED   */
/* #define HAL_PCD_MODULE_ENABLED   */
/* #define HAL_HCD_MODULE_ENABLED   */
/* #define HAL_DSI_MODULE_ENABLED   */
/* #define HAL_QSPI_MODULE_ENABLED   */
/* #define HAL_QSPI_MODULE_ENABLED   */
/* #define HAL_CEC_MODULE_ENABLED   */
/* #define HAL_FMPI2C_MODULE_ENABLED   */
/* #define HAL_SPDIFRX_MODULE_ENABLED   */
/* #define HAL_DFSDM_MODULE_ENABLED   */
/* #define HAL_LPTIM_MODULE_ENABLED   */
#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* ########################## HSE/HSI数值适配 ############################### */
/**
  * @brief  当前接口：计算并更新速度、观测器、反电势和PLL，保持对象状态和控制流程一致。
  */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U) /* 中文说明：处理并更新相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)   /* 中文说明：启动或使能相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/**
  * @brief  当前接口：计算并更新速度、观测器、反电势和PLL，保持对象状态和控制流程一致。
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /* 中文说明：处理并更新相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/**
  * @brief  当前接口：处理并更新速度，保持对象状态和控制流程一致。
  */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  ((uint32_t)32000U)       /* 中文说明：处理并更新相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */                      /* 中文说明：处理并更新电压、速度、温度。 */
/**
  * @brief  当前接口：处理并更新速度，保持对象状态和控制流程一致。
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)32768U)    /* 中文说明：处理并更新速度。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)   /* 中文说明：启动或使能相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/**
  * @brief  当前接口：计算并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    ((uint32_t)12288000U) /* 中文说明：处理并更新相关数据和控制状态。 */
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 中文说明：处理并更新相关数据和控制状态。 */

/* ########################### 系统配置 ##################################### */
/**
  * @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
#define  VDD_VALUE		      ((uint32_t)3300U) /* 中文说明：处理并更新相关数据和控制状态。 */
#define  TICK_INT_PRIORITY            ((uint32_t)4U)   /* 中文说明：处理中断并更新中断。 */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* 中文说明：注册ADC采样。 */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_CRYP_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* 中文说明：注册调试DAC。 */
#define  USE_HAL_DCMI_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_DFSDM_REGISTER_CALLBACKS       0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_DMA2D_REGISTER_CALLBACKS       0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_DSI_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_HASH_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS       0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_LTDC_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_QSPI_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_RNG_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SAI_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SDRAM_REGISTER_CALLBACKS       0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS       0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* 中文说明：注册相关数据和控制状态。 */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* 中文说明：注册串口通信帧。 */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* 中文说明：注册串口通信帧。 */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* 中文说明：注册相关数据和控制状态。 */

/* ########################## 断言功能选择 ################################## */
/**
  * @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */
/* #define USE_FULL_ASSERT    1U */

/* ################## 以太网外设配置 ######################################## */

/* 中文说明：处理并更新相关数据和控制状态。 */

/* 中文说明：处理并更新相关数据和控制状态。 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* 中文说明：处理并更新缓冲区。 */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* 中文说明：处理并更新缓冲区。 */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* 中文说明：处理并更新缓冲区。 */
#define ETH_RXBUFNB                    ((uint32_t)4U)       /* 中文说明：处理并更新缓冲区。 */
#define ETH_TXBUFNB                    ((uint32_t)4U)       /* 中文说明：处理并更新缓冲区。 */

/* 中文说明：处理并更新相关数据和控制状态。 */

/* 中文说明：处理并更新相关数据和控制状态。 */
#define DP83848_PHY_ADDRESS           0x01U
/* 中文说明：清除并复位中断。 */
#define PHY_RESET_DELAY                 ((uint32_t)0x000000FFU)
/* 中文说明：处理并更新相关数据和控制状态。 */
#define PHY_CONFIG_DELAY                ((uint32_t)0x00000FFFU)

#define PHY_READ_TO                     ((uint32_t)0x0000FFFFU)
#define PHY_WRITE_TO                    ((uint32_t)0x0000FFFFU)

/* 中文说明：处理并更新相关数据和控制状态。 */

#define PHY_BCR                         ((uint16_t)0x0000U)    /* 中文说明：注册相关数据和控制状态。 */
#define PHY_BSR                         ((uint16_t)0x0001U)    /* 中文说明：注册相关数据和控制状态。 */

#define PHY_RESET                       ((uint16_t)0x8000U)  /* 中文说明：清除并复位相关数据和控制状态。 */
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  /* 中文说明：设置相关数据和控制状态。 */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  /* 中文说明：设置相关数据和控制状态。 */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  /* 中文说明：设置相关数据和控制状态。 */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  /* 中文说明：设置相关数据和控制状态。 */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  /* 中文说明：设置相关数据和控制状态。 */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  /* 中文说明：启动或使能相关数据和控制状态。 */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  /* 中文说明：处理并更新相关数据和控制状态。 */
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  /* 中文说明：设置功率。 */
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  /* 中文说明：处理并更新相关数据和控制状态。 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  /* 中文说明：处理并更新相关数据和控制状态。 */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  /* 中文说明：处理并更新相关数据和控制状态。 */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  /* 中文说明：处理并更新相关数据和控制状态。 */

/* 中文说明：处理并更新相关数据和控制状态。 */
#define PHY_SR                          ((uint16_t)0x10U)    /* 中文说明：注册相关数据和控制状态。 */

#define PHY_SPEED_STATUS                ((uint16_t)0x0002U)  /* 中文说明：处理并更新速度。 */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004U)  /* 中文说明：处理并更新相关数据和控制状态。 */

/* ################## SPI外设配置 ########################################### */

/* 中文说明：处理并更新相关数据和控制状态。 */

#define USE_SPI_CRC                     0U

/* 头文件 */
/**
  * @brief  当前接口：处理并更新相关数据和控制状态，保持对象状态和控制流程一致。
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* 中文说明：处理并更新数字输出。 */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* 中文说明：处理并更新ADC采样。 */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_can_legacy.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* 中文说明：处理并更新调试DAC。 */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_smbus.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* 中文说明：处理并更新串口通信帧。 */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* 中文说明：处理并更新串口通信帧。 */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/* 导出宏 */
#ifdef  USE_FULL_ASSERT
/**
  * @brief  assert_failed：检查并判断参数和增益，保持对象状态和控制流程一致。
  * @param  expr  输入参数，提供该接口所需的对象句柄或控制数据。
  * @retval 接口执行结果、状态标志或计算值；具体类型由函数声明限定。
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* 导出函数 */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* 中文说明：处理并更新相关数据和控制状态。 */

#ifdef __cplusplus
}
#endif

#endif /* 中文说明：处理并更新相关数据和控制状态。 */

/************************ （C）版权所有，文件结束 ************************/
