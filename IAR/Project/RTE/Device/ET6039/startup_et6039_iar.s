;******************** (C) COPYRIGHT 2023 ETMCU ********************
;* File Name          : startup_et6039.s
;* Author             : MCD Application Team
;* Version            : V1.1.2
;* Date               : 02-Nov-2022
;* Description        : ET6001 devices vector table for MDK-ARM toolchain. 
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM7 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* <<< Use Configuration Wizard in Context Menu >>>   
;*******************************************************************************
; 
;* Redistribution and use in source and binary forms, with or without modification,
;* are permitted provided that the following conditions are met:
;*   1. Redistributions of source code must retain the above copyright notice,
;*      this list of conditions and the following disclaimer.
;*   2. Redistributions in binary form must reproduce the above copyright notice,
;*      this list of conditions and the following disclaimer in the documentation
;*      and/or other materials provided with the distribution.
;*   3. Neither the name of ETMCU nor the names of its contributors
;*      may be used to endorse or promote products derived from this software
;*      without specific prior written permission.
;*
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; 
;*******************************************************************************

; Copyright 2024 Watech Electronics
; All rights reserved.
;
; SPDX-License-Identifier: BSD-3-Clause
;

; Vector Table Mapped to Address 0 at Reset

                MODULE  ?cstartup

                ;; Forward declaration of sections.
                SECTION CSTACK:DATA:NOROOT(3)

                SECTION .intvec:CODE:NOROOT(2)

                EXTERN  __iar_program_start
                EXTERN  SystemInit
                PUBLIC   __vector_table
//                PUBLIC   __vector_table_0x1c
                PUBLIC   __Vectors
                PUBLIC   __Vectors_End
                PUBLIC   __Vectors_Size

                DATA

__vector_table
                DCD     sfe(CSTACK)                         ; Top of Stack
                DCD     Reset_Handler                       ; Reset Handler
                DCD     NMI_Handler                         ; NMI Handler
                DCD     HardFault_Handler                   ; Hard Fault Handler
                DCD     MemManage_Handler                   ; MPU Fault Handler
                DCD     BusFault_Handler                    ; Bus Fault Handler
                DCD     UsageFault_Handler                  ; Usage Fault Handler
//__vector_table_0x1c
                DCD     0                                   ; Reserved
                DCD     0                                   ; Reserved
                DCD     0                                   ; Reserved
                DCD     0                                   ; Reserved
                DCD     SVC_Handler                         ; SVCall Handler
                DCD     DebugMon_Handler                    ; Debug Monitor Handler
                DCD     0                                   ; Reserved
                DCD     OS_CPU_PendSVHandler                ; PendSV Handler
                DCD     SysTick_Handler                     ; SysTick Handler
                ; External Interrupts
                DCD     0
                DCD     AHBDMAC0_TFR_IRQHandler
                DCD     AHBDMAC0_BLOCK_IRQHandler
                DCD     AHBDMAC0_SRCTRAN_IRQHandler
                DCD     AHBDMAC0_DSTTRAN_IRQHandler
                DCD     AHBDMAC0_ERROR_IRQHandler
                DCD     AHBDMAC0_COMBINED_IRQHandler
                DCD     AHBDMAC1_TFR_IRQHandler
                DCD     AHBDMAC1_BLOCK_IRQHandler
                DCD     AHBDMAC1_SRCTRAN_IRQHandler
                DCD     AHBDMAC1_DSTTRAN_IRQHandler
                DCD     AHBDMAC1_ERROR_IRQHandler
                DCD     AHBDMAC1_COMBINED_IRQHandler
                DCD     WDG0_IRQHandler
                DCD     WDG1_IRQHandler
                DCD     UART0_IRQHandler
                DCD     UART1_IRQHandler
                DCD     I2C0_IC_IRQHandler
                DCD     I2C1_IC_IRQHandler
                DCD     SPIM0_SSI_IRQHandler
                DCD     SPIM1_SSI_IRQHandler
                DCD     SPIM2_SSI_IRQHandler
                DCD     SPIS0_SSI_IRQHandler
                DCD     CAN0_INT0_IRQHandler
                DCD     CAN0_INT1_IRQHandler
                DCD     CAN0_DMU_IRQHandler
                DCD     CAN1_INT0_IRQHandler
                DCD     CAN1_INT1_IRQHandler
                DCD     CAN1_DMU_IRQHandler
                DCD     MTTCAN_ECC_IRQHandler
                DCD     GPIOA_COMBINT_IRQHandler
                DCD     GPIOB_COMBINT_IRQHandler
                DCD     GPIOC_COMBINT_IRQHandler
                DCD     GPIOD_COMBINT_IRQHandler
                DCD     GPIOE_COMBINT_IRQHandler
                DCD     GPIOF_COMBINT_IRQHandler
                DCD     PBUS1_TIMEOUT_IRQHandler
                DCD     PBUS3_TIMEOUT_IRQHandler
                DCD     HBUS1_TIMEOUT_IRQHandler
                DCD     HBUS2_TIMEOUT_IRQHandler
                DCD     HBUS3_TIMEOUT_IRQHandler
                DCD     GPIOE_GPIOINT0_IRQHandler
                DCD     GPIOE_GPIOINT1_IRQHandler
                DCD     GPIOE_GPIOINT2_IRQHandler
                DCD     GPIOE_GPIOINT3_IRQHandler
                DCD     GPIOE_GPIOINT4_IRQHandler
                DCD     GPIOE_GPIOINT5_IRQHandler
                DCD     GPIOE_GPIOINT6_IRQHandler
                DCD     GPIOE_GPIOINT7_IRQHandler
                DCD     GPIOE_GPIOINT8_IRQHandler
                DCD     GPIOE_GPIOINT9_IRQHandler
                DCD     GPIOE_GPIOINT10_IRQHandler
                DCD     GPIOE_GPIOINT11_IRQHandler
                DCD     GPIOE_GPIOINT12_IRQHandler
                DCD     GPIOE_GPIOINT13_IRQHandler
                DCD     GPIOE_GPIOINT14_IRQHandler
                DCD     GPIOE_GPIOINT15_IRQHandler
                DCD     EFC0_FLASH_IRQHandler
                DCD     EFC1_FLASH_IRQHandler
                DCD     EFC2_FLASH_IRQHandler
                DCD     PBUS2_TIMEOUT_IRQHandler
                DCD     SRAM0_ECC_SERR_IRQHandler
                DCD     SRAM0_ECC_MERR_IRQHandler
                DCD     SRAM1_ECC_SERR_IRQHandler
                DCD     SRAM1_ECC_MERR_IRQHandler
                DCD     CORE2PERI_TIMEOUT_IRQHandler
                DCD     PERI2CORE_TIMEOUT_IRQHandler
                DCD     NONE_IRQHandler1
                DCD     NONE_IRQHandler2
                DCD     NONE_IRQHandler3
                DCD     MISC_M7_IRQHandler
                DCD     CMPC_INTR0_IRQHandler
                DCD     CMPC_INTR1_IRQHandler
                DCD     CMPC_INTR2_IRQHandler
                DCD     CMPC_INTR3_IRQHandler
                DCD     DACC_INTR0_IRQHandler
                DCD     DACC_INTR1_IRQHandler
                DCD     SARC_INTR0_IRQHandler
                DCD     SARC_INTR1_IRQHandler
                DCD     SARC_INTR2_IRQHandler
                DCD     EQEP_INTR0_IRQHandler
                DCD     EQEP_INTR1_IRQHandler
                DCD     SDFM_ERR_INTR0_IRQHandler
                DCD     SDFM_DR_INTR0_IRQHandler
                DCD     SDFM_DR_INTR1_IRQHandler
                DCD     SDFM_DR_INTR2_IRQHandler
                DCD     SDFM_DR_INTR3_IRQHandler
                DCD     SDFM_ERR_INTR1_IRQHandler
                DCD     SDFM_DR_INTR4_IRQHandler
                DCD     SDFM_DR_INTR5_IRQHandler
                DCD     SDFM_DR_INTR6_IRQHandler
                DCD     SDFM_DR_INTR7_IRQHandler
                DCD     CRC_IRQHandler
                DCD     CORDIC_IRQHandler
                DCD     FMAC_IRQHandler
                DCD     STM_INTR0_IRQHandler
                DCD     STM_INTR1_IRQHandler
                DCD     AES_IRQHandler
                DCD     ETIM_INTR0_IRQHandler
                DCD     ETIM_INTR1_IRQHandler
                DCD     ETIM_INTR2_IRQHandler
                DCD     ETIM_INTR3_IRQHandler
                DCD     ETIM_INTR4_IRQHandler
                DCD     ETIM_INTR5_IRQHandler
                DCD     ETIM_INTR6_IRQHandler
                DCD     ETIM_INTR7_IRQHandler
                DCD     ETIM_INTR8_IRQHandler
                DCD     ETIM_INTR9_IRQHandler
                DCD     ETIM_INTR10_IRQHandler
                DCD     ETIM_INTR11_IRQHandler
                DCD     ETIM_INTR12_IRQHandler
                DCD     ETIM_INTR13_IRQHandler
                DCD     EPWM_EPWMCOM_INTR0_IRQHandler
                DCD     EPWM_EPWMCOM_INTR1_IRQHandler
                DCD     EPWM_EPWMCOM_INTR2_IRQHandler
                DCD     EPWM_EPWMCOM_INTR3_IRQHandler
                DCD     EPWM_EPWMCOM_INTR4_IRQHandler
                DCD     EPWM_EPWMCOM_INTR5_IRQHandler
                DCD     EPWM_EPWMCOM_INTR6_IRQHandler
                DCD     EPWM_EPWMCOM_INTR7_IRQHandler
                DCD     EPWM_EPWMCOM_INTR8_IRQHandler
                DCD     EPWM_EPWMCOM_INTR9_IRQHandler
                DCD     EPWM_EPWMCOM_INTR10_IRQHandler
                DCD     EPWM_EPWMCOM_INTR11_IRQHandler
                DCD     SYSC_IRQHandler
                DCD     XBAR_INTR0_IRQHandler
                DCD     XBAR_INTR1_IRQHandler
                DCD     XBAR_INTR2_IRQHandler
                DCD     XBAR_INTR3_IRQHandler
                DCD     XBAR_INTR4_IRQHandler
__Vectors_End

__Vectors       EQU      __vector_table
__Vectors_Size  EQU      __Vectors_End - __Vectors


                THUMB


; Reset handler
                PUBWEAK Reset_Handler
                SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
                CPSID    I                                   ; Mask interrupts
VTOR            EQU      0xE000ED08
                LDR      R0, =VTOR
                LDR      R1, =__Vectors                      ; Set VTOR
                STR      R1, [R0]
                LDR      R2, [R1]
                MSR      MSP, R2                             ; Set MSP
                LDR      R0, =SystemInit
                BLX      R0
                CPSIE    I                                   ; Unmask interrupts
                LDR      R0, =__iar_program_start
                BX       R0

;
; Macro to define default exception/interrupt handlers.
; Default handler are weak symbols calling driver handlers.
; Users can re-define the symbols to implement their own exception/interrupt handlers.
;
Set_Default_Handler MACRO Handler_Name, DriverHandler_Name
                PUBWEAK Handler_Name
                PUBWEAK DriverHandler_Name
                SECTION .text:CODE:REORDER:NOROOT(2)
Handler_Name
                LDR      R0, =DriverHandler_Name
                BX       R0
                ENDM

; Default exception/interrupt handler
                Set_Default_Handler  NMI_Handler, NMI_DriverHandler
                Set_Default_Handler  HardFault_Handler, HardFault_DriverHandler
                Set_Default_Handler  MemManage_Handler, MemManage_DriverHandler
                Set_Default_Handler  BusFault_Handler, BusFault_DriverHandler
                Set_Default_Handler  UsageFault_Handler, UsageFault_DriverHandler
                Set_Default_Handler  SVC_Handler, SVC_DriverHandler
                Set_Default_Handler  DebugMon_Handler, DebugMon_DriverHandler
                Set_Default_Handler  OS_CPU_PendSVHandler, OS_CPU_PendSV_DriverHandler
                Set_Default_Handler  SysTick_Handler, SysTick_DriverHandler

                Set_Default_Handler  AHBDMAC0_TFR_IRQHandler, AHBDMAC0_TFR_DriverHandler
                Set_Default_Handler  AHBDMAC0_BLOCK_IRQHandler, AHBDMAC0_BLOCK_DriverHandler
                Set_Default_Handler  AHBDMAC0_SRCTRAN_IRQHandler, AHBDMAC0_SRCTRAN_DriverHandler
                Set_Default_Handler  AHBDMAC0_DSTTRAN_IRQHandler, AHBDMAC0_DSTTRAN_DriverHandler
                Set_Default_Handler  AHBDMAC0_ERROR_IRQHandler, AHBDMAC0_ERROR_DriverHandler
                Set_Default_Handler  AHBDMAC0_COMBINED_IRQHandler, AHBDMAC0_COMBINED_DriverHandler
                Set_Default_Handler  AHBDMAC1_TFR_IRQHandler, AHBDMAC1_TFR_DriverHandler
                Set_Default_Handler  AHBDMAC1_BLOCK_IRQHandler, AHBDMAC1_BLOCK_DriverHandler
                Set_Default_Handler  AHBDMAC1_SRCTRAN_IRQHandler, AHBDMAC1_SRCTRAN_DriverHandler
                Set_Default_Handler  AHBDMAC1_DSTTRAN_IRQHandler, AHBDMAC1_DSTTRAN_DriverHandler
                Set_Default_Handler  AHBDMAC1_ERROR_IRQHandler, AHBDMAC1_ERROR_DriverHandler
                Set_Default_Handler  AHBDMAC1_COMBINED_IRQHandler, AHBDMAC1_COMBINED_DriverHandler
                Set_Default_Handler  WDG0_IRQHandler, WDG0_DriverHandler
                Set_Default_Handler  WDG1_IRQHandler, WDG1_DriverHandler
                Set_Default_Handler  UART0_IRQHandler, UART0_DriverHandler
                Set_Default_Handler  UART1_IRQHandler, UART1_DriverHandler
                Set_Default_Handler  I2C0_IC_IRQHandler, I2C0_IC_DriverHandler
                Set_Default_Handler  I2C1_IC_IRQHandler, I2C1_IC_DriverHandler
                Set_Default_Handler  SPIM0_SSI_IRQHandler, SPIM0_SSI_DriverHandler
                Set_Default_Handler  SPIM1_SSI_IRQHandler, SPIM1_SSI_DriverHandler
                Set_Default_Handler  SPIM2_SSI_IRQHandler, SPIM2_SSI_DriverHandler
                Set_Default_Handler  SPIS0_SSI_IRQHandler, SPIS0_SSI_DriverHandler
                Set_Default_Handler  CAN0_INT0_IRQHandler, CAN0_INT0_DriverHandler
                Set_Default_Handler  CAN0_INT1_IRQHandler, CAN0_INT1_DriverHandler
                Set_Default_Handler  CAN0_DMU_IRQHandler, CAN0_DMU_DriverHandler
                Set_Default_Handler  CAN1_INT0_IRQHandler, CAN1_INT0_DriverHandler
                Set_Default_Handler  CAN1_INT1_IRQHandler, CAN1_INT1_DriverHandler
                Set_Default_Handler  CAN1_DMU_IRQHandler, CAN1_DMU_DriverHandler
                Set_Default_Handler  MTTCAN_ECC_IRQHandler, MTTCAN_ECC_DriverHandler
                Set_Default_Handler  GPIOA_COMBINT_IRQHandler, GPIOA_COMBINT_DriverHandler
                Set_Default_Handler  GPIOB_COMBINT_IRQHandler, GPIOB_COMBINT_DriverHandler
                Set_Default_Handler  GPIOC_COMBINT_IRQHandler, GPIOC_COMBINT_DriverHandler
                Set_Default_Handler  GPIOD_COMBINT_IRQHandler, GPIOD_COMBINT_DriverHandler
                Set_Default_Handler  GPIOE_COMBINT_IRQHandler, GPIOE_COMBINT_DriverHandler
                Set_Default_Handler  GPIOF_COMBINT_IRQHandler, GPIOF_COMBINT_DriverHandler
                Set_Default_Handler  PBUS1_TIMEOUT_IRQHandler, PBUS1_TIMEOUT_DriverHandler
                Set_Default_Handler  PBUS3_TIMEOUT_IRQHandler, PBUS3_TIMEOUT_DriverHandler
                Set_Default_Handler  HBUS1_TIMEOUT_IRQHandler, HBUS1_TIMEOUT_DriverHandler
                Set_Default_Handler  HBUS2_TIMEOUT_IRQHandler, HBUS2_TIMEOUT_DriverHandler
                Set_Default_Handler  HBUS3_TIMEOUT_IRQHandler, HBUS3_TIMEOUT_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT0_IRQHandler, GPIOE_GPIOINT0_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT1_IRQHandler, GPIOE_GPIOINT1_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT2_IRQHandler, GPIOE_GPIOINT2_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT3_IRQHandler, GPIOE_GPIOINT3_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT4_IRQHandler, GPIOE_GPIOINT4_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT5_IRQHandler, GPIOE_GPIOINT5_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT6_IRQHandler, GPIOE_GPIOINT6_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT7_IRQHandler, GPIOE_GPIOINT7_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT8_IRQHandler, GPIOE_GPIOINT8_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT9_IRQHandler, GPIOE_GPIOINT9_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT10_IRQHandler, GPIOE_GPIOINT10_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT11_IRQHandler, GPIOE_GPIOINT11_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT12_IRQHandler, GPIOE_GPIOINT12_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT13_IRQHandler, GPIOE_GPIOINT13_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT14_IRQHandler, GPIOE_GPIOINT14_DriverHandler
                Set_Default_Handler  GPIOE_GPIOINT15_IRQHandler, GPIOE_GPIOINT15_DriverHandler
                Set_Default_Handler  EFC0_FLASH_IRQHandler, EFC0_FLASH_DriverHandler
                Set_Default_Handler  EFC1_FLASH_IRQHandler, EFC1_FLASH_DriverHandler
                Set_Default_Handler  EFC2_FLASH_IRQHandler, EFC2_FLASH_DriverHandler
                Set_Default_Handler  PBUS2_TIMEOUT_IRQHandler, PBUS2_TIMEOUT_DriverHandler
                Set_Default_Handler  SRAM0_ECC_SERR_IRQHandler, SRAM0_ECC_SERR_DriverHandler
                Set_Default_Handler  SRAM0_ECC_MERR_IRQHandler, SRAM0_ECC_MERR_DriverHandler
                Set_Default_Handler  SRAM1_ECC_SERR_IRQHandler, SRAM1_ECC_SERR_DriverHandler
                Set_Default_Handler  SRAM1_ECC_MERR_IRQHandler, SRAM1_ECC_MERR_DriverHandler
                Set_Default_Handler  CORE2PERI_TIMEOUT_IRQHandler, CORE2PERI_TIMEOUT_DriverHandler
                Set_Default_Handler  PERI2CORE_TIMEOUT_IRQHandler, PERI2CORE_TIMEOUT_DriverHandler
                Set_Default_Handler  NONE_IRQHandler1, NONE1_DriverHandler
                Set_Default_Handler  NONE_IRQHandler2, NONE2_DriverHandler
                Set_Default_Handler  NONE_IRQHandler3, NONE3_DriverHandler
                Set_Default_Handler  MISC_M7_IRQHandler, MISC_M7_DriverHandler
                Set_Default_Handler  CMPC_INTR0_IRQHandler, CMPC_INTR0_DriverHandler
                Set_Default_Handler  CMPC_INTR1_IRQHandler, CMPC_INTR1_DriverHandler
                Set_Default_Handler  CMPC_INTR2_IRQHandler, CMPC_INTR2_DriverHandler
                Set_Default_Handler  CMPC_INTR3_IRQHandler, CMPC_INTR3_DriverHandler
                Set_Default_Handler  DACC_INTR0_IRQHandler, DACC_INTR0_DriverHandler
                Set_Default_Handler  DACC_INTR1_IRQHandler, DACC_INTR1_DriverHandler
                Set_Default_Handler  SARC_INTR0_IRQHandler, SARC_INTR0_DriverHandler
                Set_Default_Handler  SARC_INTR1_IRQHandler, SARC_INTR1_DriverHandler
                Set_Default_Handler  SARC_INTR2_IRQHandler, SARC_INTR2_DriverHandler
                Set_Default_Handler  EQEP_INTR0_IRQHandler, EQEP_INTR0_DriverHandler
                Set_Default_Handler  EQEP_INTR1_IRQHandler, EQEP_INTR1_DriverHandler
                Set_Default_Handler  SDFM_ERR_INTR0_IRQHandler, SDFM_ERR_INTR0_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR0_IRQHandler, SDFM_DR_INTR0_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR1_IRQHandler, SDFM_DR_INTR1_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR2_IRQHandler, SDFM_DR_INTR2_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR3_IRQHandler, SDFM_DR_INTR3_DriverHandler
                Set_Default_Handler  SDFM_ERR_INTR1_IRQHandler, SDFM_ERR_INTR1_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR4_IRQHandler, SDFM_DR_INTR4_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR5_IRQHandler, SDFM_DR_INTR5_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR6_IRQHandler, SDFM_DR_INTR6_DriverHandler
                Set_Default_Handler  SDFM_DR_INTR7_IRQHandler, SDFM_DR_INTR7_DriverHandler
                Set_Default_Handler  CRC_IRQHandler, CRC_DriverHandler
                Set_Default_Handler  CORDIC_IRQHandler, CORDIC_DriverHandler
                Set_Default_Handler  FMAC_IRQHandler, FMAC_DriverHandler
                Set_Default_Handler  STM_INTR0_IRQHandler, STM_INTR0_DriverHandler
                Set_Default_Handler  STM_INTR1_IRQHandler, STM_INTR1_DriverHandler
                Set_Default_Handler  AES_IRQHandler, AES_DriverHandler
                Set_Default_Handler  ETIM_INTR0_IRQHandler, ETIM_INTR0_DriverHandler
                Set_Default_Handler  ETIM_INTR1_IRQHandler, ETIM_INTR1_DriverHandler
                Set_Default_Handler  ETIM_INTR2_IRQHandler, ETIM_INTR2_DriverHandler
                Set_Default_Handler  ETIM_INTR3_IRQHandler, ETIM_INTR3_DriverHandler
                Set_Default_Handler  ETIM_INTR4_IRQHandler, ETIM_INTR4_DriverHandler
                Set_Default_Handler  ETIM_INTR5_IRQHandler, ETIM_INTR5_DriverHandler
                Set_Default_Handler  ETIM_INTR6_IRQHandler, ETIM_INTR6_DriverHandler
                Set_Default_Handler  ETIM_INTR7_IRQHandler, ETIM_INTR7_DriverHandler
                Set_Default_Handler  ETIM_INTR8_IRQHandler, ETIM_INTR8_DriverHandler
                Set_Default_Handler  ETIM_INTR9_IRQHandler, ETIM_INTR9_DriverHandler
                Set_Default_Handler  ETIM_INTR10_IRQHandler, ETIM_INTR10_DriverHandler
                Set_Default_Handler  ETIM_INTR11_IRQHandler, ETIM_INTR11_DriverHandler
                Set_Default_Handler  ETIM_INTR12_IRQHandler, ETIM_INTR12_DriverHandler
                Set_Default_Handler  ETIM_INTR13_IRQHandler, ETIM_INTR13_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR0_IRQHandler, EPWM_EPWMCOM_INTR0_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR1_IRQHandler, EPWM_EPWMCOM_INTR1_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR2_IRQHandler, EPWM_EPWMCOM_INTR2_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR3_IRQHandler, EPWM_EPWMCOM_INTR3_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR4_IRQHandler, EPWM_EPWMCOM_INTR4_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR5_IRQHandler, EPWM_EPWMCOM_INTR5_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR6_IRQHandler, EPWM_EPWMCOM_INTR6_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR7_IRQHandler, EPWM_EPWMCOM_INTR7_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR8_IRQHandler, EPWM_EPWMCOM_INTR8_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR9_IRQHandler, EPWM_EPWMCOM_INTR9_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR10_IRQHandler, EPWM_EPWMCOM_INTR10_DriverHandler
                Set_Default_Handler  EPWM_EPWMCOM_INTR11_IRQHandler, EPWM_EPWMCOM_INTR11_DriverHandler
                Set_Default_Handler  SYSC_IRQHandler, SYSC_DriverHandler
                Set_Default_Handler  XBAR_INTR0_IRQHandler, XBAR_INTR0_DriverHandler
                Set_Default_Handler  XBAR_INTR1_IRQHandler, XBAR_INTR1_DriverHandler
                Set_Default_Handler  XBAR_INTR2_IRQHandler, XBAR_INTR2_DriverHandler
                Set_Default_Handler  XBAR_INTR3_IRQHandler, XBAR_INTR3_DriverHandler
                Set_Default_Handler  XBAR_INTR4_IRQHandler, XBAR_INTR4_DriverHandler

;
; Default exception/interrupt driver handlers are weak symbols with an endless loop,
; which will be overwritten by SDK driver handlers.
; Users can read the SDK driver handlers, 
; and learn how to write their own exception/interrupt handlers.
;

NMI_DriverHandler
HardFault_DriverHandler
MemManage_DriverHandler
BusFault_DriverHandler
UsageFault_DriverHandler
SVC_DriverHandler
DebugMon_DriverHandler
OS_CPU_PendSV_DriverHandler
SysTick_DriverHandler

AHBDMAC0_TFR_DriverHandler
AHBDMAC0_BLOCK_DriverHandler
AHBDMAC0_SRCTRAN_DriverHandler
AHBDMAC0_DSTTRAN_DriverHandler
AHBDMAC0_ERROR_DriverHandler
AHBDMAC0_COMBINED_DriverHandler
AHBDMAC1_TFR_DriverHandler
AHBDMAC1_BLOCK_DriverHandler
AHBDMAC1_SRCTRAN_DriverHandler
AHBDMAC1_DSTTRAN_DriverHandler
AHBDMAC1_ERROR_DriverHandler
AHBDMAC1_COMBINED_DriverHandler
WDG0_DriverHandler
WDG1_DriverHandler
UART0_DriverHandler
UART1_DriverHandler
I2C0_IC_DriverHandler
I2C1_IC_DriverHandler
SPIM0_SSI_DriverHandler
SPIM1_SSI_DriverHandler
SPIM2_SSI_DriverHandler
SPIS0_SSI_DriverHandler
CAN0_INT0_DriverHandler
CAN0_INT1_DriverHandler
CAN0_DMU_DriverHandler
CAN1_INT0_DriverHandler
CAN1_INT1_DriverHandler
CAN1_DMU_DriverHandler
MTTCAN_ECC_DriverHandler
GPIOA_COMBINT_DriverHandler
GPIOB_COMBINT_DriverHandler
GPIOC_COMBINT_DriverHandler
GPIOD_COMBINT_DriverHandler
GPIOE_COMBINT_DriverHandler
GPIOF_COMBINT_DriverHandler
PBUS1_TIMEOUT_DriverHandler
PBUS3_TIMEOUT_DriverHandler
HBUS1_TIMEOUT_DriverHandler
HBUS2_TIMEOUT_DriverHandler
HBUS3_TIMEOUT_DriverHandler
GPIOE_GPIOINT0_DriverHandler
GPIOE_GPIOINT1_DriverHandler
GPIOE_GPIOINT2_DriverHandler
GPIOE_GPIOINT3_DriverHandler
GPIOE_GPIOINT4_DriverHandler
GPIOE_GPIOINT5_DriverHandler
GPIOE_GPIOINT6_DriverHandler
GPIOE_GPIOINT7_DriverHandler
GPIOE_GPIOINT8_DriverHandler
GPIOE_GPIOINT9_DriverHandler
GPIOE_GPIOINT10_DriverHandler
GPIOE_GPIOINT11_DriverHandler
GPIOE_GPIOINT12_DriverHandler
GPIOE_GPIOINT13_DriverHandler
GPIOE_GPIOINT14_DriverHandler
GPIOE_GPIOINT15_DriverHandler
EFC0_FLASH_DriverHandler
EFC1_FLASH_DriverHandler
EFC2_FLASH_DriverHandler
PBUS2_TIMEOUT_DriverHandler
SRAM0_ECC_SERR_DriverHandler
SRAM0_ECC_MERR_DriverHandler
SRAM1_ECC_SERR_DriverHandler
SRAM1_ECC_MERR_DriverHandler
CORE2PERI_TIMEOUT_DriverHandler
PERI2CORE_TIMEOUT_DriverHandler
NONE1_DriverHandler
NONE2_DriverHandler
NONE3_DriverHandler
MISC_M7_DriverHandler
CMPC_INTR0_DriverHandler
CMPC_INTR1_DriverHandler
CMPC_INTR2_DriverHandler
CMPC_INTR3_DriverHandler
DACC_INTR0_DriverHandler
DACC_INTR1_DriverHandler
SARC_INTR0_DriverHandler
SARC_INTR1_DriverHandler
SARC_INTR2_DriverHandler
EQEP_INTR0_DriverHandler
EQEP_INTR1_DriverHandler
SDFM_ERR_INTR0_DriverHandler
SDFM_DR_INTR0_DriverHandler
SDFM_DR_INTR1_DriverHandler
SDFM_DR_INTR2_DriverHandler
SDFM_DR_INTR3_DriverHandler
SDFM_ERR_INTR1_DriverHandler
SDFM_DR_INTR4_DriverHandler
SDFM_DR_INTR5_DriverHandler
SDFM_DR_INTR6_DriverHandler
SDFM_DR_INTR7_DriverHandler
CRC_DriverHandler
CORDIC_DriverHandler
FMAC_DriverHandler
STM_INTR0_DriverHandler
STM_INTR1_DriverHandler
AES_DriverHandler
ETIM_INTR0_DriverHandler
ETIM_INTR1_DriverHandler
ETIM_INTR2_DriverHandler
ETIM_INTR3_DriverHandler
ETIM_INTR4_DriverHandler
ETIM_INTR5_DriverHandler
ETIM_INTR6_DriverHandler
ETIM_INTR7_DriverHandler
ETIM_INTR8_DriverHandler
ETIM_INTR9_DriverHandler
ETIM_INTR10_DriverHandler
ETIM_INTR11_DriverHandler
ETIM_INTR12_DriverHandler
ETIM_INTR13_DriverHandler
EPWM_EPWMCOM_INTR0_DriverHandler
EPWM_EPWMCOM_INTR1_DriverHandler
EPWM_EPWMCOM_INTR2_DriverHandler
EPWM_EPWMCOM_INTR3_DriverHandler
EPWM_EPWMCOM_INTR4_DriverHandler
EPWM_EPWMCOM_INTR5_DriverHandler
EPWM_EPWMCOM_INTR6_DriverHandler
EPWM_EPWMCOM_INTR7_DriverHandler
EPWM_EPWMCOM_INTR8_DriverHandler
EPWM_EPWMCOM_INTR9_DriverHandler
EPWM_EPWMCOM_INTR10_DriverHandler
EPWM_EPWMCOM_INTR11_DriverHandler
SYSC_DriverHandler
XBAR_INTR0_DriverHandler
XBAR_INTR1_DriverHandler
XBAR_INTR2_DriverHandler
XBAR_INTR3_DriverHandler
XBAR_INTR4_DriverHandler
                B        .


                END
                
                