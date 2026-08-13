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

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     SVC_Handler                ; SVCall Handler
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                DCD     OS_CPU_PendSVHandler      ; PendSV Handler
                DCD     SysTick_Handler            ; SysTick Handler
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

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =__Vectors
                LDR     R1, =0xE000ED08 ;*VTOR register
                STR     R0,[R1]
                ;* C routines are likely to be called. Setup the stack now
                LDR     SP,=__initial_sp

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)
Default_Handler PROC
                EXPORT  NMI_Handler                                   [WEAK]
                EXPORT  HardFault_Handler                             [WEAK]
                EXPORT  MemManage_Handler                             [WEAK]
                EXPORT  BusFault_Handler                              [WEAK]
                EXPORT  UsageFault_Handler                            [WEAK]
                EXPORT  SVC_Handler                                   [WEAK]
                EXPORT  DebugMon_Handler                              [WEAK]
                EXPORT  OS_CPU_PendSVHandler                          [WEAK]
                EXPORT  SysTick_Handler                               [WEAK]
NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SVC_Handler
DebugMon_Handler
OS_CPU_PendSVHandler
SysTick_Handler
                EXPORT  REVERSE_IRQHandler                            [WEAK]
                EXPORT  AHBDMAC0_TFR_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC0_BLOCK_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC0_SRCTRAN_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC0_DSTTRAN_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC0_ERROR_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC0_COMBINED_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC1_TFR_IRQHandler                     [WEAK]
                EXPORT  AHBDMAC1_BLOCK_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC1_SRCTRAN_IRQHandler                 [WEAK]
                EXPORT  AHBDMAC1_DSTTRAN_IRQHandler                 [WEAK]
                EXPORT  AHBDMAC1_ERROR_IRQHandler                   [WEAK]
                EXPORT  AHBDMAC1_COMBINED_IRQHandler                   [WEAK]
                EXPORT  WDG0_IRQHandler                   [WEAK]
                EXPORT  WDG1_IRQHandler                   [WEAK]
                EXPORT  UART0_IRQHandler                   [WEAK]
                EXPORT  UART1_IRQHandler                   [WEAK]
                EXPORT  I2C0_IC_IRQHandler                   [WEAK]
                EXPORT  I2C1_IC_IRQHandler                   [WEAK]
                EXPORT  SPIM0_SSI_IRQHandler                   [WEAK]
                EXPORT  SPIM1_SSI_IRQHandler                   [WEAK]
                EXPORT  SPIM2_SSI_IRQHandler                   [WEAK]
                EXPORT  SPIS0_SSI_IRQHandler                   [WEAK]
                EXPORT  CAN0_INT0_IRQHandler                   [WEAK]
                EXPORT  CAN0_INT1_IRQHandler                   [WEAK]
                EXPORT  CAN0_DMU_IRQHandler                   [WEAK]
                EXPORT  CAN1_INT0_IRQHandler                   [WEAK]
                EXPORT  CAN1_INT1_IRQHandler                   [WEAK]
                EXPORT  CAN1_DMU_IRQHandler                   [WEAK]
                EXPORT  MTTCAN_ECC_IRQHandler                   [WEAK]
                EXPORT  GPIOA_COMBINT_IRQHandler                   [WEAK]
                EXPORT  GPIOB_COMBINT_IRQHandler                   [WEAK]
                EXPORT  GPIOC_COMBINT_IRQHandler                   [WEAK]
                EXPORT  GPIOD_COMBINT_IRQHandler                   [WEAK]
                EXPORT  GPIOE_COMBINT_IRQHandler                   [WEAK]
                EXPORT  GPIOF_COMBINT_IRQHandler                   [WEAK]
                EXPORT  PBUS1_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  PBUS3_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  HBUS1_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  HBUS2_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  HBUS3_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT0_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT1_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT2_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT3_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT4_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT5_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT6_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT7_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT8_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT9_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT10_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT11_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT12_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT13_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT14_IRQHandler                   [WEAK]
                EXPORT  GPIOE_GPIOINT15_IRQHandler                   [WEAK]
                EXPORT  EFC0_FLASH_IRQHandler                   [WEAK]
                EXPORT  EFC1_FLASH_IRQHandler                   [WEAK]
                EXPORT  EFC2_FLASH_IRQHandler                   [WEAK]
                EXPORT  PBUS2_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  SRAM0_ECC_SERR_IRQHandler                   [WEAK]
                EXPORT  SRAM0_ECC_MERR_IRQHandler                   [WEAK]
                EXPORT  SRAM1_ECC_SERR_IRQHandler                   [WEAK]
                EXPORT  SRAM1_ECC_MERR_IRQHandler                   [WEAK]
                EXPORT  CORE2PERI_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  PERI2CORE_TIMEOUT_IRQHandler                   [WEAK]
                EXPORT  NONE_IRQHandler1                   [WEAK]
                EXPORT  NONE_IRQHandler2                   [WEAK]
                EXPORT  NONE_IRQHandler3                   [WEAK]
                EXPORT  MISC_M7_IRQHandler                   [WEAK]
                EXPORT  CMPC_INTR0_IRQHandler                   [WEAK]
                EXPORT  CMPC_INTR1_IRQHandler                   [WEAK]
                EXPORT  CMPC_INTR2_IRQHandler                   [WEAK]
                EXPORT  CMPC_INTR3_IRQHandler                   [WEAK]
                EXPORT  DACC_INTR0_IRQHandler                   [WEAK]
                EXPORT  DACC_INTR1_IRQHandler                   [WEAK]
                EXPORT  SARC_INTR0_IRQHandler                   [WEAK]
                EXPORT  SARC_INTR1_IRQHandler                   [WEAK]
                EXPORT  SARC_INTR2_IRQHandler                   [WEAK]
                EXPORT  EQEP_INTR0_IRQHandler                   [WEAK]
                EXPORT  EQEP_INTR1_IRQHandler                   [WEAK]
                EXPORT  SDFM_ERR_INTR0_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR0_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR1_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR2_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR3_IRQHandler                   [WEAK]
                EXPORT  SDFM_ERR_INTR1_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR4_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR5_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR6_IRQHandler                   [WEAK]
                EXPORT  SDFM_DR_INTR7_IRQHandler                   [WEAK]
                EXPORT  CRC_IRQHandler                   [WEAK]
                EXPORT  CORDIC_IRQHandler                   [WEAK]
                EXPORT  FMAC_IRQHandler                   [WEAK]
                EXPORT  STM_INTR0_IRQHandler                   [WEAK]
                EXPORT  STM_INTR1_IRQHandler                   [WEAK]
                EXPORT  AES_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR0_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR1_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR2_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR3_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR4_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR5_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR6_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR7_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR8_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR9_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR10_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR11_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR12_IRQHandler                   [WEAK]
                EXPORT  ETIM_INTR13_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR0_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR1_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR2_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR3_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR4_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR5_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR6_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR7_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR8_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR9_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR10_IRQHandler                   [WEAK]
                EXPORT  EPWM_EPWMCOM_INTR11_IRQHandler                   [WEAK]
                EXPORT  SYSC_IRQHandler                   [WEAK]
                EXPORT  XBAR_INTR0_IRQHandler                   [WEAK]
                EXPORT  XBAR_INTR1_IRQHandler                   [WEAK]
                EXPORT  XBAR_INTR2_IRQHandler                   [WEAK]
                EXPORT  XBAR_INTR3_IRQHandler                   [WEAK]
                EXPORT  XBAR_INTR4_IRQHandler                   [WEAK]

REVERSE_IRQHandler
AHBDMAC0_TFR_IRQHandler
AHBDMAC0_BLOCK_IRQHandler
AHBDMAC0_SRCTRAN_IRQHandler
AHBDMAC0_DSTTRAN_IRQHandler
AHBDMAC0_ERROR_IRQHandler
AHBDMAC0_COMBINED_IRQHandler
AHBDMAC1_TFR_IRQHandler
AHBDMAC1_BLOCK_IRQHandler
AHBDMAC1_SRCTRAN_IRQHandler
AHBDMAC1_DSTTRAN_IRQHandler
AHBDMAC1_ERROR_IRQHandler
AHBDMAC1_COMBINED_IRQHandler
WDG0_IRQHandler
WDG1_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IC_IRQHandler
I2C1_IC_IRQHandler
SPIM0_SSI_IRQHandler
SPIM1_SSI_IRQHandler
SPIM2_SSI_IRQHandler
SPIS0_SSI_IRQHandler
CAN0_INT0_IRQHandler
CAN0_INT1_IRQHandler
CAN0_DMU_IRQHandler
CAN1_INT0_IRQHandler
CAN1_INT1_IRQHandler
CAN1_DMU_IRQHandler
MTTCAN_ECC_IRQHandler
GPIOA_COMBINT_IRQHandler
GPIOB_COMBINT_IRQHandler
GPIOC_COMBINT_IRQHandler
GPIOD_COMBINT_IRQHandler
GPIOE_COMBINT_IRQHandler
GPIOF_COMBINT_IRQHandler
PBUS1_TIMEOUT_IRQHandler
PBUS3_TIMEOUT_IRQHandler
HBUS1_TIMEOUT_IRQHandler
HBUS2_TIMEOUT_IRQHandler
HBUS3_TIMEOUT_IRQHandler
GPIOE_GPIOINT0_IRQHandler
GPIOE_GPIOINT1_IRQHandler
GPIOE_GPIOINT2_IRQHandler
GPIOE_GPIOINT3_IRQHandler
GPIOE_GPIOINT4_IRQHandler
GPIOE_GPIOINT5_IRQHandler
GPIOE_GPIOINT6_IRQHandler
GPIOE_GPIOINT7_IRQHandler
GPIOE_GPIOINT8_IRQHandler
GPIOE_GPIOINT9_IRQHandler
GPIOE_GPIOINT10_IRQHandler
GPIOE_GPIOINT11_IRQHandler
GPIOE_GPIOINT12_IRQHandler
GPIOE_GPIOINT13_IRQHandler
GPIOE_GPIOINT14_IRQHandler
GPIOE_GPIOINT15_IRQHandler
EFC0_FLASH_IRQHandler
EFC1_FLASH_IRQHandler
EFC2_FLASH_IRQHandler
PBUS2_TIMEOUT_IRQHandler
SRAM0_ECC_SERR_IRQHandler
SRAM0_ECC_MERR_IRQHandler
SRAM1_ECC_SERR_IRQHandler
SRAM1_ECC_MERR_IRQHandler
CORE2PERI_TIMEOUT_IRQHandler
PERI2CORE_TIMEOUT_IRQHandler
NONE_IRQHandler1
NONE_IRQHandler2
NONE_IRQHandler3
MISC_M7_IRQHandler
CMPC_INTR0_IRQHandler
CMPC_INTR1_IRQHandler
CMPC_INTR2_IRQHandler
CMPC_INTR3_IRQHandler
DACC_INTR0_IRQHandler
DACC_INTR1_IRQHandler
SARC_INTR0_IRQHandler
SARC_INTR1_IRQHandler
SARC_INTR2_IRQHandler
EQEP_INTR0_IRQHandler
EQEP_INTR1_IRQHandler
SDFM_ERR_INTR0_IRQHandler
SDFM_DR_INTR0_IRQHandler
SDFM_DR_INTR1_IRQHandler
SDFM_DR_INTR2_IRQHandler
SDFM_DR_INTR3_IRQHandler
SDFM_ERR_INTR1_IRQHandler
SDFM_DR_INTR4_IRQHandler
SDFM_DR_INTR5_IRQHandler
SDFM_DR_INTR6_IRQHandler
SDFM_DR_INTR7_IRQHandler
CRC_IRQHandler
CORDIC_IRQHandler
FMAC_IRQHandler
STM_INTR0_IRQHandler
STM_INTR1_IRQHandler
AES_IRQHandler
ETIM_INTR0_IRQHandler
ETIM_INTR1_IRQHandler
ETIM_INTR2_IRQHandler
ETIM_INTR3_IRQHandler
ETIM_INTR4_IRQHandler
ETIM_INTR5_IRQHandler
ETIM_INTR6_IRQHandler
ETIM_INTR7_IRQHandler
ETIM_INTR8_IRQHandler
ETIM_INTR9_IRQHandler
ETIM_INTR10_IRQHandler
ETIM_INTR11_IRQHandler
ETIM_INTR12_IRQHandler
ETIM_INTR13_IRQHandler
EPWM_EPWMCOM_INTR0_IRQHandler
EPWM_EPWMCOM_INTR1_IRQHandler
EPWM_EPWMCOM_INTR2_IRQHandler
EPWM_EPWMCOM_INTR3_IRQHandler
EPWM_EPWMCOM_INTR4_IRQHandler
EPWM_EPWMCOM_INTR5_IRQHandler
EPWM_EPWMCOM_INTR6_IRQHandler
EPWM_EPWMCOM_INTR7_IRQHandler
EPWM_EPWMCOM_INTR8_IRQHandler
EPWM_EPWMCOM_INTR9_IRQHandler
EPWM_EPWMCOM_INTR10_IRQHandler
EPWM_EPWMCOM_INTR11_IRQHandler
SYSC_IRQHandler
XBAR_INTR0_IRQHandler
XBAR_INTR1_IRQHandler
XBAR_INTR2_IRQHandler
XBAR_INTR3_IRQHandler
XBAR_INTR4_IRQHandler

            B       .

            ENDP

            ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
            IF      :DEF:__MICROLIB

            EXPORT  __initial_sp
            EXPORT  __heap_base
            EXPORT  __heap_limit

            ELSE

            IMPORT  __use_two_region_memory
            EXPORT  __user_initial_stackheap

__user_initial_stackheap

            LDR     R0, =  Heap_Mem
            LDR     R1, =(Stack_Mem + Stack_Size)
            LDR     R2, = (Heap_Mem +  Heap_Size)
            LDR     R3, = Stack_Mem
            BX      LR

            ALIGN

            ENDIF
;*******************************************************************************
; Software Delay function
;*******************************************************************************

     END
;************************ (C) COPYRIGHT ETMCU *****END OF FILE*****
