/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0L130X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0L130X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_CAR */
#define PWM_CAR_INST                                                       TIMG2
#define PWM_CAR_INST_IRQHandler                                 TIMG2_IRQHandler
#define PWM_CAR_INST_INT_IRQN                                   (TIMG2_INT_IRQn)
#define PWM_CAR_INST_CLK_FREQ                                             100000
/* GPIO defines for channel 0 */
#define GPIO_PWM_CAR_C0_PORT                                               GPIOA
#define GPIO_PWM_CAR_C0_PIN                                        DL_GPIO_PIN_3
#define GPIO_PWM_CAR_C0_IOMUX                                     (IOMUX_PINCM4)
#define GPIO_PWM_CAR_C0_IOMUX_FUNC                    IOMUX_PINCM4_PF_TIMG2_CCP0
#define GPIO_PWM_CAR_C0_IDX                                  DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_CAR_C1_PORT                                               GPIOA
#define GPIO_PWM_CAR_C1_PIN                                        DL_GPIO_PIN_4
#define GPIO_PWM_CAR_C1_IOMUX                                     (IOMUX_PINCM5)
#define GPIO_PWM_CAR_C1_IOMUX_FUNC                    IOMUX_PINCM5_PF_TIMG2_CCP1
#define GPIO_PWM_CAR_C1_IDX                                  DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (399U)



/* Defines for UART_1 */
#define UART_1_INST                                                        UART0
#define UART_1_INST_IRQHandler                                  UART0_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_22
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_23
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM23_PF_UART0_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM24_PF_UART0_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_1_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for PIN_27: GPIOA.27 with pinCMx 28 on package pin 31 */
#define LED_PIN_27_PIN                                          (DL_GPIO_PIN_27)
#define LED_PIN_27_IOMUX                                         (IOMUX_PINCM28)
/* Port definition for Pin Group BEEP */
#define BEEP_PORT                                                        (GPIOA)

/* Defines for PIN_26: GPIOA.26 with pinCMx 27 on package pin 30 */
#define BEEP_PIN_26_PIN                                         (DL_GPIO_PIN_26)
#define BEEP_PIN_26_IOMUX                                        (IOMUX_PINCM27)
/* Port definition for Pin Group TB6612 */
#define TB6612_PORT                                                      (GPIOA)

/* Defines for AIN1: GPIOA.5 with pinCMx 6 on package pin 9 */
#define TB6612_AIN1_PIN                                          (DL_GPIO_PIN_5)
#define TB6612_AIN1_IOMUX                                         (IOMUX_PINCM6)
/* Defines for AIN2: GPIOA.6 with pinCMx 7 on package pin 10 */
#define TB6612_AIN2_PIN                                          (DL_GPIO_PIN_6)
#define TB6612_AIN2_IOMUX                                         (IOMUX_PINCM7)
/* Defines for BIN1: GPIOA.7 with pinCMx 8 on package pin 11 */
#define TB6612_BIN1_PIN                                          (DL_GPIO_PIN_7)
#define TB6612_BIN1_IOMUX                                         (IOMUX_PINCM8)
/* Defines for BIN2: GPIOA.8 with pinCMx 9 on package pin 12 */
#define TB6612_BIN2_PIN                                          (DL_GPIO_PIN_8)
#define TB6612_BIN2_IOMUX                                         (IOMUX_PINCM9)
/* Port definition for Pin Group Encoder */
#define Encoder_PORT                                                     (GPIOA)

/* Defines for Encoder_A: GPIOA.10 with pinCMx 11 on package pin 14 */
// pins affected by this interrupt request:["Encoder_A","Encoder_B","Dncoder_A","Dncoder_B"]
#define Encoder_INT_IRQN                                        (GPIOA_INT_IRQn)
#define Encoder_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Encoder_Encoder_A_IIDX                              (DL_GPIO_IIDX_DIO10)
#define Encoder_Encoder_A_PIN                                   (DL_GPIO_PIN_10)
#define Encoder_Encoder_A_IOMUX                                  (IOMUX_PINCM11)
/* Defines for Encoder_B: GPIOA.11 with pinCMx 12 on package pin 15 */
#define Encoder_Encoder_B_IIDX                              (DL_GPIO_IIDX_DIO11)
#define Encoder_Encoder_B_PIN                                   (DL_GPIO_PIN_11)
#define Encoder_Encoder_B_IOMUX                                  (IOMUX_PINCM12)
/* Defines for Dncoder_A: GPIOA.25 with pinCMx 26 on package pin 29 */
#define Encoder_Dncoder_A_IIDX                              (DL_GPIO_IIDX_DIO25)
#define Encoder_Dncoder_A_PIN                                   (DL_GPIO_PIN_25)
#define Encoder_Dncoder_A_IOMUX                                  (IOMUX_PINCM26)
/* Defines for Dncoder_B: GPIOA.24 with pinCMx 25 on package pin 28 */
#define Encoder_Dncoder_B_IIDX                              (DL_GPIO_IIDX_DIO24)
#define Encoder_Dncoder_B_PIN                                   (DL_GPIO_PIN_24)
#define Encoder_Dncoder_B_IOMUX                                  (IOMUX_PINCM25)
/* Port definition for Pin Group GUANG */
#define GUANG_PORT                                                       (GPIOA)

/* Defines for PIN_15: GPIOA.15 with pinCMx 16 on package pin 19 */
#define GUANG_PIN_15_PIN                                        (DL_GPIO_PIN_15)
#define GUANG_PIN_15_IOMUX                                       (IOMUX_PINCM16)
/* Defines for PIN_16: GPIOA.16 with pinCMx 17 on package pin 20 */
#define GUANG_PIN_16_PIN                                        (DL_GPIO_PIN_16)
#define GUANG_PIN_16_IOMUX                                       (IOMUX_PINCM17)
/* Defines for PIN_17: GPIOA.17 with pinCMx 18 on package pin 21 */
#define GUANG_PIN_17_PIN                                        (DL_GPIO_PIN_17)
#define GUANG_PIN_17_IOMUX                                       (IOMUX_PINCM18)
/* Defines for PIN_18: GPIOA.18 with pinCMx 19 on package pin 22 */
#define GUANG_PIN_18_PIN                                        (DL_GPIO_PIN_18)
#define GUANG_PIN_18_IOMUX                                       (IOMUX_PINCM19)
/* Defines for PIN_2: GPIOA.2 with pinCMx 3 on package pin 6 */
#define GUANG_PIN_2_PIN                                          (DL_GPIO_PIN_2)
#define GUANG_PIN_2_IOMUX                                         (IOMUX_PINCM3)
/* Defines for PIN_9: GPIOA.9 with pinCMx 10 on package pin 13 */
#define GUANG_PIN_9_PIN                                          (DL_GPIO_PIN_9)
#define GUANG_PIN_9_IOMUX                                        (IOMUX_PINCM10)
/* Defines for PIN_21: GPIOA.21 with pinCMx 22 on package pin 25 */
#define GUANG_PIN_21_PIN                                        (DL_GPIO_PIN_21)
#define GUANG_PIN_21_IOMUX                                       (IOMUX_PINCM22)
/* Defines for PIN_14: GPIOA.14 with pinCMx 15 on package pin 18 */
#define GUANG_PIN_14_PIN                                        (DL_GPIO_PIN_14)
#define GUANG_PIN_14_IOMUX                                       (IOMUX_PINCM15)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for PIN_12: GPIOA.12 with pinCMx 13 on package pin 16 */
#define KEY_PIN_12_PIN                                          (DL_GPIO_PIN_12)
#define KEY_PIN_12_IOMUX                                         (IOMUX_PINCM13)
/* Defines for PIN_13: GPIOA.13 with pinCMx 14 on package pin 17 */
#define KEY_PIN_13_PIN                                          (DL_GPIO_PIN_13)
#define KEY_PIN_13_IOMUX                                         (IOMUX_PINCM14)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_CAR_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_1_init(void);

void SYSCFG_DL_SYSTICK_init(void);


#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
