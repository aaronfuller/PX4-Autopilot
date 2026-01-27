/****************************************************************************
 *
 *   Copyright (c) 2016, 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4FMU-v6u internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

#define BOARD_HAS_LTC44XX_VALIDS      0 // Analog power path only, no MCU feedback
#define BOARD_HAS_USB_VALID           0 // No USB valid detection
#define BOARD_HAS_NBAT_V              1a // 1 Analog Voltage (ADC)
#define BOARD_HAS_NBAT_I              1a // 1 Analog Current (from ESC)

/* PX4FMU GPIOs ***********************************************************************************/

/* LEDs - No status LEDs on this board revision
 * WS2812B RGB LED on PE5 is non-functional (PE5 is SPI4_MISO, can't drive WS2812B)
 * Will be fixed in next board revision
 */

// #define GPIO_nLED_RED        /* PE3 */
// #define GPIO_nLED_GREEN      /* PE4 */
// #define GPIO_nLED_BLUE       /* PE5 */

#define BOARD_HAS_CONTROL_STATUS_LEDS      0
// #define BOARD_OVERLOAD_LED     LED_RED
// #define BOARD_ARMED_STATE_LED  LED_BLUE


/* SPI */

// SPI sensors do not require reset line on this board
// #define SPI6_nRESET_EXTERNAL1       /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)
// Sync signal not used
// #define GPIO_SYNC                   /* PE9  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)

/* I2C busses */

/* Devices on the onboard buses.
 *
 * I2C1: MMC5983MA magnetometer (internal)
 * I2C2: External accessory port
 * I2C4: GPS
 */

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because */
#define ADC3_CH(n)                  (n)

/* We are only use ADC3 for REV/VER.
 * ADC3_6V6 and ADC3_3V3 are mapped back to ADC1
 * To do this We are relying on PC2_C, PC3_C being connected to PC2, PC3
 * respectively by the SYSCFG_PMCR default of setting for PC3SO PC2SO PA1SO
 * PA0SO of 0.
 *
 *  0 Analog switch closed (pads are connected through the analog switch)
 *
 * So ADC3_INP0 is GPIO_ADC123_INP12
 *    ADC3_INP1 is GPIO_ADC12_INP13
 */

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */

#define PX4_ADC_GPIO  \
	/* PA0  */  GPIO_ADC1_INP16,   \
	/* PA4  */  GPIO_ADC12_INP18,  \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC4  */  GPIO_ADC12_INP4,  \
	/* PH3  */  GPIO_ADC3_INP14,   \
	/* PH4  */  GPIO_ADC3_INP15

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL      /* PC4  */  ADC1_CH(4)
#define ADC_SCALED_V5_CHANNEL                   /* PA0  */  ADC1_CH(16)
#define ADC_ADC3_6V6_CHANNEL                    /* PC2  */  ADC3_CH(12)
#define ADC_ADC3_3V3_CHANNEL                    /* PA4  */  ADC1_CH(18)
#define ADC_HW_VER_SENSE_CHANNEL                /* PH3  */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PH4  */  ADC3_CH(15)
#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC_ADC3_3V3_CHANNEL
#define ADC_BATTERY_CURRENT_CHANNEL             ADC_ADC3_6V6_CHANNEL

#define ADC_CHANNELS \
	((1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_ADC3_6V6_CHANNEL)                | \
	 (1 << ADC_ADC3_3V3_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision - Not implemented on this board */
// #define BOARD_HAS_HW_VERSIONING
// #define GPIO_HW_VER_REV_DRIVE  /* PG0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
// #define GPIO_HW_REV_SENSE      /* PH4 */  GPIO_ADC3_INP15
// #define GPIO_HW_VER_SENSE      /* PH3 */  GPIO_ADC3_INP14
// #define HW_INFO_INIT_PREFIX   "V6U"
// #define V6U00                 HW_VER_REV(0x0,0x0)

/* HEATER - Not implemented on this board */
// #define GPIO_HEATER_OUTPUT   /* PA2  T2CH3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
// #define HEATER_OUTPUT_EN(on_true)	      px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* PE6 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define GPIO_nARMED_INIT     /* PE6 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN6)
#define GPIO_nARMED          /* PE6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)

#define BOARD_INDICATE_ARMED_STATE(on_armed)  px4_arch_configgpio((on_armed) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   9


/* Power supply control and monitoring GPIOs */

// Power valid detection not connected - analog power path only
// #define GPIO_nPOWER_IN_A                /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
// #define GPIO_nPOWER_IN_B                /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
// #define GPIO_nPOWER_IN_C                /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)

// #define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
// #define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define BOARD_NUMBER_DIGITAL_BRICKS     2
// #define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

/* Power control - Not implemented, power rails come up automatically */
// #define GPIO_VDD_5V_PERIPH_nEN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
// #define GPIO_VDD_5V_PERIPH_nOC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
// #define GPIO_VDD_5V_HIPOWER_nEN         /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
// #define GPIO_VDD_5V_HIPOWER_nOC         /* PF13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
// #define GPIO_VDD_3V3_SENSORS_EN         /* PG12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN12)
// #define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
// #define GPIO_VDD_3V3_SD_CARD_EN         /* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

/* Spare GPIO */

// #define GPIO_PG6                        /* PG6  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN6)
#define GPIO_PD15                       /* PD15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN15)
// #define GPIO_PG15                       /* PG15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN15)


/* Define True logic Power Control in arch agnostic form - Not implemented */

// #define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
// #define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
// #define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
// #define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
// #define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
// #define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))


/* Tone alarm output - Not wired on this board */

// #define TONE_ALARM_TIMER        14  /* Timer 14 */
// #define TONE_ALARM_CHANNEL      1  /* PF9 GPIO_TIM14_CH1OUT_2 */
// #define GPIO_BUZZER_1           /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)
// #define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
// #define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* PPM input - Not implemented on this board */
// #define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
// #define GPIO_PPM_IN             /* PI5 T8C1 */ GPIO_TIM8_CH1IN_2

/* RC Serial port - SBUS on UART */

#define RC_SERIAL_PORT                     "/dev/ttyS4"  // UART5
#define RC_SERIAL_SINGLEWIRE
#define RC_SERIAL_SWAP_RXTX

/* Input Capture Channels. */
#define INPUT_CAP1_TIMER                  1
#define INPUT_CAP1_CHANNEL     /* T1C2 */ 2
#define GPIO_INPUT_CAP1        /*  PE11 */ GPIO_TIM1_CH2IN

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Safety Switch - Not implemented on this board */
// #define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PD10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN10)
// #define GPIO_nSAFETY_SWITCH_LED_OUT        /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
// #define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
// #define GPIO_SAFETY_SWITCH_IN              /* PF5 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN5)
// #define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN

/* Power switch controls ******************************************************/

// #define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/* RC input - SBUS on UART, no PPM or SPEKTRUM GPIO control */
// #define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)
// #define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
// #define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
// #define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (1)  /* No USB valid detection - always assume valid */

/* FMUv6U never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif

// No overcurrent detection on this board (analog power path only)
// #define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
// #define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_PD15,                        \
		GPIO_nARMED_INIT                  \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
