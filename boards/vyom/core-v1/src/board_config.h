/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Vyom core-v1: flight controller half of the drone-core board (STM32H743VIH6).
 * Pin source of truth: hw/drone-core/fc_pinmap.csv (monorepo, PR #2). Checked by
 * fw/flight/tools/check_core_v1_pinmap.py (monorepo).
 *
 * What this FC does NOT have (by design): USB, SD card, CAN, LEDs, buzzer, IMU heater.
 * USB/LEDs/storage belong to the vision SoC; the FC keeps flying if the SoC dies.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* Parameters live in internal flash (no SD card on the FC). */
#define FLASH_BASED_PARAMS

/* No LEDs on the FC: status is shown by the SoC/arm lights. */
#undef BOARD_HAS_CONTROL_STATUS_LEDS

/* ADC ------------------------------------------------------------------------------ */
#define ADC1_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PA2 */  GPIO_ADC12_INP14, \
	/* PA3 */  GPIO_ADC12_INP15

#define ADC_BATTERY_CURRENT_CHANNEL     /* PA2 IBAT_SENSE */  ADC1_CH(14)
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PA3 VBAT_SENSE */  ADC1_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

/* Divider and shunt amplifier on drone-power-esc: TODO(verify) NEEDS-EE.
 * Placeholders chosen so a 2S pack (8.4 V max) stays below 3.3 V at the ADC. */
#define BOARD_BATTERY1_V_DIV         (4.0f)    /* TODO(verify) */
#define BOARD_BATTERY1_A_PER_V       (10.0f)   /* TODO(verify) */

/* Power, reset and enables (fc_pinmap.csv) ------------------------------------------- */
#define GPIO_SOC_RESET_N    /* PC0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN0)
#define GPIO_SOC_PWR_EN     /* PC1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_PG_5V0         /* PC12 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN12)
#define GPIO_PG_3V3         /* PC13 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_GNSS_RESET_N   /* PC4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)
#define GPIO_GIMBAL_PWR_EN  /* PD1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN1)
#define GPIO_ESC_ALERT_N    /* PC9 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN9)
#define GPIO_WDT_WDI        /* PD0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN0)

#define SOC_POWER_EN(on_true)     px4_arch_gpiowrite(GPIO_SOC_PWR_EN, (on_true))
#define SOC_RESET(on_true)        px4_arch_gpiowrite(GPIO_SOC_RESET_N, !(on_true))
#define GIMBAL_POWER_EN(on_true)  px4_arch_gpiowrite(GPIO_GIMBAL_PWR_EN, (on_true))

#define BOARD_ADC_BRICK_VALID     (px4_arch_gpioread(GPIO_PG_5V0))
#define BOARD_ADC_SERVO_VALID     (1)
#define BOARD_ADC_USB_CONNECTED   (0)
#define BOARD_ADC_USB_VALID       (0)

/* Sensor and SoC interrupt lines (used by drivers / vyom_soc_link in WP2) */
#define GPIO_MAG_INT        /* PC5 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN5)
#define GPIO_BARO_INT       /* PC6 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN6)
#define GPIO_TOF_INT        /* PC7 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)
#define GPIO_TOF_XSHUT      /* PC8 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN8)
#define GPIO_SOC_IRQ0       /* PB4 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SOC_IRQ1       /* PB5 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN5)

/* GNSS PPS on TIM2_CH1 (PA0) and IMU B CLKIN on TIM2_CH2 (PA1): wired, not used yet.
 * Held as plain inputs until the PPS capture / CLKIN work (WP3 / WP4). */
#define GPIO_GNSS_PPS       /* PA0 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_IMUB_CLKIN     /* PA1 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN1)

/* Motor outputs: 4x DShot on TIM1 CH1..CH4 ------------------------------------------- */
#define DIRECT_PWM_OUTPUT_CHANNELS   4
#define DIRECT_INPUT_TIMER_CHANNELS  4
#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS
#define BOARD_DMA_NUM_DSHOT_CHANNELS 1

/* High-resolution timer: TIM3 (no pins needed) */
#define HRT_TIMER               3
#define HRT_TIMER_CHANNEL       1

#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_SOC_RESET_N, \
		GPIO_SOC_PWR_EN, \
		GPIO_PG_5V0, \
		GPIO_PG_3V3, \
		GPIO_GNSS_RESET_N, \
		GPIO_GIMBAL_PWR_EN, \
		GPIO_ESC_ALERT_N, \
		GPIO_WDT_WDI, \
		GPIO_MAG_INT, \
		GPIO_BARO_INT, \
		GPIO_TOF_INT, \
		GPIO_TOF_XSHUT, \
		GPIO_SOC_IRQ0, \
		GPIO_SOC_IRQ1, \
		GPIO_GNSS_PPS, \
		GPIO_IMUB_CLKIN, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_NUM_IO_TIMERS 1

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
