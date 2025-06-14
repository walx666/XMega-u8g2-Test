/**
 * \file
 *
 * \brief Functions for interfacing touch
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#if 	defined(XMEGA_C3_XPLAINED)
#include <asf-xmega-c3.h>
 #elif 	defined(XMEGA_A3BU_XPLAINED)
#include <asf-xmega-a3bu.h>
 #else
#include <asf.h>
 #endif
#include "app_touch.h"

/**
 * \ingroup touch_task_group
 * \defgroup touch_task_group_implementation Touch task implementation
 *
 * Stuff that is passed to the compiler:
 * _QTOUCH_
 * SNS1=F
 * SNSK1=F
 * _SNS1_SNSK1_SAME_PORT_
 * QT_DELAY_CYCLES=1 (at 2MHz CPU freq this is 0.5us)
 * QT_NUM_CHANNELS=4
 * QTOUCH_STUDIO_MASKS=1
 *
 * @{
 */

#ifdef _DEBUG_INTERFACE_
/* Include files for QTouch Studio integration */
# include "QDebug.h"
# include "QDebugTransport.h"
#endif

/* Measurement period */
#define APP_TOUCH_PERIOD_MEASUREMENT 25u
/* TIMER_PERIOD computation */
#define TIMER_PERIOD TICKS_PER_MS * APP_TOUCH_PERIOD_MEASUREMENT

/* The timing information for timer to fire periodically to measure touch */
#ifdef TICKS_PER_MS
#  undef TICKS_PER_MS
#  define TICKS_PER_MS   3000u /* 48MHz / 2(PSADIV_2) / 8 (TC_CLKSEL_DIV8) */
#endif

/*
 * This configuration data structure parameters
 */
#ifdef QTOUCH_STUDIO_MASKS
extern TOUCH_DATA_T SNS_array[2][2];
extern TOUCH_DATA_T SNSK_array[2][2];
#endif

/* Output can be observed in the watch window using this pointer */
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;

/* flag set by timer ISR when it's time to measure touch */
static volatile bool time_to_measure_touch = false;

/* current time, set by timer ISR */
static volatile uint16_t current_time_ms_touch = 0;

static void app_touch_tc_interrupt_callback(void);

static void app_touch_tc_interrupt_callback(void)
{
	/* set flag: it's time to measure touch */
	time_to_measure_touch = true;
	/* update the current time */
	current_time_ms_touch += APP_TOUCH_PERIOD_MEASUREMENT;
}

void app_touch_init(void)
{
#ifdef QTOUCH_STUDIO_MASKS
	SNS_array[0][0] = 0x50;
	SNS_array[0][1] = 0x00;
	SNS_array[1][0] = 0x00;
	SNS_array[1][1] = 0x00;

	SNSK_array[0][0] = 0xA0;
	SNSK_array[0][1] = 0x00;
	SNSK_array[1][0] = 0x00;
	SNSK_array[1][1] = 0x00;
#endif

	/* Configures the sensors as keys and assigns the channel numbers.
	 * The sensor is wired up with SNS=PF6 and SNSK=PF7
	 * When using "pin reconfigurability" this will result in channel 0
	 * because it is the first and only channel that is used.
	 * For the standard qtouch library setup we would need to use
	 * channel 3 since we are using the last two pins on the port.
	 */
	qt_enable_key(CHANNEL_0, NO_AKS_GROUP, 10u, HYST_6_25);
	qt_enable_key(CHANNEL_1, NO_AKS_GROUP, 10u, HYST_6_25);

	qt_init_sensing();

	/* This will fill the default threshold values in the configuration
	 * data structure. But User can change the values of these parameters.
	 */
	qt_config_data.qt_di              = DEF_QT_DI;
	qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
	qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
	qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
	qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
	qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
	qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;

	/* Initialize the timer counter */
	tc_enable(&TCC0);
	tc_write_period(&TCC0, TIMER_PERIOD);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV8_gc);
	tc_set_cca_interrupt_level(&TCC0, PMIC_LVL_LOW);
	tc_set_cca_interrupt_callback(&TCC0, app_touch_tc_interrupt_callback);

	/*
	 * Set up callback function. This function is called after the library
	 * has made capacitive measurements, but before it has processed them.
	 * The user can use this hook to apply filter functions to the measured
	 * signal values.(Possibly to fix sensor layout faults)
	 */
	qt_filter_callback = NULL;

#ifdef _DEBUG_INTERFACE_
	QDebug_Init();
#endif
	sleepmgr_lock_mode(SLEEPMGR_IDLE);
}

void app_touch_task(void)
{
	uint16_t status_flag;
	uint16_t burst_flag;

#ifdef _DEBUG_INTERFACE_
	/* Process commands from PC */
	QDebug_ProcessCommands();
#endif

	if (time_to_measure_touch) {
		time_to_measure_touch = false;
		do {
			status_flag = qt_measure_sensors(current_time_ms_touch);
			burst_flag = status_flag & QTLIB_BURST_AGAIN;

#ifdef _DEBUG_INTERFACE_
			QDebug_SendData(status_flag);
#endif

			/*
			 * Time-critical host application code should be placed
			 *here since
			 * a full burst cycle may delay critical task in the
			 *main application
			 */
		} while (burst_flag);

#ifdef _DEBUG_INTERFACE_
		/* Process commands from PC */
		QDebug_ProcessCommands();
#endif
	}
}

bool app_touch_check_key_pressed(uint8_t button_num)
{
	if (qt_measure_data.qt_touch_status.sensor_states[(button_num / 8)]
			& (1 << (button_num % 8))) {
		return 0;
	} else {
		return 1;
	}
}

/* ! @} */
