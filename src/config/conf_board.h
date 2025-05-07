/**
 * \file
 *
 * \brief XMEGA-C3 / XMEGA-A3BU Xplained board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

//! Definition of USART connection for this example
//! TXC0 on PC3 and RXC0 on PC2
#define  USART_ON_PORTC
//#define  USART_ON_PORTD
//#define  USART_ON_PORTE

#ifdef XMEGA_C3_XPLAINED
// Initialize SPI and control pins for UG_2832HSWEG04 OLED controller
//#define CONF_BOARD_OLED_UG_2832HSWEG04

// Initialize SPI pins and presence pin for MicroSD card slot
//#define CONF_BOARD_SD_MMC_SPI

// Initialize Analog Comparator pin for light sensor
//#define CONF_BOARD_LIGHT_SENSOR

// Initialize Analog Comparator pin for NTC sensor
//#define CONF_BOARD_TEMPERATURE_SENSOR

// Initialize Analog Comparator pin and input signal pin
// for Analog Filter (lowpass RC @ 159 Hz)
//#define CONF_BOARD_ANALOG_FILTER
#endif

// Initialize IO pins for the LCD controller
#ifdef XMEGA_A3BU_XPLAINED
//#define CONF_BOARD_C12832A1Z

// Initialize IO pins for the DataFlash
//#define CONF_BOARD_AT45DBX

// Initialize IO pins for use with Analog Comparator
//#define CONF_BOARD_ENABLE_AC_PINS
#endif

// Initialize IO pins for use with USART 0 on port C
#ifdef USART_ON_PORTC
#define CONF_BOARD_ENABLE_USARTC0
//
#define USART               USARTC0
#define USART_RX_Vect       USARTC0_RXC_vect
#define USART_DRE_Vect      USARTC0_DRE_vect
#define USART_SYSCLK        SYSCLK_USART0
#define USART_PORT_SYSCLK   SYSCLK_PORT_C
#define USART_PORT          PORTC
#define USART_PORT_PIN_TX   (1<<3)  /* PC3 (TXC0) */
#define USART_PORT_PIN_RX   (1<<2)  /* PC2 (RXC0) */
#endif

// Initialize IO pins for use with USART 0 on port D
#ifdef USART_ON_PORTD
#define CONF_BOARD_ENABLE_USARTD0
//
#define USART               USARTD0
#define USART_RX_Vect       USARTD0_RXC_vect
#define USART_DRE_Vect      USARTD0_DRE_vect
#define USART_SYSCLK        SYSCLK_USART0
#define USART_PORT_SYSCLK   SYSCLK_PORT_D
#define USART_PORT          PORTD
#define USART_PORT_PIN_TX   (1<<3)  /* PD3 (TXD0) */
#define USART_PORT_PIN_RX	(1<<2)  /* PD2 (RXD0) */
#endif

// Initialize IO pins for use with USART 0 on port E
#ifdef USART_ON_PORTE
#define CONF_BOARD_ENABLE_USARTE0
//
#define USART               USARTE0
#define USART_RX_Vect       USARTE0_RXC_vect
#define USART_DRE_Vect      USARTE0_DRE_vect
#define USART_SYSCLK        SYSCLK_USART0
#define USART_PORT_SYSCLK   SYSCLK_PORT_E
#define USART_PORT          PORTE
#define USART_PORT_PIN_TX   (1<<3)  /* PE3 (TXE0) */
#define USART_PORT_PIN_RX   (1<<2)  /* PE2 (RXE0) */
#endif

#endif // CONF_BOARD_H
