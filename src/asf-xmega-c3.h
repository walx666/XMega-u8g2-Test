/**
 * \file
 *
 * \brief Autogenerated API include file for the Atmel Software Framework (ASF)
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef ASF_H
#define ASF_H

/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: CPU specific features
#include <ccp.h>
#include <xmega_reset_cause.h>

// From module: Delay routines
#include <delay.h>

// From module: GPIO - General purpose Input/Output
#include <gpio.h>

// From module: Generic board support
#include <board.h>

// From module: IOPORT - General purpose I/O service
#include <ioport.h>

// From module: Interrupt management - XMEGA implementation
#include <interrupt.h>

// From module: NVM - Non Volatile Memory
#include <nvm.h>

// From module: PMIC - Programmable Multi-level Interrupt Controller
#include <pmic.h>

// From module: Part identification macros
#include <parts.h>

// From module: RTC - Real Time Counter
#include <rtc.h>

// From module: SPI - Serial Peripheral Interface
#include <spi.h>

// From module: SPI - XMEGA implementation
#include <usart_spi.h>
#include <xmega_usart_spi/usart_spi.h>

// From module: Sleep Controller driver
#include <sleep.h>

// From module: Sleep manager - XMEGA A/AU/B/D implementation
#include <sleepmgr.h>
#include <xmega/sleepmgr.h>

// From module: System Clock Control - XMEGA A1/A3/A3B/A4/D/E implementation
#include <sysclk.h>

// From module: TC - Timer Counter
#include <tc.h>

// From module: USART - Serial interface - XMEGA implementation
#include <serial.h>

// From module: USART - Universal Synchronous/Asynchronous Receiver/Transmitter
#include <usart.h>

// From module: USB CDC Protocol
#include <usb_protocol_cdc.h>

// From module: USB Device CDC (Single Interface Device)
#include <udi_cdc.h>

// From module: USB Device CDC Standard I/O (stdio) - AVR implementation
#include <stdio_usb.h>

// From module: USB Device Stack Core (Common API)
#include <udc.h>
#include <udd.h>

// From module: XMEGA compiler driver
#include <compiler.h>
#include <status_codes.h>

// From module: XMEGA-C3 Xplained LED support enabled
#include <led.h>

#endif // ASF_H
