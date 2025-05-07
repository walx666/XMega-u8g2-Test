/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
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
#include "conf_board.h"
#include "conf_usb.h"
#include "main.h"
#include "twi.h"
//#include "twi_master.h"
#include <util/delay.h>
#include <u8g2.h>

static volatile bool main_b_cdc_enable = false;

#if !defined(LCD_XIAMEN_240X128) && !defined(LCD_XIAMEN_128X64) && !defined(LCD_SH1107_64X128) && !defined(LCD_SSD1306_64X32) && !defined(LCD_NHDC_128X32)
#error !!! Please define Display !!!
//#define LCD_XIAMEN_240X128
//#define LCD_XIAMEN_128X64
//#define LCD_SH1107_64X128
//#define LCD_SSD1306_64X32		// U8G2_SSD1306_64X32_NONAME_1_SW_I2C
//#define LCD_NHDC_128X32		// onboard XMEGA_A3BU_XPLAINED
#endif

#define LCD_PAGED				0
#define LCD_USART				0xD0

#define TWI_INTERFACE			(&TWIIF)
#define TWI_SPEED				50000       //!< TWI data transfer rate

/*************************************************
	Results :
	---------
	LCD_XIAMEN_240X128:
		LCD_PAGED==0 -> @24MHz : 50-55ms	@32MHz, 30-39ms
		LCD_PAGED==1 -> @24MHz : 205-210ms	@32MHz, 136-167
		LCD_PAGED==2 -> @24MHz : 120-140ms	@32MHz, 85-101

	LCD_XIAMEN_128X64:
		LCD_PAGED==0 -> @24MHz : 25-27ms	@32MHz, 18,4ms (17-19ms)
		LCD_PAGED==1 -> @24MHz : 70-74ms	@32MHz,
		LCD_PAGED==2 -> @24MHz : 41-46ms	@32MHz,
*/

 #if	(LCD_USART==0xC0)
#define LCD_USART_INTERFACE		&USARTC0
 #elif	(LCD_USART==0xD0)
#define LCD_USART_INTERFACE		&USARTD0
 #elif	(LCD_USART==0xE0)
#define LCD_USART_INTERFACE		&USARTE0
 #else
#define	LCD_SPI_INTERFACE		&SPIC
 #endif
#define LCD_SERIAL_CLOCKSPEED	12000000

#ifdef LCD_USART_INTERFACE
# include <usart_spi.h>
#elif defined(LCD_SPI_INTERFACE)
# include <spi_master.h>
#endif

 #if	(LCD_USART==0xC0)
#define LCD_SPI_SCK         IOPORT_CREATE_PIN(PORTC, 7)
//#define LCD_SPI_MISO        IOPORT_CREATE_PIN(PORTC, 6)
#define LCD_SPI_MOSI        IOPORT_CREATE_PIN(PORTC, 5)
#define LCD_DC_PIN	        IOPORT_CREATE_PIN(PORTC, 4)
#define LCD_CS_PIN          IOPORT_CREATE_PIN(PORTC, 2)
#define LCD_RES_PIN			IOPORT_CREATE_PIN(PORTC, 3)
 #elif	(LCD_USART==0xD0)
#define LCD_SPI_SCK         IOPORT_CREATE_PIN(PORTD, 1)
//#define LCD_SPI_MISO        IOPORT_CREATE_PIN(PORTD, 2)
#define LCD_SPI_MOSI        IOPORT_CREATE_PIN(PORTD, 3)
  #if defined(LCD_NHDC_128X32) && defined(XMEGA_A3BU_XPLAINED)
#define LCD_DC_PIN			IOPORT_CREATE_PIN(PORTD, 0)
#define LCD_CS_PIN          IOPORT_CREATE_PIN(PORTF, 3)
#define LCD_RES_PIN			NHD_C12832A1Z_RESETN
#define LCD_BACKLIGHT_PIN	NHD_C12832A1Z_BACKLIGHT
  #else
#define LCD_DC_PIN	        IOPORT_CREATE_PIN(PORTD, 0)
#define LCD_CS_PIN          IOPORT_CREATE_PIN(PORTE, 2)
#define LCD_RES_PIN			IOPORT_CREATE_PIN(PORTE, 3)
  #endif
 #else
#define LCD_SPI_SCK         IOPORT_CREATE_PIN(PORTC, 7)
//#define LCD_SPI_MISO        IOPORT_CREATE_PIN(PORTC, 6)
#define LCD_SPI_MOSI        IOPORT_CREATE_PIN(PORTC, 5)
#define LCD_DC_PIN	        IOPORT_CREATE_PIN(PORTC, 4)
#define LCD_CS_PIN          IOPORT_CREATE_PIN(PORTC, 2)
#define LCD_RES_PIN			IOPORT_CREATE_PIN(PORTC, 3)
 #endif

#if		defined(LCD_SH1107_64X128)
#define LCD_TWI_ADDR		(0x78)
#elif	defined(LCD_SSD1306_64X32)
#define LCD_TWI_ADDR		(0x3C)
#endif

#define LCD_CS(x)			arch_ioport_set_pin_level(LCD_CS_PIN, x)
#define LCD_CS_Enable()		arch_ioport_set_pin_level(LCD_CS_PIN, false)
#define LCD_CS_Disable()	arch_ioport_set_pin_level(LCD_CS_PIN, true)

 #if	defined(LCD_RES_PIN)
#define	LCD_ResetPIN_Init()	ioport_configure_pin(LED3_GPIO, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
 #elif	defined(LCD_RESN_PIN)	
#define	LCD_ResetPIN_Init() ioport_configure_pin(LED3_GPIO, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW | IOPORT_INV_ENABLED);
 #else
#define	LCD_ResetPIN_Init()
 #endif

#define LCD_Reset(x)		arch_ioport_set_pin_level(LCD_RES_PIN, x)
#define LCD_Reset_Clear()   arch_ioport_set_pin_level(LCD_RES_PIN, false)
#define LCD_Reset_Set()     arch_ioport_set_pin_level(LCD_RES_PIN, true)

#define LCD_DC(x)			arch_ioport_set_pin_level(LCD_DC_PIN, x)
#define LCD_Select_Data()   arch_ioport_set_pin_level(LCD_DC_PIN, false)
#define LCD_Select_Cmd()    arch_ioport_set_pin_level(LCD_DC_PIN, true)

#define LCD_DBG0            J3_PIN7

#define PDC7_SYSCLK_OUT		IOPORT_CREATE_PIN(PORTC, 7)

void LCD_Interface_Init(void);
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_avrx_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

u8g2_t u8g2;

int main (void)
{
	char stop=0;
	uint16_t cnt1=0, cnt2=9999;
	char strbuffer[100];

	#ifdef PDC7_SYSCLK_OUT
	ioport_configure_pin(PDC7_SYSCLK_OUT, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	PORTCFG_CLKEVOUT = (PORTCFG_CLKOUT_PC7_gc | PORTCFG_CLKOUTSEL_CLK1X_gc);
	#endif
    /* Initialize ASF services */
    sleepmgr_init();
    sysclk_init();
    board_init();

	/* Insert system clock initialization code here (sysclk_init()). */
	irq_initialize_vectors();
	cpu_irq_enable();

   #if (BOARD == XMEGA_A3BU_XPLAINED) || (BOARD == XMEGA_C3_XPLAINED)
	/* The status LED must be used as LED2, so we turn off
	 * the green led which is in the same packaging. */
	LED_Off(LED3_GPIO);
	ioport_configure_pin(LCD_DBG0,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	//rtc_init();
   #endif
   #ifdef LCD_BACKLIGHT_PIN
	ioport_configure_pin(LCD_BACKLIGHT_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
   #endif
   #if (BOARD == XMEGA_C3_XPLAINED)
    //rtc_init();
   #else
   #endif
    stdio_usb_init(); /* Initialize STDIO and start USB */

	// Start USB stack to authorize VBus monitoring
	udc_start();
	
	LED_On(LED1_GPIO);
    
    LCD_Interface_Init();
   #if	defined(LCD_XIAMEN_240X128)
	#define LCD_SIZE 3
	#if		(LCD_PAGED==1)
	u8g2_Setup_uc1638_240x128_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	#elif	(LCD_PAGED==2)
	u8g2_Setup_uc1638_240x128_2(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	#else
	u8g2_Setup_uc1638_240x128_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	#endif
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);
    u8g2_SetContrast(&u8g2,160);
   #elif defined(LCD_XIAMEN_128X64)
	#define LCD_SIZE 2
	//u8g2_Setup_st7565_ea_dogm128_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	//u8g2_Setup_st7565_lm6063_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	//u8g2_Setup_st7565_64128n_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* no picture */
	//u8g2_Setup_st7565_zolen_128x64_f(&u8g2, U8G2_MIRROR, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay);
	//u8g2_Setup_st7565_lm6059_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* 32pixel vertical offset, to high contrast */
	//u8g2_Setup_st7565_ks0713_f(&u8g2, U8G2_MIRROR_VERTICAL, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* 1pixel horizontal offset, to high contrast */
	//u8g2_Setup_st7565_lx12864_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* 1pixel horizontal offset, 32pixel vertical offset */
	//u8g2_Setup_st7565_erc12864_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* to high contrast */
	#if		(LCD_PAGED==1)
	u8g2_Setup_st7565_erc12864_alt_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#elif	(LCD_PAGED==2)
	u8g2_Setup_st7565_erc12864_alt_2(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#else
	u8g2_Setup_st7565_erc12864_alt_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#endif
	//u8g2_Setup_st7565_nhd_c12864_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok with constrast 100 !!! */
	//u8g2_Setup_st7565_jlx12864_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok with constrast 100 !!! */
	//u8g2_Setup_st7565_nhd_c12832_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* no picture, wrong resolution */
	//u8g2_Setup_st7565_ea_dogm132_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* no picture, wrong resolution */
	//
	u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);
    u8g2_SetContrast(&u8g2,30);
	//u8g2_SetContrast(&u8g2,100);
   #elif defined(LCD_NHDC_128X32)
	#define LCD_SIZE 1
	#if		(LCD_PAGED==1)
	u8g2_Setup_st7565_nhd_c12832_1(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#elif	(LCD_PAGED==2)
	u8g2_Setup_st7565_nhd_c12832_2(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#else
	u8g2_Setup_st7565_nhd_c12832_f(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi, u8x8_avrx_delay); /* !!! ok !!! */
	#endif
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2,0);
	u8g2_SetContrast(&u8g2,50);
   #elif defined(LCD_SH1107_64X128)
	#define LCD_SIZE 2
	#if		(LCD_PAGED==1)
	u8g2_Setup_sh1107_i2c_64x128_1(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
	#elif	(LCD_PAGED==2)
	u8g2_Setup_sh1107_i2c_64x128_2(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
	#else
	u8g2_Setup_sh1107_i2c_64x128_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
	#endif
	//
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2,0);
	//u8g2_SetContrast(&u8g2,30);
   #elif defined(LCD_SSD1306_64X32)
	#define LCD_SIZE 0
	#if		(LCD_PAGED==1)
	u8g2_Setup_ssd1306_64x32_noname_1(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
	#elif	(LCD_PAGED==2)
	u8g2_Setup_ssd1306_64x32_noname_2(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
	#else
	u8g2_Setup_ssd1306_64x32_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_avrx_delay);
    #endif
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2,0);
	//u8g2_SetContrast(&u8g2,30);
   #endif
     
	while(true)
    {
		LED_On(LED1_GPIO);
		gpio_set_pin_low(LCD_DBG0);
		//
	#if LCD_PAGED
		u8g2_FirstPage(&u8g2);
		do {
	#else 
		u8g2_ClearBuffer(&u8g2);
	#endif
        LED_Toggle(LED1_GPIO);
	#if 1
		u8g2_DrawFrame(&u8g2, 0, 0, u8g2.width, u8g2.height);
        LED_Toggle(LED1_GPIO);
	#endif
		//
	  #if	(LCD_SIZE > 2)
		sprintf (strbuffer, "U8g2 : %05u %c", cnt1, '0'+(stop & 1));
		u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
		u8g2_DrawStr(&u8g2, 20, 28, strbuffer);
	  #elif	(LCD_SIZE > 1)
		sprintf (strbuffer, "U8g2 : %05u %c", cnt1, '0'+(stop & 1));
		u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
		u8g2_DrawStr(&u8g2, 12, 14, strbuffer);
	  #else
		sprintf (strbuffer, "U8g2 : ");
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 80, 14, strbuffer);
		sprintf (strbuffer, "%05u %c", cnt1, '0'+(stop & 1));
		u8g2_DrawStr(&u8g2, 80, 26, strbuffer);
	  #endif
        LED_Toggle(LED1_GPIO);
		//
		sprintf (strbuffer, "%04u", cnt2);
	  #if	(LCD_SIZE > 2)
		u8g2_SetFont(&u8g2, u8g2_font_logisoso78_tn);
		u8g2_DrawStr(&u8g2, 20, 116, strbuffer);
	  #elif	(LCD_SIZE > 1)
		u8g2_SetFont(&u8g2, u8g2_font_logisoso42_tn);
		u8g2_DrawStr(&u8g2, 8, 58, strbuffer);
	  #else
	  	u8g2_SetFont(&u8g2, u8g2_font_logisoso28_tn);
		u8g2_DrawStr(&u8g2, 3, 30, strbuffer);
	  #endif
        LED_Toggle(LED1_GPIO);
		//
		#if LCD_PAGED
		} while( u8g2_NextPage(&u8g2) );		
		#else
		u8g2_SendBuffer(&u8g2);
		#endif
		cnt1--;
		if (++cnt2>9999)
			cnt2=0;
		
		LED_Toggle(LED1_GPIO);
		LED_Off(LED1_GPIO);
		gpio_set_pin_high(LCD_DBG0);

        //LED_Toggle(LED1_GPIO);
	  #if 1
		if (stop)
		{
			if (ioport_pin_is_high(GPIO_PUSH_BUTTON_0))
			{
				while (ioport_pin_is_high(GPIO_PUSH_BUTTON_0))	{
					delay_us(10);
					if (ioport_pin_is_low(GPIO_PUSH_BUTTON_1))	{
						delay_us(10);
						break;
					}
				}
			}
		}
		// else
		{
			if (ioport_pin_is_low(GPIO_PUSH_BUTTON_1))
			{
				while (ioport_pin_is_low(GPIO_PUSH_BUTTON_1))	
					delay_us(10);
				
				if (stop)
					stop=0;
				else
					stop=1;
			}
		}
		delay_ms(50);
	  #endif
	};
}


void LCD_Interface_Init(void)
{
    spi_flags_t spi_flags = SPI_MODE_0;
    board_spi_select_id_t spi_select_id = 0;
	//
   #if defined(TWI_INTERFACE)
	ioport_configure_pin(TWIC_SDA,		IOPORT_DIR_INPUT);
	ioport_configure_pin(TWIC_SCL,		IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
   #endif
	//LCD_ResetPIN_Init();
	ioport_configure_pin(LCD_CS_PIN,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LCD_DC_PIN,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
   #if	defined(LCD_RES_PIN)
	ioport_configure_pin(LCD_RES_PIN,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
   #elif defined(LCD_RESN_PIN)
    ioport_configure_pin(LCD_RESN_PIN,	IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
   #endif
	ioport_configure_pin(LCD_SPI_MOSI,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
   #ifdef LCD_SPI_MISO
    ioport_configure_pin(LCD_SPI_MISO,	IOPORT_DIR_INPUT);
   #endif
	//delay_ms(5);
	//LCD_Reset_Clear();
   #if defined(LCD_USART_INTERFACE)
    struct usart_spi_device device = {
	    .id = LCD_CS_PIN,
    };
    usart_spi_init(LCD_USART_INTERFACE);
    usart_spi_setup_device(LCD_USART_INTERFACE, &device, spi_flags,LCD_SERIAL_CLOCKSPEED, spi_select_id);
   #elif defined(LCD_SPI_INTERFACE)
    struct spi_device device = {
	    .id = LCD_CS_PIN,
    };
    spi_master_init(LCD_SPI_INTERFACE);
    spi_master_setup_device(LCD_SPI_INTERFACE, &device, spi_flags,LCD_SERIAL_CLOCKSPEED, spi_select_id);
    #ifdef SAM
    spi_enable(LCD_SPI_INTERFACE);
    #endif
   #endif
}


#ifndef P_CPU_NS
#define P_CPU_NS (1000000000UL / F_CPU)
#endif

uint8_t u8x8_avrx_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	uint8_t cycles;

	switch(msg)
	{
		case U8X8_MSG_DELAY_NANO:     // delay arg_int * 1 nano second
		// At 20Mhz, each cycle is 50ns, the call itself is slower.
		break;
		case U8X8_MSG_DELAY_100NANO:    // delay arg_int * 100 nano seconds
		// Approximate best case values...
		#define CALL_CYCLES 26UL
		#define CALC_CYCLES 4UL
		#define RETURN_CYCLES 4UL
		#define CYCLES_PER_LOOP 4UL

		cycles = (100UL * arg_int) / (P_CPU_NS * CYCLES_PER_LOOP);

		if(cycles > CALL_CYCLES + RETURN_CYCLES + CALC_CYCLES)
		break;

		__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (cycles) : "0" (cycles) // 2 cycles
		);
		break;
		case U8X8_MSG_DELAY_10MICRO:    // delay arg_int * 10 micro seconds
		while( arg_int-- ) _delay_us(10);
		break;
		case U8X8_MSG_DELAY_MILLI:      // delay arg_int * 1 milli second
		while( arg_int-- ) _delay_ms(1);
		break;
		default:
		return 0;
	}
	return 1;
}


uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *data, b;
    
    switch(msg)
    {
        case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while( arg_int > 0 )
        {
            arg_int--;
			b = *data++;
			#if defined(LCD_USART_INTERFACE)
			usart_spi_transmit(LCD_USART_INTERFACE, b);
			#elif defined(LCD_SPI_INTERFACE)
			spi_write_single(LCD_SPI_INTERFACE, b);
			//delay_us(SSD1306_LATENCY); // At least 3us
			#endif
        }
		//spi_write_packet(LCD_SPI_INTERFACE,data,arg_int);
        break;
        
        case U8X8_MSG_BYTE_INIT:
        /* disable chipselect */
        LCD_CS_Disable();
		//u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
        /* no wait required here */
        
        /* for SPI: setup correct level of the clock signal */
        //u8x8_gpio_SetSPIClock(u8x8, u8x8_GetSPIClockPhase(u8x8));
        break;
        case U8X8_MSG_BYTE_SET_DC:
        LCD_DC(arg_int);
		//u8x8_gpio_SetDC(u8x8, arg_int);
        break;
        case U8X8_MSG_BYTE_START_TRANSFER:
        LCD_CS_Enable();
		//u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
        u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
        break;
        case U8X8_MSG_BYTE_END_TRANSFER:
        u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
		//u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
        LCD_CS_Disable();
        break;
        default:
        return 0;
    }
    return 1;
}

  #ifdef LCD_TWI_ADDR
 uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	uint8_t status = TWI_STATUS_OK;
	uint8_t *data;
	switch(msg){
		case U8X8_MSG_BYTE_SEND: {
			data = (uint8_t *)arg_ptr;
			while( arg_int-- /* && (status==TWI_STATUS_OK) */ ) {
				status = send_TWI(TWI_INTERFACE,*data++);
			}
		}	break;
		case U8X8_MSG_BYTE_INIT: {
		    enable_TWI(TWI_INTERFACE,BAUD_400K, TIMEOUT_DIS);
		}	break;
		case U8X8_MSG_BYTE_SET_DC:
			/* ignored for i2c */
		break;
		case U8X8_MSG_BYTE_START_TRANSFER: {
			//return (start_TWI(LCD_TWI_ADDR, WRITE)==TWI_STATUS_OK);
			status = start_TWI(TWI_INTERFACE,LCD_TWI_ADDR, WRITE);
		}	break;
		case U8X8_MSG_BYTE_END_TRANSFER: {			
			stop_TWI(TWI_INTERFACE);
		}	break;
		default:	{
			return 0;
		}
	}
	return 1;
}
  #endif

void SetAllLeds (uint8_t ledstate) {
   #if (BOARD == XMEGA_A3BU_XPLAINED) || (BOARD == XMEGA_C3_XPLAINED)
	ioport_configure_pin(LED0_GPIO,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LED1_GPIO,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LED2_GPIO,	IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LED3_GPIO, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW	| IOPORT_INV_ENABLED);
    
	if (ledstate) {
		LED_On(LED0_GPIO);
		LED_On(LED1_GPIO);
		LED_On(LED2_GPIO);
		LED_On(LED3_GPIO);
	} 
	else {
		LED_Off(LED0_GPIO);
		LED_Off(LED1_GPIO);
		LED_Off(LED2_GPIO);
		LED_Off(LED3_GPIO);
	}
   #endif
}

void main_suspend_action(void)
{
	//ui_powerdown();
}

void main_resume_action(void)
{
	//ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
	return;
	//ui_process(udd_get_frame_number());
}

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
   #if defined(USART_ON_PORTC) ||defined(USART_ON_PORTD) || defined(USART_ON_PORTE)
	uart_open(port);
   #endif
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
   #if defined(USART_ON_PORTC) ||defined(USART_ON_PORTD) || defined(USART_ON_PORTE)
	uart_close(port);
   #endif
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		LED_On(LED0_GPIO);	//ui_com_open(port);
	}else{
		// Host terminal has close COM
		LED_Off(LED0_GPIO); //ui_com_close(port);
	}
}