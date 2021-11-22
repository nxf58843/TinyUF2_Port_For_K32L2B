/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//Include header files
#include "board_api.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_sim.h"

#ifndef BUILD_NO_TINYUSB
	#include "tusb.h"
#else
	#define TU_LOG1(...)
	#define TU_LOG2(...)
#endif

//--------------------------------------------------------------------+
//FUNCTION DEFINITION
//--------------------------------------------------------------------+
//--------------------------------------------------------------------+
// Timer
//--------------------------------------------------------------------+
//Start systick timer
void board_timer_start(uint32_t ms)
{
	SysTick_Config((SystemCoreClock/1000) * ms);
}

//Stop systick timer
void board_timer_stop(void)
{
	SysTick->CTRL = 0;
}

//SysTick Handler
void SysTick_Handler (void)
{
	board_timer_handler();
}

//--------------------------------------------------------------------+
// LED 
//--------------------------------------------------------------------+
//LED init
void board_led_init(void)
{
	//Define the init structure for the output led pin
	gpio_pin_config_t gpioe_pin31_config = 
	{
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
	
	//Port E Clock Gate Control: Clock enabled, PIOE_31 drive led pin
    CLOCK_EnableClock(kCLOCK_PortE);
    //Initialize GPIO functionality on pin PTE31 (pin 19)
    GPIO_PinInit(LED_PORT, LED_PIN, &gpioe_pin31_config);
    //PORTE31 (pin 19) is configured as PTE31
    PORT_SetPinMux(LED_PORT_IOMUX, LED_PIN, kPORT_MuxAsGpio);
}

//Set LED status:1.LED_ON 2.LED_OFF
void board_led_write(uint32_t state)
{
	GPIO_PinWrite(LED_PORT, LED_PIN, (state?(1 - LED_STATE_ON):LED_STATE_ON));
}

//Set RGB tricolored led status
void board_rgb_write(uint8_t const rgb[])
{
}

//Toggle LED status
void board_led_toggle(void)
{
	GPIO_PortToggle(LED_PORT, 1u << LED_PIN);
}

//--------------------------------------------------------------------+
// UART 
//--------------------------------------------------------------------+
//UART init
void board_uart_init(uint32_t baud_rate)
{
#ifdef UART_DEV
    //Define the init structure for lpuart
    lpuart_config_t config;
	
	//Set LPUART0 clock source as IRC 48MHz
	CLOCK_SetLpuart0Clock(IRC_48MHZ);
	//Lpuart0 Gate Control: Clock enabled
    CLOCK_EnableClock(kCLOCK_Lpuart0);
	//Port A Clock Gate Control: Clock enabled,
    CLOCK_EnableClock(kCLOCK_PortA);
	
	//PORTA1 (pin 23) is configured as LPUART0_RX 
    PORT_SetPinMux(UART_PORT, UART_RX_PIN, kPORT_MuxAlt2);
    //PORTA2 (pin 24) is configured as LPUART0_TX
    PORT_SetPinMux(UART_PORT, UART_TX_PIN, kPORT_MuxAlt2);
	//LPUART0 transmit data source and receive data source configuration
    SIM->SOPT5 = ((SIM->SOPT5 &
                   //Mask bits to zero which are setting 
                   (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK)))
                   //LPUART0 Transmit Data Source Select: LPUART0_TX pin.
                  | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_LPUART_TX)
                   //LPUART0 Receive Data Source Select: LPUART_RX pin.
                  | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX));
				  
	//Gets the default configuration structure
	LPUART_GetDefaultConfig(&config);
	//Set uart baudrate as baud_rate
    config.baudRate_Bps = baud_rate;
	//Enable transmitting
    config.enableTx     = true;
	//Enable receiving
    config.enableRx     = true;
    //UART init
    LPUART_Init(UART_DEV, &config, CLOCK_GetFreq(kCLOCK_McgIrc48MClk));
#endif
}

//Send characters to UART for debugging
int board_uart_write(void const * buf, int len)
{
#ifdef UART_DEV
	LPUART_WriteBlocking(UART_DEV, (const uint8_t *)buf, len);
	return len;
#else
	(void) buf; (void) len;
	return 0;
#endif
}

//Receive characters to UART for debugging
int board_uart_read(uint8_t* buf, int len)
{
#ifdef UART_DEV
	int status = 0;
	status = LPUART_ReadBlocking(UART_DEV, buf, len);
	return status;
#else
	(void) buf; (void) len;
	return 0;
#endif
}

//--------------------------------------------------------------------+
// BOARD 
//--------------------------------------------------------------------+
//Board reset
void board_reset(void)
{
	//System reset performed by NVIC
	NVIC_SystemReset();
}

//Board dfu complete by reset 
void board_dfu_complete(void)
{
	//System reset performed by NVIC
	NVIC_SystemReset();
}

//Init USB for DFU
void board_dfu_init(void)
{
  //Enable USB FS clock
  CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
}

//Board init
void board_init(void)
{
	//Init clock
	BOARD_BootClockRUN();
	//Updates the SystemCoreClock variable
	SystemCoreClockUpdate();
	//Disable systick
    board_timer_stop();
	
#ifdef LED_PIN
    //LED init
    board_led_init();
#endif

#if TUF2_LOG
    //UART init
	board_uart_init(UART_BAUDRATE);
#endif
}

//Determine whether the application is located in a reasonable memory address range
bool board_app_valid(void)
{
	volatile uint32_t const * app_vector = (volatile uint32_t const*) BOARD_FLASH_APP_START;

	//1st word is stack pointer (should be in SRAM region)

	//2nd word is application entry point (reset)
#if(BOARD_FLASH_START != 0)
	if(
		(app_vector[1] < BOARD_FLASH_START) || 
	    (app_vector[1] > BOARD_FLASH_END)
	  ) 
	{
		return false;
	}
#else
	if(
	    (app_vector[1] > BOARD_FLASH_END)
	  ) 
	{
		return false;
	}
#endif

	return true;
}

//Jump to application code
void board_app_jump(void)
{
	//application function prototype declaration
	typedef void (*app_entry_t)(void);
	//vectorTable point to start of user application 
	uint32_t *vectorTable = (uint32_t *)BOARD_FLASH_APP_START;
	//get stack pointer
	uint32_t sp = vectorTable[0];
	//get program counter
	uint32_t pc = vectorTable[1];
	
	//temp variables
	uint32_t s_stackPointer = 0;
	uint32_t s_applicationEntry = 0;
	app_entry_t s_application = 0;
	
	//set s_stackPointer as sp
	s_stackPointer = sp;
	//set s_applicationEntry as pc
	s_applicationEntry = pc;
	//change application entry digital value to function pointer type
	s_application = (app_entry_t)s_applicationEntry;
	
	//Disable Interrupts
	NVIC->ICER[0] = 0xFFFFFFFF;
	
	// Change MSP and PSP
	__set_MSP(s_stackPointer);
	__set_PSP(s_stackPointer);
	
	//Specify the position of the vector table
	SCB->VTOR = BOARD_FLASH_APP_START;
	
	//Jump to application
	s_application();
	
	//Should never reach here.
	__NOP();
}

//get chip UID 
uint8_t board_usb_get_serial(uint8_t serial_id[16])
{
	sim_uid_t uid;
	uint32_t index = 0;
	
	SIM_GetUniqueId(&uid);
	for(index = 0; index < 16;index++)
	{
		serial_id[index] = 0;
	}
	
	index = 0;
	serial_id[index++] = (uid.L & 0xFF);
	serial_id[index++] = ((uid.L >> 8) & 0xFF);
	serial_id[index++] = ((uid.L >> 16) & 0xFF);
	serial_id[index++] = ((uid.L >> 24) & 0xFF);
	
	serial_id[index++] = (uid.ML & 0xFF);
	serial_id[index++] = ((uid.ML >> 8) & 0xFF);
	serial_id[index++] = ((uid.ML >> 16) & 0xFF);
	serial_id[index++] = ((uid.ML >> 24) & 0xFF);
	
	serial_id[index++] = (uid.MH & 0xFF);
	serial_id[index++] = ((uid.MH >> 8) & 0xFF);
	
    return 16;
}

#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t * bootloader_bin, uint32_t bootloader_len)
{
  (void) bootloader_bin;
  (void) bootloader_len;
}
#endif

#ifndef BUILD_NO_TINYUSB
//Forward USB interrupt events to TinyUSB IRQ Handler
void USB0_IRQHandler(void)
{
  tud_int_handler(0);
}
#endif

