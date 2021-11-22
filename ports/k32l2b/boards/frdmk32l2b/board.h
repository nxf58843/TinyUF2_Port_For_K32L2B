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

#ifndef BOARD_H_
#define BOARD_H_

//--------------------------------------------------------------------+
// Button
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+
#define LED_PORT              GPIOE
#define LED_PORT_IOMUX        PORTE
#define LED_PIN               31
#define LED_STATE_ON          1
#define LED_STATE_OFF         (1-LED_STATE_ON)

//--------------------------------------------------------------------+
// RGB
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// USB UF2
//--------------------------------------------------------------------+
#define USB_VID           0x15A2
#define USB_PID           0x0073
#define USB_MANUFACTURER  "NXP"
#define USB_PRODUCT       "FRDM K32L2B"

#define UF2_PRODUCT_NAME  USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID      "FRDM-K32L2B3"
#define UF2_VOLUME_LABEL  "K32L2BBOOT"
#define UF2_INDEX_URL     "https://www.nxp.com/FRDM-K32L2B-EVK"

//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+
//Board Flash Size Of K32L2B311VLH0A, if you use other K32 parts, please set according to the actual flash size
#define BOARD_FLASH_SIZE     (0x40000U)

//K32L2B31VLH0A SRAM Memory Map
//SRAM_L: 0x1FFF_E000 – 0x1FFF_FFFF
//SRAM_U: 0x2000_0000 – 0x2000_5FFF
#define BOARD_SRAM_START 0x1FFFE000
#define BOARD_SRAM_END   0x20005FFF

//K32L2B31VLH0A FLASH Memory Map
//Block 0 (P-Flash) address range:0x0000_0000 – 0x0001_FFFF
//Block 1 (P-Flash) address range:0x0002_0000 – 0x0003_FFFF
#define BOARD_FLASH_START 0X00000000
#define BOARD_FLASH_END   0x0003FFFF
//--------------------------------------------------------------------+
// Clock Source
//--------------------------------------------------------------------+
#define IRC_48MHZ  0x01

//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+
#define UART_DEV                       LPUART0
#define UART_BAUDRATE                  115200
#define UART_PORT                      PORTA
#define UART_RX_PIN                    0x1U
#define UART_TX_PIN                    0x2U

#endif
