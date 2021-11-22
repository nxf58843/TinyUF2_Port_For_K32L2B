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
#include "tusb.h" 
#include "fsl_flash.h"

//Macro definitions
#define TEST_FLASH_API        0
#define NO_CACHE              0xffffffff
#define FLASH_SECTOR_SIZE     (1024)

//Variable definitions
//Flash driver structure
static flash_config_t s_flashDriver;
//Flash cache driver structure
static ftfx_cache_config_t s_cacheDriver;
//Flash block size
uint32_t pflashBlockBase  = 0;
//Flash total size
uint32_t pflashTotalSize  = 0;
//Flash sector size
uint32_t pflashSectorSize = 0;
//Flash security status
ftfx_security_state_t securityStatus = kFTFx_SecurityStateNotSecure; 
//Flash sector address
static uint32_t _flash_sector_addr = NO_CACHE;
//Flash cache
static uint8_t  _flash_cache[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

//--------------------------------------------------------------------+
//Board Flash APIs
//--------------------------------------------------------------------+
//error handle
static void error_trap(void)
{
    while (1)
    {
    }
}

//Get board flash size
uint32_t board_flash_size(void)
{
  return BOARD_FLASH_SIZE;
}

//Flash read
void board_flash_read(uint32_t addr, void* buffer, uint32_t len)
{
	uint8_t * p_flash = (uint8_t *)addr;
	uint8_t * p_buffer = (uint8_t *)buffer;
	uint32_t index = 0;
	
	if(buffer != NULL)
	{
		for(index = 0;index < len;index++)
		{
			p_buffer[index] = p_flash[index];
		}
	}
}

//Flash erase
static status_t board_flash_erase(uint32_t addr, uint32_t len)
{
	//Return code from each flash driver function
	status_t result;   

	//Call Flash driver to erase flash
	FLASH_Erase(&s_flashDriver, addr, len, kFTFx_ApiEraseKey);
	//Verify flash if it's been erased
    result = FLASH_VerifyErase(&s_flashDriver, addr, len, kFTFx_MarginValueUser);
	
	//return flash erase status
	return result;
}

//Flash program
static status_t board_flash_program(uint32_t addr, uint8_t *src, uint32_t lengthInBytes)
{
	//Return code from each flash driver function
	status_t result;   
	//Fail address and fail data
	uint32_t failAddr, failDat;
	
	//program user buffer into flash
    result = FLASH_Program(&s_flashDriver, addr, src, lengthInBytes);
    if (kStatus_FTFx_Success != result)
    {
		return result;
    }

    //Verify programming by Program Check command with user margin levels
    result = FLASH_VerifyProgram(&s_flashDriver, addr, lengthInBytes, src, kFTFx_MarginValueUser, &failAddr, &failDat);
    return result;
}

//Flash flush
void board_flash_flush(void)
{
  //Return code from each flash driver function
  status_t status;
  //Address and data for fail flash operation
  uint32_t failedAddress, failedData;
  
  //Judge whether flash has cache
  if(_flash_sector_addr == NO_CACHE) 
  {
	  return;
  }
  
  //check whether data to be programmed is equal to cache, if it is, no need to program
  status = FLASH_VerifyProgram(
                               &s_flashDriver, 
							   _flash_sector_addr, 
							   FLASH_SECTOR_SIZE, 
							   (const uint8_t *)_flash_cache, 
							   kFTFx_MarginValueUser,
							   &failedAddress, 
							   &failedData
							  );
  //need program
  if (status != kStatus_Success) {
    TU_LOG1("Erase and Write at address = 0x%08lX\r\n",_flash_sector_addr);
    status = FLASH_Erase(&s_flashDriver, _flash_sector_addr, FLASH_SECTOR_SIZE, kFTFx_ApiEraseKey);
    status = FLASH_Program(&s_flashDriver, _flash_sector_addr, _flash_cache, FLASH_SECTOR_SIZE);
  }

  _flash_sector_addr = NO_CACHE;
}

//Flash write
void board_flash_write (uint32_t addr, void const *data, uint32_t len)
{
  //translate addr to be aligned with sector size
  uint32_t newAddr = addr & ~(FLASH_SECTOR_SIZE - 1);
  //data pointer
  uint8_t* p_cache = NULL;
  uint8_t* p_data = NULL;
  
  //flash write 
  if(newAddr != _flash_sector_addr) 
  {
	//flash flush
    board_flash_flush();
	//update _flash_sector_addr
    _flash_sector_addr = newAddr;
	//read flash
    board_flash_read(newAddr, _flash_cache, FLASH_SECTOR_SIZE);
  }
  
  p_data  = (uint8_t *)data;
  p_cache = (uint8_t *)(_flash_cache + (addr & (FLASH_SECTOR_SIZE - 1)));
  for(unsigned int index = 0; index < len;index++)
  {
	  p_cache[index] = p_data[index];
  }
  //memcpy(_flash_cache + (addr & (FLASH_PAGE_SIZE - 1)), data, len);
}

//Flash erase application
void board_flash_erase_app(void)
{
  // TODO implement later
}

#ifdef TINYUF2_SELF_UPDATE
//Board self update
void board_self_update(const uint8_t * bootloader_bin, uint32_t bootloader_len)
{
  (void) bootloader_bin;
  (void) bootloader_len;
}
#endif

#if TEST_FLASH_API
//Flash API test
static void flash_api_test(void)
{
	uint8_t buffer[100]={0};
	char buf[100];
	uint32_t flash_start_addr = 15360;
	
	sprintf(buf,"enter flash_api_test function:%u\n",(unsigned int)flash_api_test);
	board_uart_write(buf, strlen(buf));
	
	sprintf(buf,"board flash read test(start address:%u):%u\n",(unsigned int)flash_start_addr,(unsigned int)board_flash_read);
	board_uart_write(buf, strlen(buf));
	board_flash_read(flash_start_addr,buffer,100);
	for(int i = 0;i < 100;i++)
	{
		sprintf(buf,"buffer[%d]:%x\n",i,buffer[i]);
	    board_uart_write(buf, strlen(buf));
	}
	
    sprintf(buf,"board flash erase test(start address:%u):%u\n",(unsigned int)flash_start_addr,(unsigned int)board_flash_erase);
	board_uart_write(buf, strlen(buf));
	board_flash_erase(flash_start_addr,FLASH_SECTOR_SIZE);
	
	board_flash_read(flash_start_addr,buffer,100);
	for(int i = 0;i < 100;i++)
	{
		sprintf(buf,"buffer[%d]:%x\n",i,buffer[i]);
	    board_uart_write(buf, strlen(buf));
	}
	
	for(int i = 0;i < 100;i++)
	{
		buffer[i] = i+4; 
	}
	sprintf(buf,"board flash program test(start address:%u):%u\n",(unsigned int)flash_start_addr,(unsigned int)board_flash_program);
	board_uart_write(buf, strlen(buf));
	board_flash_program(flash_start_addr, buffer, 100);
	for(int i = 0;i < 100;i++)
	{
		buffer[i] = 0; 
	}
	
	board_flash_read(flash_start_addr,buffer,100);
	for(int i = 0;i < 100;i++)
	{
		sprintf(buf,"buffer[%d]:%d\n",i,buffer[i]);
	    board_uart_write(buf, strlen(buf));
	}
}

//Print flash information
static void board_flash_info(void)
{
	//Print bufffer
	char buf[100];
	
	//Print flash total size
	sprintf(buf,"pflashTotalSize:%lu\n",pflashTotalSize);
	board_uart_write(buf, strlen(buf));
	//Print flash block base
	sprintf(buf,"pflashBlockBase:%lu\n",pflashBlockBase);
	board_uart_write(buf, strlen(buf));
	//Print flash sector size
	sprintf(buf,"pflashSectorSize:%lu\n",pflashSectorSize);
	board_uart_write(buf, strlen(buf));
	
	//Print security status
    switch (securityStatus)
    {
        case kFTFx_SecurityStateNotSecure:
            sprintf(buf,"kFTFx_SecurityStateNotSecure:%d\n",kFTFx_SecurityStateNotSecure);
	        board_uart_write(buf, strlen(buf));
            break;
        case kFTFx_SecurityStateBackdoorEnabled:
            sprintf(buf,"kFTFx_SecurityStateBackdoorEnabled:%d\n",kFTFx_SecurityStateBackdoorEnabled);
	        board_uart_write(buf, strlen(buf));
            break;
        case kFTFx_SecurityStateBackdoorDisabled:
            sprintf(buf,"kFTFx_SecurityStateBackdoorDisabled:%d\n",kFTFx_SecurityStateBackdoorDisabled);
	        board_uart_write(buf, strlen(buf));
            break;
        default:
            break;
    }
}
#endif

//Initialize flash for DFU
void board_flash_init(void)
{
	//Return code from each flash driver function
	int32_t result;   
	 
	//Clean up Flash, Cache driver Structure
    memset(&s_flashDriver, 0, sizeof(flash_config_t));
    memset(&s_cacheDriver, 0, sizeof(ftfx_cache_config_t));
	
	//Setup flash driver structure for device and initialize variables. 
    result = FLASH_Init(&s_flashDriver);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
	
    //Setup flash cache driver structure for device and initialize variables. 
    result = FTFx_CACHE_Init(&s_cacheDriver);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
	
    //Get flash properties
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
    FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);
	
	//Check security status
    result = FLASH_GetSecurityState(&s_flashDriver, &securityStatus);
    if (kStatus_FTFx_Success != result)
    {
        error_trap();
    }
	
	#if TEST_FLASH_API
	board_flash_info();
	flash_api_test();
	#endif
}
