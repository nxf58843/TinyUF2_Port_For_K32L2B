# UF2 Family ID
UF2_FAMILY_ID = 0x00000000      
# Cross Compiler 
CROSS_COMPILE = arm-none-eabi-

# SDK File Directory
SDK_DIR = lib/nxp/mcux-sdk
#MCU File Directory
MCU_DIR = $(SDK_DIR)/devices/$(MCU)

# Port Compiler Flags
CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m0plus \
  -DCFG_TUSB_MCU=OPT_MCU_K32L2BXX

# Suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=unused-parameter -Wno-error=float-equal -Wno-error=unused-function

SRC_S += $(MCU_DIR)/gcc/startup_$(MCU_CORE).S

# Port source
SRC_C += \
	$(MCU_DIR)/system_$(MCU_CORE).c \
	$(MCU_DIR)/project_template/clock_config.c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(SDK_DIR)/drivers/gpio/fsl_gpio.c \
	$(SDK_DIR)/drivers/lpuart/fsl_lpuart.c \
    $(SDK_DIR)/drivers/flash/fsl_ftfx_flash.c \
	$(SDK_DIR)/drivers/flash/fsl_ftfx_controller.c \
	$(SDK_DIR)/drivers/flash/fsl_ftfx_cache.c \
	$(SDK_DIR)/drivers/flash/fsl_ftfx_flexnvm.c \
	$(SDK_DIR)/drivers/sim/fsl_sim.c
	
# Port include
INC += \
    $(TOP)/$(PORT_DIR) \
	$(TOP)/$(BOARD_DIR) \
	$(TOP)/$(SDK_DIR)/CMSIS/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/project_template \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(SDK_DIR)/drivers/sim \
	$(TOP)/$(SDK_DIR)/drivers/smc \
	$(TOP)/$(SDK_DIR)/drivers/gpio \
	$(TOP)/$(SDK_DIR)/drivers/port \
	$(TOP)/$(SDK_DIR)/drivers/flash \
	$(TOP)/$(SDK_DIR)/drivers/lpuart \
	$(TOP)/$(SDK_DIR)/drivers/common 

