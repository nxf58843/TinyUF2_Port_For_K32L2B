# List of git submodules that is included as part of the UF2 version
GIT_SUBMODULES = nxp/mcux-sdk sct_neopixel tinyusb

# List of mk files that are included as part of this project
include ../make.mk
include port.mk


# Source Files
SRC_C += \
	$(PORT_DIR)/boards.c \
	$(PORT_DIR)/board_flash.c
	
ifndef BUILD_NO_TINYUSB
SRC_C += lib/tinyusb/src/portable/nxp/khci/dcd_khci.c
endif

include ../rules.mk

# Linker script
LD_FILES ?= $(MCU_DIR)/gcc/K32L2B31xxxxA_flash.ld

#-------------- Self-update  --------------
# Self-update is not implemented yet
SELF_DIR = apps/self_update

$(SELF_DIR)/bootloader_bin.c:	$(BUILD)/$(OUTNAME).bin
	$(PYTHON3) $(TOP)/lib/uf2/utils/uf2conv.py --carray $^ -o $@

self-update: $(SELF_DIR)/bootloader_bin.c
	make -C $(SELF_DIR) BOARD=$(BOARD) self-update
	@rm $^
