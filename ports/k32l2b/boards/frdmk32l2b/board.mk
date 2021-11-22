MCU = K32L2B31A
MCU_CORE = $(MCU)
CFLAGS += -DCPU_K32L2B31VLH0A
            
# For flash-pyocd
PYOCD_TARGET = $(MCU)

# For flash-jlink target
JLINK_DEVICE = CPU_K32L2B31VLH0A

flash: flash-pyocd
erase: flash-pyocd
