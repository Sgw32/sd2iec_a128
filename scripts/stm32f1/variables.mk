ASMSRC = stm32f1/startup.S
SRC += stm32f1/iec-bus.c

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
NM = arm-none-eabi-nm

MCU = cortex-m3
FORMAT = ihex

ARCH_CFLAGS = -mthumb -mcpu=$(MCU)
ARCH_ASFLAGS = -mthumb -mcpu=$(MCU)
ARCH_LDFLAGS = -mcpu=$(MCU) -mthumb -nostartfiles
ARCH_LDFLAGS += -Tscripts/stm32f1/stm32f103c8.ld
ARCH_LDFLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

ELFSIZE = $(SIZE) $(TARGET).elf
