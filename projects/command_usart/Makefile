# Put your source files here (or *.c, etc)
SRCS=main.c ../../common/src/*.c ../../drivers/src/*.c

# Library modules
SRCS += stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c misc.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=command_usart

# Put your STM32F4 library code directory here
STM_COMMON=../../STM32F4-Discovery_FW_V1.1.0

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -T../../common/stm32_flash.ld -std=c99
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs
CFLAGS += -I../../common/include
CFLAGS += -I../../drivers/include

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc

# add startup file to build
SRCS += $(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s 
OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	mkdir -p bin
	$(CC) $(CFLAGS) $^ -o bin/$@ 
	$(OBJCOPY) -O ihex bin/$(PROJ_NAME).elf bin/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary bin/$(PROJ_NAME).elf bin/$(PROJ_NAME).bin

clean:
	rm -fr bin/

# Flash the STM32F4
flash: proj
	qstlink2 -cwV bin/$(PROJ_NAME).bin
