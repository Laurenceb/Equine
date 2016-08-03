# Compile the project

# Uncomment the appropriate device type and startup file
DEVICE_TYPE = STM32F10X_MD

# Set the external clock frequency
HSE_VALUE = 12288000UL

# Enable debug compilation - this can also be enabled with D as last argument to make
#DEBUG = 1

# [OPTIONAL] Set the serial details for bootloading
STM32LDR_PORT = /dev/rfcomm0
STM32LDR_BAUD = 115200
# [OPTIONAL] Comment out to disable bootloader verification
STM32LDR_VERIFY = -v

# [OPTIONAL] Uncomment to use the firmware library
FWLIB = lib/STM32F10x_StdPeriph_Driver/libstm32fw.a
# [OPTIONAL] Uncomment to use the USB library
USBLIB = lib/STM32_USB-FS-Device_Driver/libstm32usb.a

# [OPTIONAL] Uncomment to link to maths library libm
LIBM = -lm

export DEBUG
export MESSAGES

TARGET_ARCH = -mcpu=cortex-m3 -mthumb

INCLUDE_DIRS = -I . -I lib/STM32F10x_StdPeriph_Driver/inc\
 -I lib/CMSIS_CM3 -I lib/STM32F10x_StdPeriph_Driver\
 -I lib/STM32_USB-FS-Device_Driver/inc -I Util -I Sensors -I Util/USB\
 -I Util/fat_fs/inc

LIBRARY_DIRS = -L lib/STM32F10x_StdPeriph_Driver/\
 -L lib/STM32_USB-FS-Device_Driver

CRT_VERSION="\"$(shell hg id)\""

DEFINES = -D$(DEVICE_TYPE) -DHSE_Value=$(HSE_VALUE) -DCRT -DBOARD=3 -DCRT_VERSION=$(CRT_VERSION) #-DSYSCLK_FREQ_72MHz=72000000

# If the first argument is "upload"...
ifeq (upload,$(firstword $(MAKECMDGOALS)))
  # use the rest as arguments for "run"
  UPLOAD_ARGS := $(word 2,$(MAKECMDGOALS))
  # ...and turn them into do-nothing targets
  $(eval $(UPLOAD_ARGS):;@:)
  STM32LDR_PORT = /dev/rfcomm$(UPLOAD_ARGS)
  $(info port set to: /dev/rfcomm$(UPLOAD_ARGS))
endif

# If the last argument is "D", force debug build on
ifeq (D,$(lastword $(MAKECMDGOALS)))
  DEBUG := 1
  $(eval $(DEBUG):;@:)
  $(info running debug build, $(DEBUG))
endif

# Passing BUILD_VERBOSE=1 to make will give a verbose output from make 
ifeq ("$(BUILD_VERBOSE)","1")
Q :=
vecho = true
else
Q := @
vecho = false
endif

COMPILE_OPTS = $(WARNINGS) $(TARGET_OPTS) $(MESSAGES) $(INCLUDE_DIRS) $(DEFINES) $(TARGET_ARCH)
WARNINGS = -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -Winline

ifdef DEBUG
 TARGET_OPTS = -O3 -g3 -fuse-linker-plugin -fno-common -flto -ggdb -DDEBUG
else	#Changed from O2 - optimisation split between control loop and rest of project, using a seperate makefile
 TARGET_OPTS = $(OPTIMISE) -flto -finline -finline-functions-called-once -fuse-linker-plugin\
  -funroll-loops -fno-common -fno-exceptions -ffunction-sections
endif

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
SIZE = arm-none-eabi-size

CFLAGS = -std=gnu99 $(COMPILE_OPTS)
CXXFLAGS = $(COMPILE_OPTS)

AS = $(CC) -x assembler-with-cpp -c $(TARGET_ARCH)
ASFLAGS = $(COMPILE_OPTS)

LD = $(CC)
LDFLAGS = -Wl,--gc-sections,-Map=$(MAIN_MAP),-cref -T lib/ARM-GCC/sections.ld -L lib\
 $(INCLUDE_DIRS) $(LIBRARY_DIRS) $(LIBM) -ffunction-sections -lnosys -specs=nano.specs -specs=nosys.specs#-specs=nosys.specs #-fuse-linker-plugin#-lstdc++
ifndef DEBUG
LDFLAGS += -flto -Os
endif

AR = arm-none-eabi-ar
ARFLAGS = cr

OBJCOPY = arm-none-eabi-objcopy
OBJCOPYFLAGS = -O binary

STARTUP_OBJ = lib/ARM-GCC/startup_ARMCM3.o

MAIN_OUT = main.elf
MAIN_MAP = $(MAIN_OUT:%.elf=%.map)
MAIN_BIN = $(MAIN_OUT:%.elf=%.bin)

MAIN_OBJS = $(sort \
 $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
 $(patsubst %.cc,%.o,$(wildcard *.cc)) \
 $(patsubst %.c,%.o,$(wildcard *.c)) \
 $(patsubst %.s,%.o,$(wildcard *.s)) \
 $(patsubst %.c,%.o,$(wildcard lib/CMSIS_CM3/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/USB/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/fat_fs/src/*.c)) \
 $(patsubst %.c,%.o,$(wildcard Util/fat_fs/option/ccsbcs.c)) \
 $(STARTUP_OBJ))

FAST_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard Sensors/*.c))\
 $(patsubst %.s,%.o,$(wildcard Sensors/*.s)) \
)

#optimisation
$(MAIN_OBJS): OPTIMISE= -Os

$(FAST_OBJS): OPTIMISE= -O3

#all - output the size from the elf
.PHONY: all
all: $(MAIN_BIN)
	$(SIZE) $(MAIN_OUT)

# main
%.o: %.c
ifeq ($(vecho),false)
	$(info -> Compiling $@)
endif
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

$(MAIN_OUT): $(MAIN_OBJS) $(FAST_OBJS) $(FWLIB) $(USBLIB)
ifeq ($(vecho),false)
	$(info -> Linking $@)
endif
	$(Q)$(LD) $(CFLAGS) $(TARGET_ARCH) $^ -o $@ $(LDFLAGS)

$(MAIN_OBJS): $(wildcard *.h) $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)\
 $(wildcard lib/CMSIS_CM3/*.h)\
 $(wildcard Util*.h)\
 $(wildcard Util/USB*.h)\
 $(wildcard Util/fat_fs/inc/*.h)\
 $(wildcard Sensors*.h)

$(MAIN_BIN): $(MAIN_OUT)
	$(OBJCOPY) $(OBJCOPYFLAGS) $< $@

# fwlib

.PHONY: fwlib
fwlib: $(FWLIB)

$(FWLIB): $(wildcard lib/STM32F10x_StdPeriph_Driver/*.h)\
 $(wildcard lib/STM32F10x_StdPeriph_Driver/inc/*.h)
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE)

# usblib

.PHONY: usblib
usblib: $(USBLIB)

$(USBLIB): $(wildcard lib/STM32_USB-FS-Device_Driver/inc*.h)
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE)

#size

.PHONY: size
size: all
	@echo "Size:"
	$(SIZE) $(MAIN_OUT) $@
	@$(CAT) $@

# flash

.PHONY: flash
flash: flash-elf
#flash: flash-bin

.PHONY: flash-elf
flash-elf: all
	@cp $(MAIN_OUT) jtag/flash.elf
	@cd jtag && openocd -f flash-elf.cfg
	@rm jtag/flash.elf

.PHONY: flash-bin
flash-bin: all
	@cp $(MAIN_BIN) jtag/flash.bin
	@cd jtag && openocd -f flash-bin.cfg
	@rm jtag/flash.bin

.PHONY: upload
upload: all
	@python jtag/stm32loader.py -p $(STM32LDR_PORT) -b $(STM32LDR_BAUD)\
    -e $(STM32LDR_VERIFY) -w main.bin

.PHONY: version
version:
	rm main.o
	$(MAKE)

# clean

.PHONY: clean
clean:
	-rm -f $(MAIN_OBJS) $(MAIN_OUT) $(MAIN_MAP) $(MAIN_BIN)
	-rm -f $(FAST_OBJS)
	-rm -f jtag/flash.elf jtag/flash.bin

.PHONY: cleanlibs
cleanlibs:
	@cd lib/STM32F10x_StdPeriph_Driver && $(MAKE) clean
	@cd lib/STM32_USB-FS-Device_Driver && $(MAKE) clean

