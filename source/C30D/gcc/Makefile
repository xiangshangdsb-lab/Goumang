# ==============================================================================
#  Goumang C30D (STM32F407) —— arm-none-eabi-gcc 构建脚本
#
#  用法：
#     make            # 编译，产物在 build/ 下（elf / hex / bin / map）
#     make clean
#     make -j8
#
#  与 Keil 工程的差异只有三处：启动文件、FreeRTOS 移植层、newlib 适配层，
#  其余源文件、宏定义、头文件路径与 WHEELTEC.uvprojx 的 FreeRTOS 目标一致。
# ==============================================================================

TARGET   := Goumang_C30D_v3.10.0_gcc

# --- 工具链 -------------------------------------------------------------------
TOOLCHAIN ?=
PREFIX    ?= arm-none-eabi-
TOOLCHAIN_BIN := $(if $(strip $(TOOLCHAIN)),$(TOOLCHAIN)/)
CC        := $(TOOLCHAIN_BIN)$(PREFIX)gcc
AS        := $(TOOLCHAIN_BIN)$(PREFIX)gcc -x assembler-with-cpp
OBJCOPY   := $(TOOLCHAIN_BIN)$(PREFIX)objcopy
SIZE      := $(TOOLCHAIN_BIN)$(PREFIX)size

BUILD     := gcc/build
MKFILE    := $(lastword $(MAKEFILE_LIST))

# --- 器件 ---------------------------------------------------------------------
MCU       := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS      := -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -D__FPU_PRESENT=1 \
             -DARM_MATH_CM4 -DUSE_USB_OTG_FS

INCLUDES  := \
  -ICORE \
  -ISYSTEM/delay -ISYSTEM/sys -ISYSTEM/usart \
  -IUSER \
  -IFWLIB/inc \
  -IFreeRTOS/include \
  -IFreeRTOS/portable/GCC/ARM_CM4F \
  -IBALANCE \
  -IHARDWARE \
  -IAPP/bluetooth \
  -IAPP/control \
  -IAPP/line \
  -IAPP/pan \
  -IAPP/storage \
  -IHARDWARE/analog \
  -IHARDWARE/display \
  -IHARDWARE/io \
  -IHARDWARE/motion \
  -IHARDWARE/storage \
  -IHARDWARE/MPU6050 \
  -IHARDWARE/ICM20948 \
  -IHARDWARE/USB/STM32_USB_Host_Library/Class/HID/Inc \
  -IHARDWARE/USB/STM32_USB_Host_Library/Core/Inc \
  -IHARDWARE/USB/USB_HOST/App \
  -IHARDWARE/USB/USB_HOST/Target \
  -Igcc

# --- 编译选项 -----------------------------------------------------------------
# -funsigned-char : ARMCC 默认把 char 当无符号，GCC 默认有符号，必须对齐，
#                   否则协议里按 char 做的范围判断会变。
OPT       := -O2 -g
# -Wno-unused-function : ICM20948.h 里有一批 `static` 原型只在本文件内定义，
#                        其它 TU 包含该头文件时会刷 "declared 'static' but never
#                        defined"，属厂商头文件遗留问题，不影响生成代码。
CFLAGS    := $(MCU) $(DEFS) $(INCLUDES) $(OPT) \
             -ffunction-sections -fdata-sections \
             -funsigned-char -fcommon \
             -std=gnu99 -Wall -Wno-unused-function \
             -include gcc/gcc_compat.h \
             -MMD -MP

ASFLAGS   := $(MCU) $(DEFS) $(INCLUDES) -g
LDFLAGS   := $(MCU) $(OPT) \
             -Tgcc/STM32F407.ld \
             -specs=nano.specs \
             -nostartfiles \
             -Wl,--gc-sections \
             -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
             -Wl,--print-memory-usage \
             -lm

# --- 源文件 -------------------------------------------------------------------
C_SOURCES := \
  USER/main.c \
  USER/stm32f4xx_it.c \
  USER/system_stm32f4xx.c \
  \
  HARDWARE/analog/adc.c \
  HARDWARE/motion/encoder.c \
  HARDWARE/motion/motor.c \
  APP/control/s31_link.c \
  APP/protocol/s31_protocol.c \
  HARDWARE/storage/stmflash.c \
  APP/line/line_fusion.c \
  HARDWARE/analog/tsl1401.c \
  APP/bluetooth/bluetooth_control.c \
  APP/bluetooth/bluetooth_link.c \
  HARDWARE/io/I2C.c \
  HARDWARE/display/oled_min.c \
  HARDWARE/display/oledfont_min.c \
  HARDWARE/ICM20948/ICM20948.c \
  HARDWARE/MPU6050/MPU6050.c \
  \
  SYSTEM/delay/delay.c \
  SYSTEM/sys/sys.c \
  \
  FWLIB/src/misc.c \
  FWLIB/src/stm32f4xx_gpio.c \
  FWLIB/src/stm32f4xx_rcc.c \
  FWLIB/src/stm32f4xx_usart.c \
  FWLIB/src/stm32f4xx_adc.c \
  FWLIB/src/stm32f4xx_tim.c \
  \
  FreeRTOS/list.c \
  FreeRTOS/queue.c \
  FreeRTOS/tasks.c \
  FreeRTOS/timers.c \
  FreeRTOS/portable/GCC/ARM_CM4F/port.c \
  FreeRTOS/portable/MemMang/heap_4.c \
  \
  gcc/retarget.c

ASM_SOURCES := gcc/startup_stm32f40_41xxx_gcc.S

# --- 规则 ---------------------------------------------------------------------
# Keep source paths in object names so modules can have identical basenames.
SOURCES := $(C_SOURCES) $(ASM_SOURCES)
OBJECTS := $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(SOURCES))))

.PHONY: all clean size

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c $(MKFILE)
	@mkdir -p $(@D)
	@echo "  CC   $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.C $(MKFILE)
	@mkdir -p $(@D)
	@echo "  CC   $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.S $(MKFILE)
	@mkdir -p $(@D)
	@echo "  AS   $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS) gcc/STM32F407.ld
	@echo "  LD   $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SIZE) $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O binary -S $< $@

size: $(BUILD)/$(TARGET).elf
	@$(SIZE) -A -x $<

clean:
	@rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
