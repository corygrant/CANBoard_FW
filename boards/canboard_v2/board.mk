# List of all the board related files.
BOARDSRC = ./boards/canboard_v2/board.c

# Required include directories
BOARDINC = ./boards/canboard_v2

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)

include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f3xx.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F3xx/platform.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk