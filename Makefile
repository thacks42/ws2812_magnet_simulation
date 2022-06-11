PROJECT = ws2812barlight
BUILD_DIR = bin

SHARED_DIR = /home/michael/documents/cpp/include
#CFILES = main.c
CPPFILES = main.cpp
#LDLIBS += -lm
#CFILES = vgaout.c main-cube.c libfixmath/libfixmath/fix16.c libfixmath/libfixmath/fix16_trig.c
#AFILES += api-asm.S
#INCLUDES += $(patsubst %,-I%, . ./libfixmath/libfixmath/)

#CFLAGS += -DFIXMATH_NO_OVERFLOW
#CFLAGS += -DFIXMATH_NO_ROUNDING
#CFLAGS += -DFIXMATH_NO_CACHE
#CFLAGS += -DFIXMATH_FAST_SIN

CFLAGS += -Wall

# TODO - you will need to edit these two lines!
DEVICE=stm32f103c8t6
OOCD_FILE = board/stm32f103c8_blue_pill.cfg
OOCD_INTERFACE = stlink-v2.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
