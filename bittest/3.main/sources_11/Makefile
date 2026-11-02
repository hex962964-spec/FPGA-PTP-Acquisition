TARGET = net_acq_ptp_closed_loop

SIMU ?= 0

CFLAGS += -O2 -g
ifeq ($(SIMU),1)
CFLAGS += -DSIMU
endif

C_SRCS := ./main.c ./net_acq_hal.c ./ptp_link_recovery.c ./ptp_time_quality.c ./ptp_fast50_servo.c

OBJDIR = obj
COMMON_DIR = ../../bsp
GCC_DIR = ../../../toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0
PICOLIBC_DIR = ../../../toolchains/picolibc
include ../../bsp/common.mk
