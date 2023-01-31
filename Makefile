io=uart
PMSIS_OS = freertos

ifeq ($(SETUP_WIFI_AP), 1)
APP_CFLAGS += -DSETUP_WIFI_AP
endif

APP = velocity_sender
APP_SRCS += velocity_sender.c inc/lib/cpx/src/com.c inc/lib/cpx/src/cpx.c
APP_CFLAGS += -O3 -g
APP_INC=inc/lib/cpx/inc
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1


RUNNER_CONFIG = $(CURDIR)/config.ini


include $(RULES_DIR)/pmsis_rules.mk
