ARCH= cortex-m0plus
PART= samd11d14am
TARGET_FLASH= piezo-sensor.elf
PROJECT_TYPE= flash

CSRCS= \
       main.c                                             \
       common/services/usb/class/cdc/device/udi_cdc.c     \
       common/services/usb/class/cdc/device/udi_cdc_desc.c \
       common/services/usb/udc/udc.c                      \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       sam0/drivers/ac/ac_sam_d_r_h/ac.c                  \
       sam0/drivers/tc/tc_sam_d_r_h/tc.c                  \
       sam0/drivers/sercom/sercom.c                       \
       sam0/drivers/sercom/usart/usart.c                  \
       sam0/drivers/sercom/i2c/i2c_sam0/i2c_master.c      \
       sam0/drivers/port/port.c                           \
       sam0/drivers/system/clock/clock_samd09_d10_d11/clock.c \
       sam0/drivers/system/clock/clock_samd09_d10_d11/gclk.c \
       sam0/drivers/system/interrupt/system_interrupt.c   \
       sam0/drivers/system/pinmux/pinmux.c                \
       sam0/drivers/system/system.c                       \
       sam0/drivers/usb/stack_interface/usb_device_udd.c  \
       sam0/drivers/usb/stack_interface/usb_dual.c        \
       sam0/drivers/usb/usb_sam_d_r/usb.c                 \
       sam0/utils/cmsis/samd11/source/gcc/startup_samd11.c \
       sam0/utils/cmsis/samd11/source/system_samd11.c     \
       sam0/utils/syscalls/gcc/syscalls.c

INC_PATH = \
       common/boards                                      \
       common/utils                                       \
       sam0/boards                                        \
       common/services/sleepmgr                           \
       common/services/usb                                \
       common/services/usb/class/cdc                      \
       common/services/usb/class/cdc/device               \
       common/services/usb/udc                            \
       sam0/drivers/ac                                    \
       sam0/drivers/adc                                   \
       sam0/drivers/adc/adc_sam_d_r_h                     \
       sam0/drivers/tc                                    \
       sam0/drivers/extint                                \
       sam0/drivers/extint/extint_sam_d_r_h               \
       sam0/drivers/sercom                                \
       sam0/drivers/sercom/usart                          \
       sam0/drivers/sercom/i2c                            \
       sam0/drivers/events                                \
       sam0/drivers/port                                  \
       sam0/drivers/system                                \
       sam0/drivers/system/clock                          \
       sam0/drivers/system/clock/clock_samd09_d10_d11     \
       sam0/drivers/system/interrupt                      \
       sam0/drivers/system/interrupt/system_interrupt_samd10_d11 \
       sam0/drivers/system/pinmux                         \
       sam0/drivers/system/power                          \
       sam0/drivers/system/power/power_sam_d_r_h          \
       sam0/drivers/system/reset                          \
       sam0/drivers/system/reset/reset_sam_d_r_h          \
       sam0/drivers/usb                                   \
       sam0/drivers/usb/stack_interface                   \
       sam0/drivers/usb/usb_sam_d_r                       \
       sam0/utils                                         \
       sam0/utils/cmsis/samd11/include                    \
       sam0/utils/cmsis/samd11/source                     \
       sam0/utils/header_files                            \
       sam0/utils/preprocessor                            \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC

LINKER_SCRIPT_FLASH = sam0/utils/linker_scripts/samd11/gcc/samd11d14am_flash.ld
DEBUG_SCRIPT_FLASH = ../flash.gdb

OPTIMIZATION = -Og

CPPFLAGS = \
       -D ARM_MATH_CM0PLUS=true                           \
       -D __SAMD11D14AM__                                 \
       -D AC_CALLBACK_MODE=false -DTC_ASYNC=false         \
       -D ADC_CALLBACK_MODE=false                         \
       -D EVENTS_INTERRUPT_HOOKS_MODE=false               \
       -D USART_CALLBACK_MODE=false                       \
       -D EXTINT_CALLBACK_MODE=false                      \
       -D I2C_MASTER_CALLBACK_MODE=false                  \
       -Wall -I.
