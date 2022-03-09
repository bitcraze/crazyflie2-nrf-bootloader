
#Put your personal build config in config.mk and DO NOT COMMIT IT!
-include config.mk

BLE ?= 1

CROSS_COMPILE=arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)gcc
GDB=$(CROSS_COMPILE)gdb

OPENOCD           ?= openocd
OPENOCD_DIR       ?= 
OPENOCD_INTERFACE ?= $(OPENOCD_DIR)interface/stlink-v2.cfg
OPENOCD_TARGET    ?= target/nrf51_stlink.tcl
OPENOCD_CMDS      ?=

O                 ?= -Os


USES_RFX2411N     ?= 0
USES_TI_CHARGER   ?= 1

NRF51_SDK ?= nrf51_sdk/nrf51822
NRF_S110 ?= s110

INCLUDES = -I include/nrf -I $(NRF_S110)/s110_nrf51822_7.3.0_API/include -I include/cm0 -I include/

PROCESSOR = -mcpu=cortex-m0 -mthumb
NRF= -DNRF51

ifeq ($(USES_RFX2411N), 1)
PROGRAM=cload_nrf_rfx2411n
else
PROGRAM=cload_nrf
endif

CFLAGS=$(PROCESSOR) $(NRF) $(INCLUDES) -g3 $(O) -Wall -Werror# -ffunction-sections -fdata-sections
# --specs=nano.specs -flto
ASFLAGS=$(PROCESSOR)
LDFLAGS=$(PROCESSOR) $(O) --specs=nano.specs -Wl,-Map=$(PROGRAM).map # -Wl,--gc-sections
ifdef SEMIHOSTING
LDFLAGS+= --specs=rdimon.specs -lc -lrdimon
CFLAGS+= -DSEMIHOSTING
endif

ifdef BLE
LDFLAGS += -T gcc_nrf51_s110_xxaa.ld 
else
LDFLAGS += -T gcc_nrf51_blank_xxaa.ld 
endif

# BLE settings

OBJS += $(NRF51_SDK)/Source/ble/ble_advdata.o
OBJS += $(NRF51_SDK)/Source/ble/ble_conn_params.o
OBJS += $(NRF51_SDK)/Source/ble/ble_services/ble_srv_common.o
OBJS += $(NRF51_SDK)/Source/ble/ble_services/ble_dis.o
OBJS += $(NRF51_SDK)/Source/sd_common/softdevice_handler.o
OBJS += $(NRF51_SDK)/Source/app_common/app_timer.o



CFLAGS += -DBLE_STACK_SUPPORT_REQD -DNRF51
CFLAGS += -I$(NRF51_SDK)/Include/gcc
CFLAGS += -I$(NRF51_SDK)/Include/ 
CFLAGS += -I$(NRF51_SDK)/Include/ble/ 
CFLAGS += -I$(NRF51_SDK)/Include/ble/ble_services/ 
CFLAGS += -I$(NRF_S110)/include
CFLAGS += -I$(NRF51_SDK)/Include/app_common/ 
CFLAGS += -I$(NRF51_SDK)/Include/sd_common/ 

ifeq ($(USES_RFX2411N), 1)
CFLAGS += -DHAS_RFX2411N
endif

ifeq ($(USES_TI_CHARGER), 1)
CFLAGS += -DHAS_TI_CHARGER
endif

OBJS += src/main.o gcc_startup_nrf51.o system_nrf51.o src/button.o
OBJS += src/systick.o src/ble.o src/ble_crazyflies.o src/esb.o src/timeslot.o
OBJS += src/bootloader.o src/uart.o src/syslink.o src/crc.o

all: $(PROGRAM).elf $(PROGRAM).bin $(PROGRAM).hex
	arm-none-eabi-size $(PROGRAM).elf

$(PROGRAM).hex: $(PROGRAM).elf
	arm-none-eabi-objcopy $^ -O ihex $@

$(PROGRAM).bin: $(PROGRAM).elf
	arm-none-eabi-objcopy $^ -O binary $@

$(PROGRAM).elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^

clean:
	rm -f $(PROGRAM).bin $(PROGRAM).elf $(OBJS)
	
flash: $(PROGRAM).hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROGRAM).hex" -c "verify_image $(PROGRAM).hex" \
                 -c "mww 0x4001e504 0x01" -c "mww 0x10001080 0x3A000" -c "reset run" -c shutdown

reset_bootloader:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "mww 0x4000051C 0x40" -c reset -c shutdown

flash_to_bootloader:
	make flash
	make reset_bootloader

flash_sd110: s110/s110_nrf51822_7.3.0_softdevice.hex
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "nrf51 mass_erase" \
                 -c "flash write_image erase s110/s110_nrf51822_7.3.0_softdevice.hex" \
                 -c "reset run" -c shutdown

reset_openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c reset -c shutdown

gdb_openocd: $(PROGRAM).elf
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets

reset:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "mww 0x40000544 0x01" -c reset -c shutdown

openocd: $(PROGRAM).elf
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets

gdb: $(PROGRAM).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" -ex "set ((NRF_POWER_Type*)0x40000000)->GPREGRET = 0x40" $^

semihosting: $(PROGRAM).elf
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c reset -c "arm semihosting enable" -c reset
