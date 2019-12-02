DEBUG ?= 0

PROJECTFOLDER = $(shell pwd)
TOPFOLDER = $(PROJECTFOLDER)/..
TRANSPORTS_DIR = $(PROJECTFOLDER)/transports
CRAZYFLIE_BASE = $(PROJECTFOLDER)/crazyflie-firmware

CROSSDEV = arm-none-eabi-
ARCHCPUFLAGS =  -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP  -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections

ifeq ($(DEBUG), 1)
	ARCHCPUFLAGS += -O0 -g3
  	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

# micro-ROS variables

EXTERNAL_TRANSPORT_HEADER = $(PROJECTFOLDER)/transports/crazyflie_transport.h
EXTERNAL_TRANSPORT =$(PROJECTFOLDER)/transports/crazyflie_transport.c

MICROXRCE_INCLUDES += -I$(PROJECTFOLDER)/Micro-XRCE-DDS-Client/build/install/include

MICROXRCE_LIBRARIES += time_external.o
MICROXRCE_LIBRARIES += libmicroxrcedds_client.a
MICROXRCE_LIBRARIES += libmicrocdr.a

BUILD_INCLUDES += $(CRAZYFLIE_BASE)/src/hal/interface 
BUILD_INCLUDES += $(CRAZYFLIE_BASE)/src/modules/interface 
BUILD_INCLUDES += $(CRAZYFLIE_BASE)/src/utils/interface 
BUILD_INCLUDES += $(CRAZYFLIE_BASE)/src/config 
BUILD_INCLUDES += $(CRAZYFLIE_BASE)/src/drivers/interface
BUILD_INCLUDES_STR := $(foreach x,$(BUILD_INCLUDES),$(x)\n)


# Crazyflie 2.1 app configuration

APP = 1
APP_STACKSIZE = 2100
APP_PRIORITY = 3

PROJ_OBJ += microxrceddsapp.o
PROJ_OBJ += $(MICROXRCE_LIBRARIES) 
INCLUDES += $(MICROXRCE_INCLUDES)
VPATH += $(PROJECTFOLDER)
CFLAGS += -DFREERTOS_HEAP_SIZE=52500

include $(CRAZYFLIE_BASE)/Makefile

# Micro-XRCE targets

arm_toolchain.cmake: arm_toolchain.cmake.in
	rm -f $(PROJECTFOLDER)/arm_toolchain.cmake; \
	cat $(PROJECTFOLDER)/arm_toolchain.cmake.in | \
		sed "s/@CROSSDEV@/$(CROSSDEV)/g" | \
		sed "s/@FREERTOS_TOPDIR@/$(subst /,\/,$(TOPFOLDER))/g" | \
		sed "s/@EXTERNAL_TRANSPORT_HEADER@/$(subst /,\/,$(EXTERNAL_TRANSPORT_HEADER))/g" | \
		sed "s/@EXTERNAL_TRANSPORT@/$(subst /,\/,$(EXTERNAL_TRANSPORT))/g" | \
		sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)\"/g" | \
		sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" | \
		sed "s/@INCLUDES@/$(subst /,\/,$(BUILD_INCLUDES_STR))/g" \
		> $(PROJECTFOLDER)/arm_toolchain.cmake

libmicroxrcedds: arm_toolchain.cmake
	cd $(PROJECTFOLDER)/Micro-XRCE-DDS-Client; rm -rf build; mkdir build; cd build; \
	cmake .. -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DUCLIENT_PIC=OFF -DCMAKE_INSTALL_PREFIX=./install -DCMAKE_TOOLCHAIN_FILE=$(PROJECTFOLDER)/arm_toolchain.cmake && make && make install; \
	cd $(PROJECTFOLDER); mkdir -p $(PROJECTFOLDER)/bin/; \
	cp $(PROJECTFOLDER)/Micro-XRCE-DDS-Client/build/install/lib/libmicrocdr.a $(PROJECTFOLDER)/bin/; \
	cp $(PROJECTFOLDER)/Micro-XRCE-DDS-Client/build/install/lib/libmicroxrcedds_client.a $(PROJECTFOLDER)/bin/	