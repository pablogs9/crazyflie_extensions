PROJECTFOLDER = $(shell pwd)

ifeq ($(CONFIG_UROS_DIR),)
	TOPFOLDER = $(PROJECTFOLDER)/..
	UROS_DIR = $(TOPFOLDER)/mcu_ws
else
	UROS_DIR = $(CONFIG_UROS_DIR)
	TOPFOLDER = $(UROS_DIR)/..
endif

EXTENSIONS_DIR = $(TOPFOLDER)/crazyflie_microros_extensions
TRANSPORTS_DIR = $(EXTENSIONS_DIR)/transports

CROSSDEV = arm-none-eabi-
ARCHCPUFLAGS =  -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__TARGET_FPU_VFP  -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections
# ARCHOPTIMIZATION = -Os -g3

ifeq ($(DEBUG), 1)
	ARCHCPUFLAGS += -O0 -g3
  	BUILD_TYPE = Debug
else
	BUILD_TYPE = Release
endif

MICROROS_INCLUDES += $(shell find $(UROS_DIR)/install -name 'include' | sed -E "s/(.*)/-I\1/")

MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/include
MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/include/sys
MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/include/private
MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/FreeRTOS-Plus-POSIX/include
MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/FreeRTOS-Plus-POSIX/include/portable
MICROROS_INCLUDES += -I$(EXTENSIONS_DIR)/FreeRTOS-Plus-POSIX/include/portable/crazyflie

MICROROS_LIBRARIES = libmicroros.a

MICROROS_POSIX_FREERTOS_OBJECTS_VPATH =  $(EXTENSIONS_DIR)/FreeRTOS-Plus-POSIX/source
MICROROS_POSIX_FREERTOS_OBJECTS = FreeRTOS_POSIX_clock.o FreeRTOS_POSIX_pthread_mutex.o FreeRTOS_POSIX_semaphore.o FreeRTOS_POSIX_mqueue.o FreeRTOS_POSIX_sched.o FreeRTOS_POSIX_pthread.o FreeRTOS_POSIX_timer.o FreeRTOS_POSIX_pthread_barrier.o FreeRTOS_POSIX_unistd.o FreeRTOS_POSIX_pthread_cond.o FreeRTOS_POSIX_utils.o libatomic.o

COLCON_INCLUDES = $(EXTENSIONS_DIR)/FreeRTOS-Plus-POSIX/include $(EXTENSIONS_DIR)/include $(EXTENSIONS_DIR)/include/private $(EXTENSIONS_DIR)/include/FreeRTOS_POSIX $(EXTENSIONS_DIR)/include/FreeRTOS_POSIX/sys
COLCON_INCLUDES += $(PROJECTFOLDER)/src/hal/interface $(PROJECTFOLDER)/src/modules/interface $(PROJECTFOLDER)/src/utils/interface $(PROJECTFOLDER)/src/config $(PROJECTFOLDER)/src/drivers/interface
COLCON_INCLUDES_STR := $(foreach x,$(COLCON_INCLUDES),$(x)\n)

all: libmicroros

arm_toolchain.cmake: $(EXTENSIONS_DIR)/arm_toolchain.cmake.in
	rm -f $(EXTENSIONS_DIR)/arm_toolchain.cmake; \
	cat $(EXTENSIONS_DIR)/arm_toolchain.cmake.in | \
		sed "s/@CROSSDEV@/$(CROSSDEV)/g" | \
		sed "s/@FREERTOS_TOPDIR@/$(subst /,\/,$(TOPFOLDER))/g" | \
		sed "s/@ARCH_CPU_FLAGS@/\"$(ARCHCPUFLAGS)\"/g" | \
		sed "s/@ARCH_OPT_FLAGS@/\"$(ARCHOPTIMIZATION)\"/g" | \
		sed "s/@INCLUDES@/$(subst /,\/,$(COLCON_INCLUDES_STR))/g" \
		> $(EXTENSIONS_DIR)/arm_toolchain.cmake

colcon_compile: arm_toolchain.cmake
	cd $(UROS_DIR); \
	colcon build \
		--packages-ignore-regex=.*_cpp \
		--cmake-args \
		-DUCLIENT_SUPERBUILD=OFF \
		-DUCLIENT_PIC=OFF \
		-DCMAKE_POSITION_INDEPENDENT_CODE=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/arm_toolchain.cmake \
		-DCMAKE_VERBOSE_MAKEFILE=ON; \

libmicroros: colcon_compile
	mkdir -p $(UROS_DIR)/libmicroros; cd $(UROS_DIR)/libmicroros; \
	for file in $$(find $(UROS_DIR)/install/ -name '*.a'); do \
		folder=$$(echo $$file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $$folder; cd $$folder; ar x $$file; \
		for f in *; do \
			mv $$f ../$$folder-$$f; \
		done; \
		cd ..; rm -rf $$folder; \
	done ; \
	ar rc libmicroros.a *.obj; cp libmicroros.a $(PROJECTFOLDER)/bin/; ranlib $(PROJECTFOLDER)/bin/libmicroros.a;\
	cd ..; rm -rf libmicroros;


# find "$(UROS_DIR)"/install -name '*.a' | xargs -I{} cp "{}" $(PROJECTFOLDER)/bin/ ;


# colcon_compile: arm_toolchain.cmake
# 	cd $(UROS_DIR); \
# 	rm -rf build install log; \
# 	colcon build \
# 		--packages-ignore-regex=.*_cpp \
# 		--cmake-args \
#  		-DBUILD_SHARED_LIBS=OFF \
# 		-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
# 		-DTHIRDPARTY=ON \
# 		-DBUILD_TESTING=OFF \
# 		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
# 		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/arm_toolchain.cmake \
# 		-DCMAKE_VERBOSE_MAKEFILE=ON; \
# 	find "$(UROS_DIR)"/install -name '*.a' | xargs -I{} cp "{}" $(PROJECTFOLDER)/bin/ ;

	
# colcon_compile: arm_toolchain.cmake
# 	cd $(UROS_DIR); \
# 	rm -rf build install log; \
# 	colcon build \
# 		--packages-up-to microxrcedds_client \
# 		--packages-ignore-regex=.*_cpp \
# 		--cmake-args \
# 		-DBUILD_SHARED_LIBS=OFF \
# 		-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
# 		-DBUILD_TESTING=OFF \
# 		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
# 		-DCMAKE_TOOLCHAIN_FILE=$(EXTENSIONS_DIR)/arm_toolchain.cmake \
# 		-DCMAKE_VERBOSE_MAKEFILE=ON \