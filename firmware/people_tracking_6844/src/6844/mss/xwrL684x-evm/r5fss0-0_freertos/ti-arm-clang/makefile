#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy
HEX=$(CG_TOOL_ROOT)/bin/tiarmhex

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=mmwave_demo_xwrL684x.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
CONFIG_PATH:=$(BOOTIMAGE_PATH)/config
TEMP_PATH:=$(BOOTIMAGE_PATH)/temp

BOOTIMAGE_BIN_NAME:=mmwave_demo_xwrL684x.$(PROFILE).appimage
HEX_FILE1:=$(TEMP_PATH)/mmwave_demo_xwrL684x_reset_vectors.hex
HEX_FILE2:=$(TEMP_PATH)/mmwave_demo_xwrL684x_tcma_ram.hex
HEX_FILE3:=$(TEMP_PATH)/mmwave_demo_xwrL684x_tcmb_rbl_reserv.hex
HEX_FILE4:=$(TEMP_PATH)/mmwave_demo_xwrL684x_tcmb_ram.hex
HEX_FILE5:=$(TEMP_PATH)/mmwave_demo_xwrL684x_dss_l3.hex
CORE_IMAGE:= mmwave_demo_xwrL684x_r5_img.$(PROFILE).rig
CORE_IMAGE_TEMP:= $(TEMP_PATH)/mmwave_demo_xwrL684x_r5_img.$(PROFILE).rig

FILES_common := \
	mmw_cli.c \
	mmwave_demo.c \
	mmwave_control_config.c \
	mmw_flash_cal.c \
	range_phase_bias_measurement.c \
	main.c \
	mmw_demo_utils.c \
	ADC_testbuf.c \
	interrupts.c \
	dpc.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../.. \
	../../.. \
	../../../source \
	../../../source/calibrations \
	../../../source/utils \
	../../../source/test \
	../../../source/mmwave_control \
	../../../source/dpc \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/examples/mmw_demo/mmwave_demo_xwrL684x \
	-I${MCU_PLUS_SDK_PATH}/examples/mmw_demo/mmwave_demo_xwrL684x/source \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/xwrL684x/r5f \
	-I${MMW_DFP_PATH} \
	-Igenerated \

DEFINES_common := \
	-DSOC_XWRL684X \

CFLAGS_common := \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-mno-unaligned-access \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-deprecated-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-x c++ \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-Os \

LNK_FILES_common = \
	linker.cmd \

LNK_FILES_hex = \
	memory_hex.cmd \

LIBS_PATH_common = \
	-Wl,-i${MCU_PLUS_SDK_PATH} \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/datapath/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/control/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/utils/lib \
	-Wl,-i${MMW_DFP_PATH}/mmwavelink/lib/xWRL68XX \
	-Wl,-i${MMW_DFP_PATH}/fecsslib/lib/xWRL68XX \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lfreertos.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldatapath.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lboard.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lcontrol.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lutils.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lmmwavelink_r5.lib \
	-lfecss_ram_r5.lib \
	-llibc.a \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--ram_model \
	-Wl,--reread_libs \

LFLAGS_hex = \
	-romwidth 32 \
	-memwidth 8 \
	-i \

LIBS_NAME = \
	freertos.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	datapath.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	board.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	control.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	utils.xwrL684x.r5f.ti-arm-clang.${ConfigName}.lib \
	mmwavelink_r5.lib \
	fecss_ram_r5.lib \
	libc.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH} \
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/datapath/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/control/lib \
	${MCU_PLUS_SDK_PATH}/source/utils/lib \
	${MMW_DFP_PATH}/mmwavelink/lib/xWRL68XX \
	${MMW_DFP_PATH}/fecsslib/lib/xWRL68XX \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: xwrL684x:r5fss0-0:freertos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: xwrL684x:r5fss0-0:freertos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

all: $(BOOTIMAGE_BIN_NAME)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: xwrL684x:r5fss0-0:freertos:ti-arm-clang $@ ...
	$(LNK) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: xwrL684x:r5fss0-0:freertos:ti-arm-clang $@ Done !!!
	@echo  .
	@echo  Generating Hex Files ...
	$(MKDIR) $(TEMP_PATH)
	$(HEX) $(LFLAGS_hex) $(OUTNAME) $(LNK_FILES_hex)

clean:
	@echo  Cleaning: xwrL684x:r5fss0-0:freertos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_BIN_NAME)
	$(RM) $(AUTH_BOOTIMAGE_BIN_NAME)
	$(RM) $(INTEGRITY_BIN)
	$(RMDIR) generated/
	$(RM) $(CORE_IMAGE)
	$(RMDIR) $(TEMP_PATH)

scrub:
	@echo  Scrubing: xwrL684x:r5fss0-0:freertos:ti-arm-clang mmwave_demo_xwrL684x ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@


.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: ../example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part Default --package BGA --output generated/ ../example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device XWRL684X --context r5fss0-0 --part Default --package BGA --output generated/  ../example.syscfg

#
# Generation of boot image which can be loaded by ROM Boot Loader (RBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif

CORE_IMG_GEN=../../../../../../tools/MetaImageGen/build_image$(EXE_EXT)
CORE=APPSS
GENERATED_HEX_FILES= $(wildcard $(HEX_FILE1)) $(wildcard $(HEX_FILE2)) $(wildcard $(HEX_FILE3)) $(wildcard $(HEX_FILE4)) $(wildcard $(HEX_FILE5))
CORE_IMG_INPUT_FILES= $(subst .hex ,.hex --input_file ,$(strip $(GENERATED_HEX_FILES)))

META_IMG_GEN_PATH=../../../../../../tools/MetaImageGen
META_IMG_GEN=metaImage_tool_wrapper$(EXE_EXT)
META_IMG_CONFIG=$(CONFIG_PATH)/metaimage_cfg.$(PROFILE).json


$(BOOTIMAGE_BIN_NAME): $(OUTNAME)
	$(foreach file,$(GENERATED_HEX_FILES),@echo  Writing $(file) && ) @echo  .
	@echo  Boot image: xwrL684x:r5fss0-0:freertos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
	$(CORE_IMG_GEN) -c $(CORE) --input_file $(CORE_IMG_INPUT_FILES) --output_file $(CORE_IMAGE_TEMP)
	cd $(META_IMG_GEN_PATH) && $(META_IMG_GEN) --complete_metaimage $(META_IMG_CONFIG)
	$(COPY) $(CORE_IMAGE_TEMP) $(CORE_IMAGE)
	$(RMDIR) $(TEMP_PATH)
	@echo  Boot image: xwrL684x:r5fss0-0:freertos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .


-include $(addprefix $(OBJDIR)/, $(DEPS))
