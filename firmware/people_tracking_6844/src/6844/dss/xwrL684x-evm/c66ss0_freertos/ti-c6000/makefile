#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak

CG_TOOL_ROOT=$(CGT_TI_C6000_PATH)

CC=$(CG_TOOL_ROOT)/bin/cl6x
LNK=$(CG_TOOL_ROOT)/bin/lnk6x
STRIP=$(CG_TOOL_ROOT)/bin/strip6x
OBJCOPY=$(CG_TOOL_ROOT)/bin/
HEX=$(CG_TOOL_ROOT)/bin/hex6x

PROFILE?=release
ConfigName:=$(PROFILE)

SYSTEM_FLAG ?= false

ifeq ($(SYSTEM_FLAG), false)
OUTNAME:=ipc_notify_echo.$(PROFILE).out
else
OUTNAME:=ipc_notify_echo_system.$(PROFILE).out
endif

BOOTIMAGE_PATH=$(abspath .)
TEMP_PATH:=$(BOOTIMAGE_PATH)/temp

HEX_FILE1:=$(TEMP_PATH)/ipc_notify_echo_dss_l2.hex
HEX_FILE2:=$(TEMP_PATH)/ipc_notify_echo_mailbox_hsm.hex
HEX_FILE3:=$(TEMP_PATH)/ipc_notify_echo_mailbox_r5f.hex
HEX_FILE4:=$(TEMP_PATH)/ipc_notify_echo_dss_l3.hex
HEX_FILE5:=$(TEMP_PATH)/ipc_notify_echo_user_shm_mem.hex
HEX_FILE6:=$(TEMP_PATH)/ipc_notify_echo_log_shm_mem.hex
HEX_FILE7:=$(TEMP_PATH)/ipc_notify_echo_rtos_nortos_ipc_shm_mem.hex
ifeq ($(SYSTEM_FLAG), false)
CORE_IMAGE:= ipc_notify_echo_c66_img.$(PROFILE).rig
else
CORE_IMAGE:= ipc_notify_echo_c66_img_system.$(PROFILE).rig
endif

TARGETS := $(CORE_IMAGE)

FILES_common := \
	ipc_notify_echo.c \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_CGT/DSP_C66 \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/xwrL684x/c66 \
	-Igenerated \

DEFINES_common := \
	-DSOC_XWRL684X \

CFLAGS_common := \
	-mv6600 \
	--c99 \
	-q \
	-mo \
	-pden \
	--emit_warnings_as_errors \
	--diag_suppress=1,179,188,303 \
	--mem_model:const=data \
	--mem_model:data=far_aggregates \
	--remove_hooks_when_inlining \
	-on2 \
	--disable_push_pop \
	--fp_mode=relaxed \
	--assume_control_regs_read \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	--program_level_compile \
	-o3 \
	-mf2 \

LNK_FILES_common = \
	linker.cmd \

LNK_FILES_hex = \
	memory_hex.cmd \

LIBS_PATH_common = \
	-i${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lfreertos.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	-ldrivers.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	-lboard.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	-llibc.a \

LFLAGS_common = \
	--warn_sections \
	--diag_suppress=1,179,188,303 \
	--emit_warnings_as_errors \
	--ram_model \
	-x \

LFLAGS_hex = \
	-romwidth 32 \
	-memwidth 8 \
	-i \

LIBS_NAME = \
	freertos.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	drivers.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	board.xwrL684x.c66.ti-c6000.${ConfigName}.lib \
	libc.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
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
	@echo  Compiling: xwrL684x:c66ss0:freertos:ti-c6000 $(OUTNAME): $<
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) -ppd=$(OBJDIR)/$(basename $@).d -ppa -fr=$(OBJDIR)/ -fc=$<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: xwrL684x:c66ss0:freertos:ti-c6000 $(LIBNAME): $<
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) -ppd=$(OBJDIR)/$(basename $@).d -ppa -fr=$(OBJDIR)/ -fa=$<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

ifeq ($(SYSTEM_FLAG), false)
SYSTEM_COMMAND = syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
else
SYSTEM_COMMAND = $(OBJS) $(LNK_FILES) $(LIBS_NAME)
endif

$(OUTNAME): $(SYSTEM_COMMAND)
	@echo  .
	@echo  Linking: xwrL684x:c66ss0:freertos:ti-c6000 $@ ...
	$(LNK) $(LFLAGS) $(LIBS_PATH) -m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: xwrL684x:c66ss0:freertos:ti-c6000 $@ Done !!!
	@echo  .
	@echo  Generating Hex Files ...
	$(MKDIR) $(TEMP_PATH)
	$(HEX) $(LFLAGS_hex) $(OUTNAME) $(LNK_FILES_hex)

clean:
	@echo  Cleaning: xwrL684x:c66ss0:freertos:ti-c6000 $(OUTNAME) ...
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
	@echo  Scrubing: xwrL684x:c66ss0:freertos:ti-c6000 ipc_notify_echo ...
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
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context c66ss0 --part Default --package BGA --output generated/ ../example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device XWRL684X --context c66ss0 --part Default --package BGA --output generated/  ../example.syscfg

#
# Generation of boot image which can be loaded by ROM Boot Loader (RBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif

CORE_IMG_GEN=../../../../../../../tools/MetaImageGen/build_image$(EXE_EXT)
CORE=DSP
GENERATED_HEX_FILES= $(wildcard $(HEX_FILE1)) $(wildcard $(HEX_FILE2)) $(wildcard $(HEX_FILE3)) $(wildcard $(HEX_FILE4)) $(wildcard $(HEX_FILE5)) $(wildcard $(HEX_FILE6)) $(wildcard $(HEX_FILE7))
CORE_IMG_INPUT_FILES= $(subst .hex ,.hex --input_file ,$(strip $(GENERATED_HEX_FILES)))

$(CORE_IMAGE): $(OUTNAME)
	$(foreach file,$(GENERATED_HEX_FILES),echo  Writing $(file) && ) echo  .
	@echo  Core image: xwrL684x:c66ss0:freertos:ti-c6000 $(BOOTIMAGE_PATH)/$@ ...
	$(CORE_IMG_GEN) -c $(CORE) --input_file $(CORE_IMG_INPUT_FILES) --output_file $(CORE_IMAGE)
	$(RMDIR) $(TEMP_PATH)
	@echo  Core image: xwrL684x:c66ss0:freertos:ti-c6000 $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .


-include $(addprefix $(OBJDIR)/, $(DEPS))
