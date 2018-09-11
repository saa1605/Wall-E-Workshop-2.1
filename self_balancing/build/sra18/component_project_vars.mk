# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)/components/sra18/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/sra18 -lsra18 -Wl,--undefined=uxTopUsedPriority
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += sra18
component-sra18-build: 
