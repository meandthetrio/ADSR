# Project Name
TARGET = WaveContV3

USE_DAISYSP_LGPL = 1

# Sources
CPP_SOURCES = WaveContV3.cpp StorageService.cpp SampleMemoryManager.cpp VoiceManager.cpp WaveformCache.cpp audio_engine.cpp ui.cpp app_controller.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Build for Daisy bootloader in external QSPI flash
APP_TYPE = BOOT_QSPI

# Includes FatFS source files within project.
USE_FATFS = 1

# Configurable audio block size (override via `make AUDIO_BLOCK_SIZE=96`)
AUDIO_BLOCK_SIZE ?= 48
CXXFLAGS += -DAUDIO_BLOCK_SIZE=$(AUDIO_BLOCK_SIZE)

# Guardrails: module include boundaries.
.PHONY: check_modules
check_modules:
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"](StorageService.h|ff\\.h|ui.h)[>\"]' audio_engine.cpp audio_engine.h) { Write-Host 'ERROR: audio_engine includes forbidden headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"](PerformVoice.h|SampleMemoryManager.h|ff\\.h)[>\"]' ui.cpp ui.h) { Write-Host 'ERROR: ui includes forbidden headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"]ff\\.h[>\"]' StorageService.h ui.cpp ui.h) { Write-Host 'ERROR: ff.h leaked into headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+\".*daisy.*\"' shared_messages.h) { Write-Host 'ERROR: shared_messages.h should not include Daisy headers'; exit 1 } }"

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
