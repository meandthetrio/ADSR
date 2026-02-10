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

# Build modes
BUILD ?= RELEASE
PERF ?= 0

ifeq ($(BUILD),DEBUG)
    CXXFLAGS += -Og -g3 -DDEBUG_BUILD=1
else
    CXXFLAGS += -O3 -DNDEBUG=1 -DRELEASE_BUILD=1
    LTO ?= 1
    ifeq ($(LTO),1)
        CXXFLAGS += -flto
    endif
endif

CXXFLAGS += -DPERF=$(PERF)

# Guardrails: module include boundaries.
.PHONY: check_modules
check_modules:
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"](StorageService.h|ff\\.h|ui.h)[>\"]' audio_engine.cpp audio_engine.h) { Write-Host 'ERROR: audio_engine includes forbidden headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"](PerformVoice.h|SampleMemoryManager.h|ff\\.h)[>\"]' ui.cpp ui.h) { Write-Host 'ERROR: ui includes forbidden headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+[<\"]ff\\.h[>\"]' StorageService.h ui.cpp ui.h) { Write-Host 'ERROR: ff.h leaked into headers'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '#include\\s+\".*daisy.*\"' shared_messages.h) { Write-Host 'ERROR: shared_messages.h should not include Daisy headers'; exit 1 } }"

.PHONY: check_e13
check_e13:
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n 'extern\\s+' -g '*.h' -g '*.cpp') { Write-Host 'ERROR: extern globals found'; exit 1 } }"
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n '^\\s*(static\\s+)?(volatile\\s+)?(bool|int|int32_t|uint32_t|size_t|float|double|UiMode|.*Queue|.*Index|.*Flag)\\s+g_' WaveContV3.cpp) { Write-Host 'ERROR: runtime g_* globals found in WaveContV3.cpp'; exit 1 } }"

.PHONY: release debug check_release
release:
	@$(MAKE) BUILD=RELEASE

debug:
	@$(MAKE) BUILD=DEBUG

check_release:
	@powershell -Command "if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n 'PERF_DIAGNOSTICS\\s+1' -g '*.h' -g '*.cpp') { Write-Host 'ERROR: PERF_DIAGNOSTICS left in code'; exit 1 } }"
	@powershell -Command \"if (Get-Command rg -ErrorAction SilentlyContinue) { if (rg -n 'printf|sprintf|snprintf|LogLine' -g '*.h' -g '*.cpp' --glob '!BuildConfig.h') { Write-Host 'ERROR: printf/sprintf/snprintf/LogLine found'; exit 1 } }\"
	@powershell -Command \"if (Get-Command rg -ErrorAction SilentlyContinue) { if (-not (rg -n '#define\\s+ENABLE_DEBUG_UI\\s+0' BuildConfig.h)) { Write-Host 'ERROR: ENABLE_DEBUG_UI not defaulting to 0 in RELEASE'; exit 1 } }\"

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
