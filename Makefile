# Project Name
TARGET = WaveContV3

USE_DAISYSP_LGPL = 1

# Sources
CPP_SOURCES = WaveContV3.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy/
DAISYSP_DIR = ../../DaisySP/

# Build for Daisy bootloader in external QSPI flash
APP_TYPE = BOOT_QSPI

# Includes FatFS source files within project.
USE_FATFS = 1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
