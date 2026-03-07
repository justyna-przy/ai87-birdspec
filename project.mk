# BirdSpec MAX78002 project configuration
# MAXIM_PATH is read from your environment (already set by the MSDK installer).

# Enable CMSIS-DSP for arm_rfft_fast_f32, mel filterbank computation
LIB_CMSIS_DSP = 1

# Enable SDHC library (FatFS) for SD card batch evaluation
LIB_SDHC = 1

# Enable Board Support Package (includes TFT display driver for EvKit_V1)
LIB_BOARD = 1

# Keep ABI compatible with MSDK libraries.
# softfp is the MSDK default and avoids hard/softfp link ABI mismatches.
MFLOAT_ABI = softfp

# Define ARM_MATH_CM4 for CMSIS-DSP
PROJ_CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1

# SPI version: board.c TFT_SPI_Init uses the v1 MXC_SPI_Init() signature
MXC_SPI_VERSION = v1

# Font needed for MXC_TFT_Printf / MXC_TFT_ConfigPrintf to render text
FONTS = LiberationSans16x16

# Project output name
PROJECT = birdspec
