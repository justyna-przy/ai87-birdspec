# BirdSpec MAX78002 project configuration
# MAXIM_PATH is read from your environment (already set by the MSDK installer).

# Enable CMSIS-DSP for arm_rfft_fast_f32, mel filterbank computation
LIB_CMSIS_DSP = 1

# Enable SDHC library (FatFS) for SD card batch evaluation
LIB_SDHC = 1

# Enable Board Support Package (includes TFT display driver for EvKit_V1)
LIB_BOARD = 1

# Use hardware FPU — Cortex-M4 has an FPU, use it for float32 spectrogram
MFLOAT_ABI = hard

# Define ARM_MATH_CM4 for CMSIS-DSP
PROJ_CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1

# Use TMR0 as the CNN inference stopwatch.
# cnn.c calls MXC_TMR_SW_Start/Stop around the CNN execution;
# MXC_TMR_SW_Start self-initialises the timer so no separate MXC_TMR_Init is needed.
# This gives accurate latency_us values independent of CPU sleep mode.
PROJ_CFLAGS += -DCNN_INFERENCE_TIMER=MXC_TMR0

# SPI version required by the EvKit TFT (ST7789V uses v1 API)
MXC_SPI_VERSION = v1

# Font for TFT text rendering — compiles LiberationSans16x16.c and defines
# -DFONT_LiberationSans16x16 so that fonts.h exposes Liberation_Sans16x16[]
FONTS = LiberationSans16x16

# Project output name
PROJECT = birdspec
