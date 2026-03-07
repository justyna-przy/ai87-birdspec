/*
 * BirdSpec MAX78002 — On-Device Inference Application
 *
 * SW4 (PB_Get(0)): Start / dismiss recording
 *
 * State machine:
 *   IDLE  →(SW4)→  RECORDING  →(done)→  PROCESSING  →  SHOWING
 *                 →(timeout)→  SHOWING  [shows error]
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc.h"
#include "board.h"
#include "pb.h"
#include "led.h"
#include "cnn.h"
#include "inference.h"
#include "audio_capture.h"
#include "spectrogram.h"
#include "display.h"

/* ------------------------------------------------------------------ */
/* Shared buffers                                                        */
/* ------------------------------------------------------------------ */

static int8_t   g_cnn_input[64 * 128];
static result_t g_results[3];
static uint32_t g_latency_us = 0;

/* ------------------------------------------------------------------ */
/* State machine                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    APP_IDLE,
    APP_RECORDING,
    APP_PROCESSING,
    APP_SHOWING,
} app_state_t;

static app_state_t state = APP_IDLE;

/* Recording timeout: 4 s = 200 × 20 ms ticks */
#define RECORD_TIMEOUT_TICKS  200
static int record_ticks = 0;

/* ------------------------------------------------------------------ */
/* Button edge detection (polled at 50 Hz)                              */
/* ------------------------------------------------------------------ */

static int btn0_prev = 0;

static int btn0_edge(void)
{
    int cur  = PB_Get(0);
    int edge = cur && !btn0_prev;
    btn0_prev = cur;
    return edge;
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    /* Re-sync console UART baud rate after clock switch */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();
    Console_Init();

    MXC_ICC_Enable(MXC_ICC0);
    MXC_Delay(MXC_DELAY_MSEC(200));

    printf("\n\n=== BirdSpec MAX78002 ===\n");
    printf("Clock: %lu Hz\n", (unsigned long)SystemCoreClock);

    /* ---- Peripheral init ------------------------------------------ */
    display_init();
    display_status("Loading CNN weights...");

    inference_init();
    display_status("Init spectrogram...");

    spectrogram_init();
    display_status("Init microphone...");

    audio_capture_init();

    display_status("Ready  |  SW4: record 3s");
    printf("[main] Boot complete. Press SW4 to record.\n");

    /* ---- Main loop ------------------------------------------------- */
    while (1) {
        switch (state) {

        /* ---- IDLE -------------------------------------------------- */
        case APP_IDLE:
            if (btn0_edge()) {
                record_ticks = 0;
                state = APP_RECORDING;
                display_status("Recording... (3 s)");
                LED_On(0);
                printf("[main] Recording started.\n");
                audio_capture_start();
            }
            break;

        /* ---- RECORDING --------------------------------------------- */
        case APP_RECORDING: {
            record_ticks++;

            /* Show countdown on TFT so user knows we're alive */
            if (record_ticks % 25 == 0) {   /* every 500 ms */
                int secs_left = 3 - (record_ticks / 50);
                char buf[48];
                snprintf(buf, sizeof(buf), "Recording... %d s left", secs_left > 0 ? secs_left : 0);
                display_status(buf);
                LED_Toggle(0);
            }

            if (audio_capture_is_done()) {
                LED_Off(0);
                printf("[main] Capture done (%d samples).\n", AUDIO_CLIP_SAMPLES);
                state = APP_PROCESSING;
                display_status("Computing spectrogram...");
            } else if (record_ticks >= RECORD_TIMEOUT_TICKS) {
                /* DMA never completed — microphone likely not responding */
                LED_Off(0);
                printf("[main] ERROR: recording timed out — mic not detected.\n");
                display_status("MIC ERROR: no audio (check JH4)");
                state = APP_SHOWING;   /* let SW4 dismiss */
            }
            break;
        }

        /* ---- PROCESSING -------------------------------------------- */
        case APP_PROCESSING: {
            printf("[main] Computing spectrogram...\n");
            spectrogram_compute(audio_capture_get_buffer(),
                                AUDIO_CLIP_SAMPLES, g_cnn_input);

            display_status("Running inference...");
            printf("[main] Running inference...\n");

            g_latency_us = inference_run(g_cnn_input, g_results, 3);

            printf("[main] Done. Latency: %lu us\n", (unsigned long)g_latency_us);
            for (int i = 0; i < 3; i++) {
                printf("  #%d idx=%-3d conf=%.1f%%\n",
                       i + 1, g_results[i].class_idx,
                       (double)g_results[i].confidence);
            }

            display_spectrogram(g_cnn_input);
            display_results(g_results, 3, g_latency_us);
            state = APP_SHOWING;
            break;
        }

        /* ---- SHOWING ----------------------------------------------- */
        case APP_SHOWING:
            if (btn0_edge()) {
                state = APP_IDLE;
                display_status("Ready  |  SW4: record 3s");
                printf("[main] Back to IDLE.\n");
            }
            break;
        }

        MXC_Delay(MXC_DELAY_MSEC(20)); /* 50 Hz poll */
    }
}
