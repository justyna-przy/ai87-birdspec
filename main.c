/*
 * BirdSpec MAX78002 — On-Device Inference Application
 *
 * Default mode: continuous sliding-window inference.
 *   - Mic records continuously into a 3-second ring buffer.
 *   - Every ~1 second a new spectrogram is computed on the latest 3 s.
 *   - RMS gate: if the window is too quiet, skip inference and show
 *     "Listening..." instead of running the CNN.
 *   - Spectrogram and top-3 results update live on the TFT.
 *
 * Optional UART mode (press SW4 to toggle):
 *   - Stops continuous recording.
 *   - Accepts LOAD_PCM / REC / INFER / STATUS / BATCH commands.
 *   - Press SW4 again to return to continuous mode.
 *
 * Button mapping (MAX78002 EV kit):
 *   SW4  (PB_Get(0))  — toggle UART mode on/off
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mxc.h"
#include "pb.h"
#include "led.h"
#include "cnn.h"
#include "inference.h"
#include "audio_capture.h"
#include "spectrogram.h"
#include "display.h"
#include "uart_cmd.h"
#include "sd_batch.h"

/* ------------------------------------------------------------------ */
/* Shared globals (also extern'd by uart_cmd.c)                        */
/* ------------------------------------------------------------------ */

int8_t   g_cnn_input[64 * 128];
result_t g_results[INFERENCE_TOP_K_MAX];
uint32_t g_last_latency_us = 0;
uint32_t g_last_spec_us    = 0;

/* ------------------------------------------------------------------ */
/* State machine                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    APP_LISTENING,   /* default: continuous sliding-window inference    */
    APP_UART_MODE,   /* UART command mode (SW4 to exit)                 */
} app_state_t;

static app_state_t state = APP_LISTENING;

/* ------------------------------------------------------------------ */
/* Button edge detection (polled in main loop)                         */
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
    /* --- System init ------------------------------------------------ */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);    /* 120 MHz             */
    SystemCoreClockUpdate();
    Console_Init();                               /* re-sync UART baud  */

    MXC_ICC_Enable(MXC_ICC0);
    MXC_GCR->ipll_ctrl |= MXC_F_GCR_IPLL_CTRL_EN; /* IPLL for CNN     */

    /* Enable DWT cycle counter for spectrogram timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    MXC_Delay(MXC_DELAY_MSEC(200));

    printf("\n\n=== BirdSpec MAX78002 ===\n");
    printf("Clock: %lu Hz\n", (unsigned long)SystemCoreClock);

    /* --- Peripheral init ------------------------------------------- */
    display_init();
    display_status("Loading CNN...");

    inference_init();

    display_status("Init spectrogram...");
    spectrogram_init();

    display_status("Init microphone...");
    audio_capture_init();

    uart_cmd_init();
    sd_batch_init();

    /* Start continuous capture immediately */
    audio_capture_start();

    display_status("Listening... (filling 3s buffer)");
    printf("[main] Boot complete. Continuous mode. SW4 = UART mode.\n");

    /* --- Main loop -------------------------------------------------- */
    while (1) {

        switch (state) {

        /* ---- LISTENING (default) ----------------------------------- */
        case APP_LISTENING:

            if (audio_capture_snapshot_ready()) {

                float rms = audio_capture_rms();

                if (rms < AUDIO_RMS_THRESHOLD) {
                    /* Quiet — skip inference, keep listening */
                    display_status("Listening... (quiet)");
                } else {
                    /* Compute spectrogram */
                    uint32_t t0 = DWT->CYCCNT;
                    spectrogram_compute(audio_capture_get_buffer(),
                                        AUDIO_CLIP_SAMPLES, g_cnn_input);
                    g_last_spec_us = (DWT->CYCCNT - t0)
                                     / (SystemCoreClock / 1000000U);

                    /* Run CNN inference */
                    g_last_latency_us = inference_run(g_cnn_input,
                                                      g_results, 3);

                    /* Update display */
                    display_spectrogram(g_cnn_input);
                    display_results(g_results, 3, g_last_latency_us);

                    printf("[main] RMS=%.0f  lat=%lu us  spec=%lu us\n",
                           (double)rms,
                           (unsigned long)g_last_latency_us,
                           (unsigned long)g_last_spec_us);
                }

                /* Slide window forward 1 s and re-arm DMA */
                audio_capture_slide();
            }

            /* SW4 → enter UART mode */
            if (btn0_edge()) {
                audio_capture_stop();
                display_status("UART mode  |  SW4: return to listening");
                printf("[main] Entering UART mode.\n");
                state = APP_UART_MODE;
            }
            break;

        /* ---- UART MODE --------------------------------------------- */
        case APP_UART_MODE:

            uart_cmd_poll();

            /* SW4 → return to continuous listening */
            if (btn0_edge()) {
                printf("[main] Returning to listening mode.\n");
                audio_capture_start();
                display_status("Listening... (filling 3s buffer)");
                state = APP_LISTENING;
            }
            break;
        }

        MXC_Delay(MXC_DELAY_MSEC(5)); /* 200 Hz poll, low CPU overhead */
    }
}
