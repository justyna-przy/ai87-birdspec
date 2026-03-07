#include <stdio.h>
#include <string.h>
#include "mxc.h"
#include "cnn.h"
/* Note: softmax_q17p14_q15 is declared in cnn.h — no separate softmax.h needed */
#include "sampledata.h"
#include "sampleoutput.h"
#include "inference.h"

/* Defined in cnn.c — set by CNN_ISR when inference completes */
volatile uint32_t cnn_time;

/* Output buffers — reused across calls */
static int32_t  ml_data[CNN_NUM_OUTPUTS];
static q15_t    ml_softmax[CNN_NUM_OUTPUTS];

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

static void clock_boost(void)
{
    /* CNN clock: PLL (200 MHz) / 1 */
    MXC_GCR->pclkdiv = (MXC_GCR->pclkdiv
                        & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL))
                       | MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1
                       | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
}

static void clock_throttle(void)
{
    /* CNN clock: PLL (200 MHz) / 4 = 50 MHz (idle / safe) */
    MXC_GCR->pclkdiv = (MXC_GCR->pclkdiv
                        & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL))
                       | MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4
                       | MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL;
}

/* Simple insertion sort (50 elements, top_k <= 5 — fast enough) */
static void top_k_sort(result_t *results, int top_k)
{
    for (int i = 1; i < top_k; i++) {
        result_t key = results[i];
        int j = i - 1;
        while (j >= 0 && results[j].confidence < key.confidence) {
            results[j + 1] = results[j];
            j--;
        }
        results[j + 1] = key;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void inference_init(void)
{
    /* Enable IPLL (needed for CNN clock source) */
    MXC_GCR->ipll_ctrl |= MXC_F_GCR_IPLL_CTRL_EN;

    /* Enable CNN peripheral; start at PLL/4 = 50 MHz */
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_IPLL, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV4);

    cnn_init();         /* Bring state machine to consistent state */
    cnn_load_weights(); /* Load quantised kernels into CNN SRAM    */
    cnn_load_bias();    /* No-op for BirdSpec (no bias)            */
    cnn_configure();    /* Configure all layer registers           */

    printf("[inference] Weights loaded, CNN ready.\n");
}

uint32_t inference_run(const int8_t *tensor, result_t *results, int top_k)
{
    if (top_k > INFERENCE_TOP_K_MAX) top_k = INFERENCE_TOP_K_MAX;
    if (top_k < 1)                    top_k = 1;

    /* --- Load input into CNN input SRAM (quadrant 0, 0x51800000) --- */
    /* Input is 64×128 int8 = 8192 bytes = 2048 uint32_t words.
     * The CNN accelerator reads data in 32-bit words; casting int8* to
     * uint32_t* is safe here because the buffer is word-aligned and the
     * accelerator treats it as raw bytes regardless of sign. */
    memcpy32((uint32_t *)0x51800000, (const uint32_t *)tensor, 2048);

    /* --- Boost CNN clock and start inference --- */
    cnn_time = 0;
    clock_boost();
    cnn_start();

    /* Sleep until CNN_ISR fires (sets cnn_time) */
    while (cnn_time == 0)
        MXC_LP_EnterSleepMode();

    clock_throttle();

    /* --- Unload outputs and apply softmax --- */
    cnn_unload((uint32_t *)ml_data);
    softmax_q17p14_q15((const q31_t *)ml_data, CNN_NUM_OUTPUTS, ml_softmax);

    /* --- Build top-k result list (full scan of 50 classes) --- */
    /* Initialise results with the worst possible values */
    for (int i = 0; i < top_k; i++) {
        results[i].class_idx  = 0;
        results[i].confidence = -1.0f;
    }

    for (int i = 0; i < CNN_NUM_OUTPUTS; i++) {
        /* ml_softmax is Q15: value/32768 * 100 = percentage */
        float conf = (float)ml_softmax[i] * (100.0f / 32768.0f);
        if (conf > results[top_k - 1].confidence) {
            results[top_k - 1].class_idx  = i;
            results[top_k - 1].confidence = conf;
            top_k_sort(results, top_k);
        }
    }

    return cnn_time; /* microseconds */
}

int inference_kat(void)
{
    /* Load the built-in sample input (same pattern as generated main.c) */
    static const uint32_t input_0[]     = SAMPLE_INPUT_0;
    static const uint32_t sample_out[]  = SAMPLE_OUTPUT;

    memcpy32((uint32_t *)0x51800000, input_0, 2048);

    cnn_time = 0;
    clock_boost();
    cnn_start();
    while (cnn_time == 0)
        MXC_LP_EnterSleepMode();
    clock_throttle();

    /* Verify output against known-answer */
    const uint32_t    *ptr = sample_out;
    volatile uint32_t *addr;
    uint32_t mask, len;
    int i;

    while ((addr = (volatile uint32_t *)(uintptr_t)*ptr++) != 0) {
        mask = *ptr++;
        len  = *ptr++;
        for (i = 0; i < (int)len; i++) {
            if ((*addr++ & mask) != *ptr++) {
                printf("[KAT] FAIL at address 0x%08x\n", (unsigned)(uintptr_t)(addr - 1));
                return CNN_FAIL;
            }
        }
    }

    printf("[KAT] PASS  (inference time: %u us)\n", (unsigned)cnn_time);
    return CNN_OK;
}
