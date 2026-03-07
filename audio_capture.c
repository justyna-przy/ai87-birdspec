/*
 * audio_capture.c — I2S DMA microphone capture for MAX78002 EV kit
 *
 * Pattern taken directly from MSDK Examples/MAX78002/I2S_DMA/main.c.
 *
 * Hardware: SPH0645 I2S MEMS mic on the EV kit (header JH4).
 *   - 18-bit audio, left-justified in a 32-bit I2S word.
 *   - We extract int16 by taking the top 16 bits (right-shift 16).
 *   - MONO_LEFT_CH — right channel is zero-filled by the mic.
 *
 * Clock: clkdiv=5 assumes a 12.288 MHz I2S source clock, giving:
 *   BCLK = 12.288 MHz / (2*(5+1)) = 1.024 MHz
 *   LRCK = 1.024 MHz / 64          = 16 kHz
 *
 * Capture: chunked DMA, CHUNK_SAMPLES int32 words at a time.
 *   DMA0_IRQHandler → MXC_DMA_Handler() → i2s_dma_callback().
 *   Callback accumulates chunks into pcm_buf[] until AUDIO_CLIP_SAMPLES.
 */

#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "i2s.h"
#include "dma.h"
#include "nvic_table.h"
#include "audio_capture.h"

/* ------------------------------------------------------------------ */
/* Buffers                                                              */
/* ------------------------------------------------------------------ */

/* Same size as the MSDK I2S_DMA example — proven to work */
#define CHUNK_SAMPLES  256

static int32_t  dma_chunk[CHUNK_SAMPLES];   /* one DMA transfer         */
static int16_t  pcm_buf[AUDIO_CLIP_SAMPLES];/* final 16-bit PCM output  */

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

static volatile int  samples_collected = 0;
static volatile bool capture_done      = false;

/* ------------------------------------------------------------------ */
/* DMA IRQ — overrides the weak default in the vector table            */
/* ------------------------------------------------------------------ */

void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

/* ------------------------------------------------------------------ */
/* DMA completion callback (registered with MXC_I2S_RegisterDMACallback) */
/* ------------------------------------------------------------------ */

static void i2s_dma_callback(int channel, int error)
{
    (void)channel;

    if (error != E_NO_ERROR) {
        capture_done = true;    /* unblock caller; error visible in serial */
        return;
    }

    /* Copy this chunk: top 16 bits of each 32-bit I2S word */
    for (int i = 0; i < CHUNK_SAMPLES && samples_collected < AUDIO_CLIP_SAMPLES; i++) {
        pcm_buf[samples_collected++] = (int16_t)(dma_chunk[i] >> 16);
    }

    if (samples_collected >= AUDIO_CLIP_SAMPLES) {
        capture_done = true;
    } else {
        /* Restart DMA for the next chunk — exact pattern from MSDK example */
        MXC_DMA_ReleaseChannel(0);
        MXC_I2S_RXDMAConfig(dma_chunk, CHUNK_SAMPLES * sizeof(int32_t));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void audio_capture_init(void)
{
    /* DMA must be initialised before I2S, and before registering
     * the callback, so that channel handles are valid. */
    MXC_DMA_Init();

    /*
     * Mirror the MSDK I2S_DMA example exactly.
     * Key: pass rxData pointing to a real buffer so MXC_I2S_Init()
     * enables the I2S RX path.  Without this the FIFO never fills
     * and the DMA waits indefinitely.
     */
    mxc_i2s_req_t req;
    memset(&req, 0, sizeof(req));

    req.wordSize    = MXC_I2S_WSIZE_WORD;            /* 32-bit       */
    req.sampleSize  = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.bitsWord    = 32;
    req.adjust      = MXC_I2S_ADJUST_LEFT;
    req.justify     = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity  = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    req.stereoMode  = MXC_I2S_MONO_LEFT_CH;
    req.bitOrder    = MXC_I2S_MSB_FIRST;
    req.clkdiv      = 5;            /* 12.288 MHz / 12 / 64 = 16 kHz */
    req.rawData     = NULL;
    req.txData      = NULL;
    req.rxData      = dma_chunk;    /* real buffer — enables I2S RX   */
    req.length      = CHUNK_SAMPLES;

    int ret = MXC_I2S_Init(&req);
    if (ret != E_NO_ERROR) {
        printf("[audio] MXC_I2S_Init failed: %d\n", ret);
        return;
    }

    MXC_I2S_SetRXThreshold(4);
    MXC_I2S_RegisterDMACallback(i2s_dma_callback);

    NVIC_EnableIRQ(DMA0_IRQn);
    /* __enable_irq() is already called by startup code; safe to call again */
    __enable_irq();

    printf("[audio] I2S init OK — clkdiv=%d, 32-bit mono, DMA chunk=%d\n",
           5, CHUNK_SAMPLES);
}

void audio_capture_start(void)
{
    samples_collected = 0;
    capture_done      = false;
    memset(pcm_buf, 0, sizeof(pcm_buf));

    /* Release in case Init or a previous capture claimed the channel */
    MXC_DMA_ReleaseChannel(0);

    /* Start first DMA chunk — callback chains the rest */
    int ret = MXC_I2S_RXDMAConfig(dma_chunk, CHUNK_SAMPLES * sizeof(int32_t));
    if (ret != E_NO_ERROR) {
        printf("[audio] RXDMAConfig failed: %d\n", ret);
        capture_done = true; /* prevent infinite wait */
    }
}

bool audio_capture_is_done(void)
{
    return capture_done;
}

int16_t *audio_capture_get_buffer(void)
{
    return pcm_buf;
}

void audio_capture_load_pcm(const int16_t *src, int n_samples)
{
    if (n_samples > AUDIO_CLIP_SAMPLES) n_samples = AUDIO_CLIP_SAMPLES;
    memcpy(pcm_buf, src, n_samples * sizeof(int16_t));
    capture_done = true;
}
