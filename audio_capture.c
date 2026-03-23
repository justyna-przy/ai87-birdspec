/*
 * audio_capture.c — Sliding-window I2S DMA capture for MAX78002 EV kit.
 *
 * DMA pattern taken from MSDK Examples/MAX78002/I2S_DMA (proven working).
 *
 * Hardware: SPH0645 I2S MEMS mic on EV kit header JH4.
 *   18-bit audio, left-justified in a 32-bit I2S word.
 *   Top 16 bits extracted by right-shifting 16 in the callback.
 *
 * Clock: clkdiv=5 with ERFO source (12.288 MHz):
 *   BCLK = 12.288 MHz / (2*(5+1)) = 1.024 MHz
 *   LRCK = 1.024 MHz / 64          = 16 kHz  ✓
 *
 * Sliding window:
 *   pcm_buf[48000] always contains the latest 3 seconds.
 *   DMA fills in CHUNK_SAMPLES=256 chunks → callback chains next chunk.
 *   Every AUDIO_SLIDE_SAMPLES=16000 new samples, snapshot_ready is raised.
 *   Main loop processes pcm_buf, then calls audio_capture_slide() which:
 *     1. memmove(pcm_buf, pcm_buf+16000, 64000 bytes)  — drop oldest 1 s
 *     2. Resets write pointer to 32000
 *     3. Re-arms DMA for the next 16000-sample slice
 */

#include <math.h>
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

#define CHUNK_SAMPLES  256

static int32_t dma_chunk[CHUNK_SAMPLES];          /* 1 KB — DMA target  */
static int16_t pcm_buf[AUDIO_CLIP_SAMPLES];       /* 96 KB — 3-s window */

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

static volatile int  ring_pos        = 0;     /* next write position   */
static volatile bool snapshot_ready  = false; /* 16000 new samples in  */
static volatile bool capture_active  = false;
static volatile int  dma_channel     = -1;    /* channel acquired by MXC_I2S_RXDMAConfig */

/* ------------------------------------------------------------------ */
/* DMA IRQ — register handlers for channels 0 AND 1.                  */
/* MXC_I2S_RXDMAConfig acquires the lowest free channel via            */
/* MXC_DMA_AcquireChannel().  If the SPI/TFT driver holds channel 0   */
/* during a display update, I2S falls back to channel 1.  Without a   */
/* DMA1 handler the interrupt is lost and audio capture freezes.       */
/* ------------------------------------------------------------------ */

void DMA0_IRQHandler(void) { MXC_DMA_Handler(); }
void DMA1_IRQHandler(void) { MXC_DMA_Handler(); }

/* ------------------------------------------------------------------ */
/* DMA completion callback                                              */
/* ------------------------------------------------------------------ */

static void i2s_dma_callback(int ch, int err)
{
    (void)ch;

    if (err != E_NO_ERROR) {
        /* On error: mark snapshot ready so the main loop isn't blocked */
        snapshot_ready = true;
        return;
    }

    /* How many samples can we still fit before we hit the end? */
    int space = AUDIO_CLIP_SAMPLES - ring_pos;
    int n     = (CHUNK_SAMPLES < space) ? CHUNK_SAMPLES : space;

    for (int i = 0; i < n; i++) {
        pcm_buf[ring_pos + i] = (int16_t)(dma_chunk[i] >> 16);
    }
    ring_pos += n;

    if (ring_pos >= AUDIO_CLIP_SAMPLES) {
        /*
         * Window is full — signal main loop.
         * Do NOT re-arm here; audio_capture_slide() will re-arm after
         * the main loop has finished processing this window.
         */
        snapshot_ready = true;
    } else if (capture_active) {
        /* Release the channel we were given (ch), then re-acquire.
         * Using ch (not hardcoded 0) ensures we free the right channel
         * even if SPI/TFT pushed us off channel 0. */
        MXC_DMA_ReleaseChannel(ch);
        int new_ch = MXC_I2S_RXDMAConfig(dma_chunk,
                                          CHUNK_SAMPLES * sizeof(int32_t));
        if (new_ch >= 0) {
            dma_channel = new_ch;
            /* Enable NVIC for whichever channel was acquired */
            NVIC_EnableIRQ((IRQn_Type)(DMA0_IRQn + new_ch));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Initialisation                                                       */
/* ------------------------------------------------------------------ */

void audio_capture_init(void)
{
    MXC_DMA_Init();

    /*
     * Replicate the MSDK I2S_DMA example init exactly — these extra fields
     * (sampleSize, bitsWord, adjust) are required for correct 32-bit operation.
     * rxData must point to a real buffer to enable the I2S RX path.
     */
    mxc_i2s_req_t req;
    memset(&req, 0, sizeof(req));

    req.wordSize    = MXC_I2S_WSIZE_WORD;
    req.sampleSize  = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.bitsWord    = 32;
    req.adjust      = MXC_I2S_ADJUST_LEFT;
    req.justify     = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity  = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    req.stereoMode  = MXC_I2S_MONO_LEFT_CH;
    req.bitOrder    = MXC_I2S_MSB_FIRST;
    req.clkdiv      = 5;
    req.rawData     = NULL;
    req.txData      = NULL;
    req.rxData      = dma_chunk;    /* real buffer — enables I2S RX path */
    req.length      = CHUNK_SAMPLES;

    int ret = MXC_I2S_Init(&req);
    if (ret != E_NO_ERROR) {
        printf("[audio] MXC_I2S_Init failed: %d\n", ret);
        return;
    }

    MXC_I2S_SetRXThreshold(4);
    MXC_I2S_RegisterDMACallback(i2s_dma_callback);

    NVIC_EnableIRQ(DMA0_IRQn);
    __enable_irq();

    printf("[audio] I2S init OK — 16 kHz mono 32-bit, chunk=%d samples\n",
           CHUNK_SAMPLES);
}

/* ------------------------------------------------------------------ */
/* Start / stop                                                         */
/* ------------------------------------------------------------------ */

void audio_capture_start(void)
{
    ring_pos       = 0;
    snapshot_ready = false;
    capture_active = true;
    memset(pcm_buf, 0, sizeof(pcm_buf));

    if (dma_channel >= 0)
        MXC_DMA_ReleaseChannel(dma_channel);
    int ch = MXC_I2S_RXDMAConfig(dma_chunk, CHUNK_SAMPLES * sizeof(int32_t));
    if (ch < E_NO_ERROR) {
        printf("[audio] RXDMAConfig failed: %d\n", ch);
        snapshot_ready = true; /* prevent infinite wait */
    } else {
        dma_channel = ch;
        NVIC_EnableIRQ((IRQn_Type)(DMA0_IRQn + ch));
    }
}

void audio_capture_stop(void)
{
    capture_active = false;
    MXC_I2S_RXDisable();
}

/* ------------------------------------------------------------------ */
/* Sliding window                                                       */
/* ------------------------------------------------------------------ */

void audio_capture_slide(void)
{
    /*
     * Shift out the oldest AUDIO_SLIDE_SAMPLES (1 second) and make room
     * for the next second.  memmove is safe with overlapping regions.
     * At 120 MHz this moves 64 KB in ~130 µs — fine in the main loop.
     */
    memmove(pcm_buf,
            pcm_buf + AUDIO_SLIDE_SAMPLES,
            (AUDIO_CLIP_SAMPLES - AUDIO_SLIDE_SAMPLES) * sizeof(int16_t));

    ring_pos       = AUDIO_CLIP_SAMPLES - AUDIO_SLIDE_SAMPLES; /* = 32000 */
    snapshot_ready = false;
    capture_active = true;

    if (dma_channel >= 0)
        MXC_DMA_ReleaseChannel(dma_channel);
    int ch = MXC_I2S_RXDMAConfig(dma_chunk, CHUNK_SAMPLES * sizeof(int32_t));
    if (ch >= 0) {
        dma_channel = ch;
        NVIC_EnableIRQ((IRQn_Type)(DMA0_IRQn + ch));
    }
}

/* ------------------------------------------------------------------ */
/* Public accessors                                                     */
/* ------------------------------------------------------------------ */

bool audio_capture_snapshot_ready(void)
{
    return snapshot_ready;
}

/* Legacy one-shot poll used by uart_cmd.c REC handler */
bool audio_capture_is_done(void)
{
    return snapshot_ready;
}

int16_t *audio_capture_get_buffer(void)
{
    return pcm_buf;
}

int16_t *audio_capture_get_pcm_buf(void)
{
    return pcm_buf;
}

void audio_capture_load_pcm(const int16_t *src, int n)
{
    if (n > AUDIO_CLIP_SAMPLES) n = AUDIO_CLIP_SAMPLES;
    memcpy(pcm_buf, src, (size_t)n * sizeof(int16_t));
    snapshot_ready = true;
}

float audio_capture_rms(void)
{
    int64_t sum_sq = 0;
    for (int i = 0; i < AUDIO_CLIP_SAMPLES; i++) {
        int32_t s = pcm_buf[i];
        sum_sq += s * s;
    }
    return sqrtf((float)(sum_sq / AUDIO_CLIP_SAMPLES));
}
