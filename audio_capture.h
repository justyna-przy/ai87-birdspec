#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Continuous sliding-window audio capture from the SPH0645 I2S microphone.
 *
 * Design:
 *   - pcm_buf[48000] holds the LATEST 3 seconds of audio at all times.
 *   - DMA fills in 256-sample chunks (chunked DMA from MSDK I2S_DMA example).
 *   - Every AUDIO_SLIDE_SAMPLES new samples (1 second), snapshot_ready is set.
 *   - Main loop processes pcm_buf, then calls audio_capture_slide() to shift
 *     the window forward and re-arm DMA for the next second.
 *
 * RMS gating:
 *   - audio_capture_rms() computes RMS of the current pcm_buf window.
 *   - Skip spectrogram+inference if RMS < AUDIO_RMS_THRESHOLD.
 */

#define AUDIO_SAMPLE_RATE     16000
#define AUDIO_CLIP_SAMPLES    48000   /* 3 seconds × 16 kHz            */
#define AUDIO_SLIDE_SAMPLES   16000   /* slide 1 second per update      */

/* RMS silence gate threshold (0–32767 scale).
 * SPH0645 floor noise ≈ 100–150 RMS in a quiet room.
 * Bird calls are typically several thousand RMS.
 * Tune this for your environment. */
#define AUDIO_RMS_THRESHOLD   300.0f

/* Initialise I2S peripheral and DMA. Call once at boot. */
void audio_capture_init(void);

/*
 * Start continuous capture.  Fills pcm_buf from scratch (first snapshot
 * arrives after 3 seconds).  Non-blocking — DMA runs in background.
 */
void audio_capture_start(void);

/*
 * Stop continuous capture (e.g. before entering UART mode).
 */
void audio_capture_stop(void);

/*
 * Returns true when a new 1-second slice has been collected and pcm_buf
 * holds a fresh 3-second window ready for spectrogram computation.
 */
bool audio_capture_snapshot_ready(void);

/*
 * Shift the sliding window forward by AUDIO_SLIDE_SAMPLES and re-arm DMA.
 * Call this from the main loop after each snapshot has been processed.
 */
void audio_capture_slide(void);

/*
 * Returns a pointer to the current 48000-sample int16 PCM window.
 * Valid to read after audio_capture_snapshot_ready() returns true,
 * up until audio_capture_slide() is called.
 */
int16_t *audio_capture_get_buffer(void);

/*
 * Alias of audio_capture_get_buffer() — used by uart_cmd.c LOAD_PCM path.
 */
int16_t *audio_capture_get_pcm_buf(void);

/*
 * Legacy one-shot poll — returns true once the first 3-second window is full.
 * Used by uart_cmd.c REC handler which calls audio_capture_start() and waits.
 */
bool audio_capture_is_done(void);

/*
 * Inject pre-recorded PCM (e.g. from UART LOAD_PCM).
 * Copies n samples into pcm_buf and marks snapshot ready.
 */
void audio_capture_load_pcm(const int16_t *src, int n);

/*
 * Compute RMS amplitude of the current pcm_buf window (0–32767 scale).
 * Use before inference to gate on silence.
 */
float audio_capture_rms(void);

#endif /* AUDIO_CAPTURE_H */
