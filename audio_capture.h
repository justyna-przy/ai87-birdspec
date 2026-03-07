#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 16 kHz mono audio capture from the I2S SPH0645 microphone on the
 * MAX78002 EV kit.  Uses DMA double-buffering so a 3-second capture
 * runs without CPU involvement.
 *
 * Buffer size: 48000 samples × sizeof(int16_t) = 96 KB.
 *
 * SPH0645 note: the microphone outputs 18-bit left-justified data in a
 * 32-bit I2S frame.  We take only the upper 16 bits (right-shift by 14
 * inside the DMA callback) to produce standard int16 PCM.
 */

#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_CLIP_SAMPLES  48000   /* 3 seconds at 16 kHz */

/*
 * Initialise the I2S peripheral and DMA channel.  Call once at boot.
 */
void audio_capture_init(void);

/*
 * Start a new 3-second DMA capture.  Non-blocking.
 * The previous buffer contents are overwritten.
 */
void audio_capture_start(void);

/*
 * Returns true when the DMA capture is complete (3 s of audio ready).
 */
bool audio_capture_is_done(void);

/*
 * Returns a pointer to the captured int16 PCM buffer (AUDIO_CLIP_SAMPLES
 * samples).  Valid only after audio_capture_is_done() returns true.
 */
int16_t *audio_capture_get_buffer(void);

/*
 * Copy `n` int16 samples from `src` into the capture buffer.
 * Useful for injecting pre-recorded PCM (e.g., from UART or SD card).
 */
void audio_capture_load_pcm(const int16_t *src, int n);

#endif /* AUDIO_CAPTURE_H */
