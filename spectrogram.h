#ifndef SPECTROGRAM_H
#define SPECTROGRAM_H

#include <stdint.h>

/*
 * Compute a 64×128 int8 log-mel spectrogram from 16 kHz mono PCM audio.
 *
 * Pipeline (matches the offline Python/librosa training pipeline exactly):
 *   1. Hann-windowed 1024-pt STFT, hop=256, center=False → 184 frames
 *   2. Power spectrum |FFT|² for 513 bins (0..8000 Hz)
 *   3. Mel filterbank (128 filters, fmin=120, fmax=8000)
 *   4. 10·log10 dB, normalize by global max (ref=max), clamp [-80, 0]
 *   5. Remap to uint8 [0, 255], resize 128×184 → 64×128
 *   6. Flip freq axis (row 0 = lowest mel), uint8 → int8 (subtract 128)
 *   7. Per-sample z-score norm: clamp((val-mean)/std, -3,3)/3*127
 *      (matches Python PerSampleNorm transform applied during training)
 *
 * Output shape: (1, 64, 128) int8  →  8192 bytes, ready for cnn_load_input.
 */

/*
 * Initialise the CMSIS-DSP RFFT instance (call once at boot).
 */
void spectrogram_init(void);

/*
 * Compute the spectrogram.
 *   audio    : 16-bit signed PCM at 16 kHz, mono
 *   n_samples: number of samples (must be >= 1024; nominally 48000 for 3 s)
 *   out      : pointer to 64*128 = 8192 bytes output buffer (int8_t)
 */
void spectrogram_compute(const int16_t *audio, int n_samples, int8_t *out);

#endif /* SPECTROGRAM_H */
