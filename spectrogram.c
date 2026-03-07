#include <math.h>
#include <string.h>
#include "arm_math.h"
#include "spectrogram.h"
#include "mel_filterbank.h"
#include "hann_window.h"

/* ------------------------------------------------------------------ */
/* Constants matching the Python training pipeline exactly             */
/* ------------------------------------------------------------------ */
#define SPEC_SR          16000
#define SPEC_N_FFT       1024
#define SPEC_HOP         256
#define SPEC_N_MELS      128
#define SPEC_N_BINS      513    /* n_fft/2 + 1 */
#define SPEC_OUT_MELS    64     /* output rows after 2:1 downsampling */
#define SPEC_OUT_FRAMES  128    /* output cols after time-axis resize  */
#define SPEC_DB_MIN     -80.0f
#define SPEC_DB_MAX      0.0f

/*
 * Maximum number of STFT frames we ever process.
 * For n_samples=48000, hop=256, n_fft=1024, center=False:
 *   n_frames = floor((48000 - 1024) / 256) + 1 = 184
 */
#define SPEC_MAX_FRAMES  184

/* ------------------------------------------------------------------ */
/* Static buffers                                                       */
/* ------------------------------------------------------------------ */

/*
 * mel_buf[frame][mel_bin] — float32 mel energies for all frames.
 * 184 × 128 × 4 bytes = 94,208 bytes.  Declared static so it lives in
 * BSS (zero-init) rather than on the stack.
 */
static float mel_buf[SPEC_MAX_FRAMES][SPEC_N_MELS];

/*
 * FFT work buffer: arm_rfft_fast_f32 needs an in-place buffer of length
 * n_fft floats for real input + one extra float for the Nyquist bin.
 */
static float fft_buf[SPEC_N_FFT];

/* CMSIS-DSP RFFT instance */
static arm_rfft_fast_instance_f32 rfft_inst;

/* ------------------------------------------------------------------ */
/* Initialisation                                                       */
/* ------------------------------------------------------------------ */

void spectrogram_init(void)
{
    arm_rfft_fast_init_f32(&rfft_inst, SPEC_N_FFT);
}

/* ------------------------------------------------------------------ */
/* Core pipeline                                                        */
/* ------------------------------------------------------------------ */

void spectrogram_compute(const int16_t *audio, int n_samples, int8_t *out)
{
    /* ---- 1. STFT + mel filterbank ---------------------------------- */
    int n_frames = 0;

    for (int start = 0;
         start + SPEC_N_FFT <= n_samples && n_frames < SPEC_MAX_FRAMES;
         start += SPEC_HOP, n_frames++)
    {
        /* Copy and window one frame: int16 → float32, apply Hann */
        for (int i = 0; i < SPEC_N_FFT; i++) {
            fft_buf[i] = (float)audio[start + i] * hann_window[i];
        }

        /* Real FFT: in-place.  arm_rfft_fast_f32 packs the output as:
         *   buf[0]       = DC (real)
         *   buf[1]       = Nyquist (real)
         *   buf[2k],     = Re(bin k)   for k = 1 .. N/2-1
         *   buf[2k+1]    = Im(bin k)
         */
        arm_rfft_fast_f32(&rfft_inst, fft_buf, fft_buf, 0);

        /* Power spectrum: |FFT[k]|² for k = 0 .. N/2 (513 bins) */
        float power[SPEC_N_BINS];
        power[0] = fft_buf[0] * fft_buf[0];           /* DC          */
        power[SPEC_N_BINS - 1] = fft_buf[1] * fft_buf[1]; /* Nyquist */
        for (int k = 1; k < SPEC_N_BINS - 1; k++) {
            float re = fft_buf[2 * k];
            float im = fft_buf[2 * k + 1];
            power[k] = re * re + im * im;
        }

        /* Apply mel filterbank (sparse representation from mel_filterbank.h) */
        for (int m = 0; m < SPEC_N_MELS; m++) {
            float energy = 0.0f;
            int   off    = mel_weights_offset[m];
            int   start_bin = mel_start[m];
            int   len_bin   = mel_len[m];
            for (int b = 0; b < len_bin; b++) {
                energy += mel_weights_data[off + b] * power[start_bin + b];
            }
            mel_buf[n_frames][m] = energy;
        }
    }

    /* ---- 2. Convert to dB and find global max ---------------------- */
    float global_max_db = -1e30f;

    for (int f = 0; f < n_frames; f++) {
        for (int m = 0; m < SPEC_N_MELS; m++) {
            float e = mel_buf[f][m];
            /* 10*log10, guard against log(0) */
            float db = (e > 1e-10f) ? (10.0f * log10f(e)) : -100.0f;
            mel_buf[f][m] = db; /* reuse buffer for dB values */
            if (db > global_max_db) global_max_db = db;
        }
    }

    /* ---- 3. Normalize, clamp to [-80, 0], remap to [0, 255] ------- */
    /* dB_norm = dB - global_max  → clamp [-80, 0]
     * uint8   = (dB_norm + 80) / 80 * 255                             */
    for (int f = 0; f < n_frames; f++) {
        for (int m = 0; m < SPEC_N_MELS; m++) {
            float db_norm = mel_buf[f][m] - global_max_db;
            if (db_norm < SPEC_DB_MIN) db_norm = SPEC_DB_MIN;
            if (db_norm > SPEC_DB_MAX) db_norm = SPEC_DB_MAX;
            float u8 = (db_norm - SPEC_DB_MIN) / (SPEC_DB_MAX - SPEC_DB_MIN) * 255.0f;
            mel_buf[f][m] = u8; /* reuse buffer for uint8 values */
        }
    }

    /* ---- 4. Resize 128 mel × n_frames  →  64 × 128  --------------- */
    /* Then flip freq axis and convert uint8 → int8.
     *
     * Output layout: out[row * 128 + col]
     *   row 0 = lowest mel frequency (after freq flip)
     *   col 0 = earliest time frame
     *
     * Freq axis:   128 mel bins → 64 rows   (average adjacent pairs)
     * Time axis:   n_frames → 128 cols      (nearest-neighbour)
     *
     * After freq flip:
     *   out_row = 63 - mel_pair_index   (row 0 is lowest mel)
     *
     * The Python pipeline flips so that low freqs appear at the bottom
     * of the image (row index 0).  The CNN was trained on this layout.
     */
    for (int col = 0; col < SPEC_OUT_FRAMES; col++) {
        /* Nearest-neighbour time mapping */
        int src_frame = (col * n_frames + n_frames / 2) / SPEC_OUT_FRAMES;
        if (src_frame >= n_frames) src_frame = n_frames - 1;

        for (int mel_pair = 0; mel_pair < SPEC_OUT_MELS; mel_pair++) {
            /* Average two adjacent mel bins */
            float v0 = mel_buf[src_frame][mel_pair * 2];
            float v1 = mel_buf[src_frame][mel_pair * 2 + 1];
            float avg = (v0 + v1) * 0.5f;

            /* Clamp to uint8 range */
            int u8 = (int)(avg + 0.5f);
            if (u8 < 0)   u8 = 0;
            if (u8 > 255) u8 = 255;

            /* Freq flip: mel_pair 0 (lowest mel) → out row 0
             * Python stores low freqs at the bottom (row 0 = lowest).
             * We wrote 128 mel bins; pair 0 = lowest pair.
             * Flip means out_row = (SPEC_OUT_MELS - 1 - mel_pair).
             * But Python's flip puts row 0 = index 0 of the FLIPPED
             * array, which is the original row 127 (highest freq in
             * the unreflipped mel array).
             *
             * Actually librosa returns mel spec with row 0 = lowest mel.
             * Then flip_freq_axis=True reverses the rows so row 0 becomes
             * the HIGHEST mel.  The CNN was trained on that (highest freq
             * at the top visually = row 0 in the array).
             *
             * So we need: out_row = (SPEC_OUT_MELS - 1 - mel_pair)
             * where mel_pair 0 = lowest mel pair → goes to row 63.
             *                                                          */
            int out_row = (SPEC_OUT_MELS - 1) - mel_pair;

            /* int8 = uint8 - 128 */
            out[out_row * SPEC_OUT_FRAMES + col] = (int8_t)(u8 - 128);
        }
    }
}
