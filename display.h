#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "inference.h"   /* for result_t */

/*
 * TFT display driver wrapper for the MAX78002 EV kit 3.5" display.
 *
 * Screen layout (landscape, 320×240 pixels assumed):
 *
 *  ┌───────────────────────────────────────┐
 *  │  BirdSpec — Irish Bird Classifier     │  header (20 px)
 *  ├───────────────────────────────────────┤
 *  │                                       │
 *  │    [128×64 spectrogram, scaled 2×]    │  spectrogram panel (~130 px)
 *  │    (grayscale, 256×128 px on screen)  │
 *  │                                       │
 *  ├───────────────────────────────────────┤
 *  │  1. Common Blackbird       87.3 %     │  results (30 px each)
 *  │  2. Song Thrush             5.1 %     │
 *  │  3. European Robin          3.2 %     │
 *  ├───────────────────────────────────────┤
 *  │  Latency: 42 ms                       │  footer (20 px)
 *  └───────────────────────────────────────┘
 */

/* Initialise TFT, clear screen, draw static layout chrome */
void display_init(void);

/* Show a short status message (e.g. "Recording...", "Processing...", "Ready") */
void display_status(const char *msg);

/*
 * Draw the 64×128 int8 spectrogram tensor as a grayscale image scaled 2×.
 * int8 values are mapped: [-128..127] → [0..255] grayscale.
 */
void display_spectrogram(const int8_t *tensor);

/*
 * Show the top-k inference results and latency in the results panel.
 * k must be <= INFERENCE_TOP_K_MAX.
 */
void display_results(const result_t *results, int k, uint32_t latency_us);

#endif /* DISPLAY_H */
