#ifndef INFERENCE_H
#define INFERENCE_H

#include <stdint.h>
#include "cnn.h"

#define INFERENCE_TOP_K_MAX 5

typedef struct {
    int     class_idx;
    float   confidence; /* 0.0 – 100.0 */
} result_t;

/*
 * Call once at boot.  Enables CNN peripheral, loads weights, configures
 * all layers.  Keeps CNN clock at PLL/DIV4 (50 MHz) between inferences.
 */
void inference_init(void);

/*
 * Run a single inference on a 64×128 int8 tensor (8192 bytes, CHW layout,
 * channel-0 only).  Boosts CNN clock to PLL/DIV1 (200 MHz), runs, then
 * restores PLL/DIV4.  Fills `results[0..top_k-1]` sorted descending by
 * confidence.  top_k must be <= INFERENCE_TOP_K_MAX.
 *
 * Returns the CNN inference latency in microseconds (from cnn_time ISR).
 */
uint32_t inference_run(const int8_t *tensor, result_t *results, int top_k);

/*
 * Run the built-in known-answer test using sampledata.h / sampleoutput.h.
 * Prints PASS or FAIL over UART.  Returns CNN_OK or CNN_FAIL.
 */
int inference_kat(void);

#endif /* INFERENCE_H */
