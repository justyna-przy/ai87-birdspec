#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mxc.h"
#include "uart.h"
#include "uart_cmd.h"
#include "inference.h"
#include "spectrogram.h"
#include "audio_capture.h"
#include "species_labels.h"
#include "sd_batch.h"

/* UART instance used for commands */
#define CMD_UART    MXC_UART0
#define CMD_UART_IRQ MXC_UART0_GET_IRQ(0)

/* Receive line buffer */
#define RX_BUF_LEN  128
static char  rx_buf[RX_BUF_LEN];
static int   rx_pos = 0;

/* Shared CNN input/output buffers (owned by main, extern here) */
extern int8_t    g_cnn_input[64 * 128];
extern result_t  g_results[INFERENCE_TOP_K_MAX];
extern uint32_t  g_last_latency_us;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

/* Read one byte from UART, returns -1 if FIFO empty */
static int uart_getc(void)
{
    if (MXC_UART_GetRXFIFOAvailable(CMD_UART) == 0)
        return -1;
    uint8_t b;
    MXC_UART_ReadRXFIFO(CMD_UART, &b, 1);
    return (int)b;
}

/* Send a null-terminated string */
void uart_cmd_send(const char *msg)
{
    MXC_UART_WriteTXFIFO(CMD_UART, (const uint8_t *)msg, strlen(msg));
    /* Write a final newline */
    const uint8_t nl = '\n';
    MXC_UART_WriteTXFIFO(CMD_UART, &nl, 1);
}

/* Build the top-k JSON and send it */
static void send_topk(const result_t *results, int k, uint32_t latency_us)
{
    char buf[512];
    int  n = 0;

    n += snprintf(buf + n, sizeof(buf) - n,
                  "{\"status\":\"ok\",\"top%d\":[", k);

    for (int i = 0; i < k; i++) {
        int idx = results[i].class_idx;
        /* conf as integer tenths: e.g. 87.3% → digs=87, tens=3 */
        int digs = (int)results[i].confidence;
        int tens = (int)((results[i].confidence - digs) * 10.0f + 0.5f);
        n += snprintf(buf + n, sizeof(buf) - n,
                      "%s{\"idx\":%d,\"label\":\"%s\",\"common\":\"%s\",\"conf\":%d.%d}",
                      (i > 0 ? "," : ""),
                      idx,
                      species_sci_name[idx],
                      species_common_name[idx],
                      digs, tens);
    }
    n += snprintf(buf + n, sizeof(buf) - n,
                  "],\"latency_us\":%lu}", (unsigned long)latency_us);

    uart_cmd_send(buf);
}

/* ------------------------------------------------------------------ */
/* LOAD_PCM handler                                                     */
/* ------------------------------------------------------------------ */

/*
 * Receive raw int16-LE PCM from UART.
 * Protocol: host already sent "LOAD_PCM <n_bytes>\n".
 * We receive exactly n_bytes bytes into the audio capture buffer,
 * then compute the spectrogram and run inference.
 * n_bytes should be 96000 (= 48000 samples * 2 bytes).
 */
static void handle_load_pcm(int n_bytes)
{
    /* Clamp to maximum supported size */
    if (n_bytes < 2 || n_bytes > 96000) {
        uart_cmd_send("{\"status\":\"error\",\"msg\":\"LOAD_PCM bad byte count\"}");
        return;
    }

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"receiving_pcm\"}");

    /* Receive raw bytes directly into the audio capture buffer */
    int16_t *pcm_buf = audio_capture_get_buffer(); /* 48000 int16 */
    uint8_t *raw     = (uint8_t *)pcm_buf;
    int received = 0;

    while (received < n_bytes) {
        int c = uart_getc();
        if (c >= 0) {
            raw[received++] = (uint8_t)c;
        }
        /* No timeout — host is expected to send all bytes promptly */
    }

    int n_samples = n_bytes / 2; /* int16 samples */

    /* Compute spectrogram */
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"computing_spectrogram\"}");
    spectrogram_compute(pcm_buf, n_samples, g_cnn_input);

    /* Run inference */
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"inferring\"}");
    g_last_latency_us = inference_run(g_cnn_input, g_results, 3);

    send_topk(g_results, 3, g_last_latency_us);
}

/* ------------------------------------------------------------------ */
/* Command dispatcher                                                   */
/* ------------------------------------------------------------------ */

static void dispatch(const char *line)
{
    if (strcmp(line, "KAT") == 0) {
        int r = inference_kat();
        if (r == CNN_OK)
            uart_cmd_send("{\"status\":\"ok\",\"kat\":\"pass\"}");
        else
            uart_cmd_send("{\"status\":\"error\",\"kat\":\"fail\"}");

    } else if (strcmp(line, "STATUS") == 0) {
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"idle\",\"classes\":50}");

    } else if (strcmp(line, "INFER") == 0) {
        /* Run inference on whatever is already in g_cnn_input */
        g_last_latency_us = inference_run(g_cnn_input, g_results, 3);
        send_topk(g_results, 3, g_last_latency_us);

    } else if (strncmp(line, "GET_TOPK", 8) == 0) {
        int k = 3;
        if (line[8] == ' ') k = atoi(line + 9);
        if (k < 1) k = 1;
        if (k > INFERENCE_TOP_K_MAX) k = INFERENCE_TOP_K_MAX;
        send_topk(g_results, k, g_last_latency_us);

    } else if (strncmp(line, "LOAD_PCM", 8) == 0) {
        int n_bytes = 0;
        if (line[8] == ' ') n_bytes = atoi(line + 9);
        handle_load_pcm(n_bytes);

    } else if (strcmp(line, "BATCH") == 0) {
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"batch_start\"}");
        sd_batch_run(0);

    } else if (strcmp(line, "REC") == 0) {
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"recording\"}");
        audio_capture_start();
        while (!audio_capture_is_done()) {}
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"computing_spectrogram\"}");
        spectrogram_compute(audio_capture_get_buffer(), 48000, g_cnn_input);
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"inferring\"}");
        g_last_latency_us = inference_run(g_cnn_input, g_results, 3);
        send_topk(g_results, 3, g_last_latency_us);

    } else {
        char err[80];
        snprintf(err, sizeof(err),
                 "{\"status\":\"error\",\"msg\":\"unknown command: %.40s\"}", line);
        uart_cmd_send(err);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void uart_cmd_init(void)
{
    /* UART0 is already initialised by Board_Init() / Console_Init() at
     * 115200 baud.  No need to re-init here. */
    printf("[uart_cmd] UART0 command interface ready at %u baud.\n",
           UART_CMD_BAUD_NORMAL);
}

void uart_cmd_poll(void)
{
    int c;
    while ((c = uart_getc()) >= 0) {
        if (c == '\r') continue; /* ignore CR in CRLF */
        if (c == '\n') {
            rx_buf[rx_pos] = '\0';
            if (rx_pos > 0)
                dispatch(rx_buf);
            rx_pos = 0;
        } else if (rx_pos < RX_BUF_LEN - 1) {
            rx_buf[rx_pos++] = (char)c;
        }
    }
}
