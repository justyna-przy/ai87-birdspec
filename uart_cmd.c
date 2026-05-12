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
#include "pmon_gpio.h"
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
extern uint32_t  g_last_spec_us;
extern uint32_t  g_last_total_us;

/* DWT microsecond timer helper — DWT must be enabled in main() before use.
 * Handles 32-bit wraparound correctly via unsigned subtraction. */
#define SPEC_TIMER MXC_TMR1
#define TOTAL_TIMER MXC_TMR2

#define PMON_CNN_REPEATS          100U
#define PMON_FULL_REPEATS_DEFAULT  10U
#define PMON_FULL_REPEATS_MAX      50U
#define PMON_IDLE_MS             1000U
#define PMON_STAGE_GAP_MS         500U

static const char g_fw_git_hash[] =
    "04a57d4ced1f8029385b53ddb6e97b8353ad3f15";

static const char g_dw1_ai8xize_cmd[] =
    "ai8xize.py --verbose --overwrite --test-dir demos --prefix ai87-birdspec-dw1 "
    "--checkpoint-file trained/birdspec_dw1_q8.pth.tar --config-file "
    "networks/birdspec-dw1-ai87.yaml --sample-input tests/sample_birdspec.npy "
    "--device MAX78002 --compact-data --softmax";

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

/* Send a null-terminated string followed by newline.
 * Uses MXC_UART_WriteCharacter which blocks per byte until TX FIFO has
 * space — the same blocking path used by printf/_write.  This avoids
 * the silent-drop risk of WriteTXFIFO and is proven to work. */
void uart_cmd_send(const char *msg)
{
    const uint8_t *p = (const uint8_t *)msg;
    while (*p)
        MXC_UART_WriteCharacter(CMD_UART, *p++);
    MXC_UART_WriteCharacter(CMD_UART, '\n');
}

/* Build the top-k JSON and send it.
 * Energy estimates use datasheet typical active-power figures:
 *   CNN accelerator @ 200 MHz CNN clock : ~4.4 mW  →  4.4 nJ/µs
 *   Cortex-M4       @ 120 MHz IPO       : ~10  mW  → 10   nJ/µs
 */
static void send_topk(const result_t *results, int k, uint32_t latency_us)
{
    uint32_t cnn_nj  = latency_us      * 44 / 10;  /* 4.4 nJ/µs */
    uint32_t spec_nj = g_last_spec_us  * 10;        /* 10  nJ/µs */
    uint32_t tot_nj  = cnn_nj + spec_nj;

    char buf[640];
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
                  "],\"latency_us\":%lu,\"cnn_latency_us\":%lu,\"spec_us\":%lu,\"total_us\":%lu"
                  ",\"energy_kind\":\"duration_estimate\""
                  ",\"cnn_nj\":%lu,\"spec_nj\":%lu,\"total_nj\":%lu"
                  ",\"cnn_energy_est_nj\":%lu,\"spec_energy_est_nj\":%lu"
                  ",\"total_energy_est_nj\":%lu}",
                  (unsigned long)latency_us,
                  (unsigned long)latency_us,
                  (unsigned long)g_last_spec_us,
                  (unsigned long)g_last_total_us,
                  (unsigned long)cnn_nj,
                  (unsigned long)spec_nj,
                  (unsigned long)tot_nj,
                  (unsigned long)cnn_nj,
                  (unsigned long)spec_nj,
                  (unsigned long)tot_nj);

    uart_cmd_send(buf);
}

/* Console log for device-side DSP + CNN energy estimate.
 * This does not alter the JSON protocol consumed by the host API. */
static void log_energy_metrics(const char *mode, uint32_t spec_us, uint32_t cnn_us)
{
    uint32_t cnn_nj  = cnn_us  * 44 / 10;  /* 4.4 nJ/us */
    uint32_t spec_nj = spec_us * 10;       /* 10  nJ/us */
    uint32_t tot_nj  = cnn_nj + spec_nj;

    printf("[energy_estimate] mode=%s kind=duration_based spec_us=%lu cnn_us=%lu total_us=%lu spec_nj=%lu cnn_nj=%lu total_nj=%lu\n",
           mode,
           (unsigned long)spec_us,
           (unsigned long)cnn_us,
           (unsigned long)(spec_us + cnn_us),
           (unsigned long)spec_nj,
           (unsigned long)cnn_nj,
           (unsigned long)tot_nj);
}

static void run_full_path_once(const int16_t *pcm_buf, int n_samples, int top_k,
                               uint32_t *spec_us, uint32_t *cnn_us, uint32_t *total_us)
{
    MXC_TMR_SW_Start(TOTAL_TIMER);
    MXC_TMR_SW_Start(SPEC_TIMER);
    spectrogram_compute(pcm_buf, n_samples, g_cnn_input);
    *spec_us = MXC_TMR_SW_Stop(SPEC_TIMER);
    *cnn_us = inference_run(g_cnn_input, g_results, top_k);
    *total_us = MXC_TMR_SW_Stop(TOTAL_TIMER);

    g_last_spec_us = *spec_us;
    g_last_latency_us = *cnn_us;
    g_last_total_us = *total_us;
}

static void handle_pmon_info(void)
{
    printf("[pmon] fw_git=%s\n", g_fw_git_hash);
    printf("[pmon] model=dw1 ai8xize_cmd=\"%s\"\n", g_dw1_ai8xize_cmd);
    printf("[pmon] trigger_map=sys=P1.6/JP18/TRIG1 cnn=P1.7/JP19/TRIG2\n");
    printf("[pmon] ai8xize_energy_flag_present=no\n");
    printf("[pmon] default_energy_reporting=duration_estimate unless PMON is captured\n");

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"pmon_info\","
                  "\"fw_git\":\"04a57d4ced1f8029385b53ddb6e97b8353ad3f15\","
                  "\"ai8xize_energy\":false,"
                  "\"trig1\":\"P1.6/JP18/SYS\","
                  "\"trig2\":\"P1.7/JP19/CNN\","
                  "\"energy_default\":\"duration_estimate\"}");
}

static void handle_pmon_cnn(void)
{
    uint64_t cnn_us_sum = 0;
    uint32_t cnn_us_mean;

    printf("[pmon] mode=cnn_power repeats=%u requires_jp18=yes requires_jp19=yes\n",
           (unsigned)PMON_CNN_REPEATS);
    printf("[pmon] stage=idle window=SYS_START..SYS_COMPLETE duration_ms=%u\n",
           (unsigned)PMON_IDLE_MS);
    printf("[pmon] stage=kernel_load window=CNN_START..CNN_COMPLETE repeats=%u includes=weights_only\n",
           (unsigned)PMON_CNN_REPEATS);
    printf("[pmon] stage=input_load window=CNN_START..CNN_COMPLETE repeats=%u includes=input_load_only\n",
           (unsigned)PMON_CNN_REPEATS);
    printf("[pmon] stage=input_plus_inference window=CNN_START..CNN_COMPLETE repeats=%u includes=input_load_and_cnn_only excludes=output_unload_softmax\n",
           (unsigned)PMON_CNN_REPEATS);
    printf("[pmon] note=current_tensor_source=g_cnn_input load_via_LOAD_PCM_or_LOAD_SPEC_before_measurement_if_needed\n");

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"pmon_cnn_begin\","
                  "\"mode\":\"cnn_power\",\"repeats\":100}");

    SYS_START;
    MXC_Delay(MXC_DELAY_MSEC(PMON_IDLE_MS));
    SYS_COMPLETE;
    MXC_Delay(MXC_DELAY_MSEC(PMON_STAGE_GAP_MS));

    CNN_START;
    for (uint32_t i = 0; i < PMON_CNN_REPEATS; i++) {
        cnn_load_weights();
    }
    CNN_COMPLETE;
    MXC_Delay(MXC_DELAY_MSEC(PMON_STAGE_GAP_MS));

    CNN_START;
    for (uint32_t i = 0; i < PMON_CNN_REPEATS; i++) {
        inference_load_input(g_cnn_input);
    }
    CNN_COMPLETE;
    MXC_Delay(MXC_DELAY_MSEC(PMON_STAGE_GAP_MS));

    CNN_START;
    for (uint32_t i = 0; i < PMON_CNN_REPEATS; i++) {
        inference_load_input(g_cnn_input);
        cnn_us_sum += inference_run_cnn_only();
    }
    CNN_COMPLETE;

    cnn_us_mean = (uint32_t)(cnn_us_sum / PMON_CNN_REPEATS);
    g_last_spec_us = 0;
    g_last_latency_us = cnn_us_mean;
    g_last_total_us = cnn_us_mean;

    printf("[pmon] summary mode=cnn_power repeats=%u cnn_latency_us_mean=%lu per_inference_energy_obtain=PMON(input_plus_inference)-PMON(input_load_only)\n",
           (unsigned)PMON_CNN_REPEATS,
           (unsigned long)cnn_us_mean);

    {
        char buf[320];
        snprintf(buf, sizeof(buf),
                 "{\"status\":\"ok\",\"state\":\"pmon_cnn_complete\","
                 "\"mode\":\"cnn_power\",\"repeats\":%u,"
                 "\"cnn_latency_us_mean\":%lu,"
                 "\"energy_kind\":\"pmon_required_for_measured_energy\","
                 "\"duration_estimate_cnn_nj\":%lu}",
                 (unsigned)PMON_CNN_REPEATS,
                 (unsigned long)cnn_us_mean,
                 (unsigned long)(cnn_us_mean * 44 / 10));
        uart_cmd_send(buf);
    }
}

static void handle_pmon_full(int repeats)
{
    uint64_t spec_sum = 0;
    uint64_t cnn_sum = 0;
    uint64_t total_sum = 0;
    uint32_t spec_us;
    uint32_t cnn_us;
    uint32_t total_us;
    int16_t *pcm_buf = audio_capture_get_buffer();

    if (repeats < 1) {
        repeats = PMON_FULL_REPEATS_DEFAULT;
    }
    if (repeats > (int)PMON_FULL_REPEATS_MAX) {
        repeats = PMON_FULL_REPEATS_MAX;
    }

    printf("[pmon] mode=system_power repeats=%d requires_jp18=yes requires_jp19=yes\n", repeats);
    printf("[pmon] system_window=SYS_START..SYS_COMPLETE includes=spectrogram+cnn_inference+softmax_topk excludes=uart_load_pcm\n");
    printf("[pmon] note=divide_pmon_energy_by_repeats_for_per_clip_energy\n");
    printf("[pmon] note=current_pcm_source=audio_capture_buffer load_via_LOAD_PCM_or_REC_before_measurement_if_needed\n");

    {
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "{\"status\":\"ok\",\"state\":\"pmon_full_begin\","
                 "\"mode\":\"system_power\",\"repeats\":%d}",
                 repeats);
        uart_cmd_send(buf);
    }

    pmon_cnn_hold_begin();
    SYS_START;
    for (int i = 0; i < repeats; i++) {
        run_full_path_once(pcm_buf, AUDIO_CLIP_SAMPLES, 3, &spec_us, &cnn_us, &total_us);
        spec_sum += spec_us;
        cnn_sum += cnn_us;
        total_sum += total_us;
    }
    SYS_COMPLETE;
    pmon_cnn_hold_end();

    g_last_spec_us = (uint32_t)(spec_sum / (uint32_t)repeats);
    g_last_latency_us = (uint32_t)(cnn_sum / (uint32_t)repeats);
    g_last_total_us = (uint32_t)(total_sum / (uint32_t)repeats);

    printf("[pmon] summary mode=system_power repeats=%d spec_us_mean=%lu cnn_latency_us_mean=%lu total_us_mean=%lu total_window_us=%lu\n",
           repeats,
           (unsigned long)g_last_spec_us,
           (unsigned long)g_last_latency_us,
           (unsigned long)g_last_total_us,
           (unsigned long)total_sum);

    {
        char buf[384];
        snprintf(buf, sizeof(buf),
                 "{\"status\":\"ok\",\"state\":\"pmon_full_complete\","
                 "\"mode\":\"system_power\",\"repeats\":%d,"
                 "\"spec_us_mean\":%lu,\"cnn_latency_us_mean\":%lu,"
                 "\"total_us_mean\":%lu,\"total_window_us\":%lu,"
                 "\"energy_kind\":\"pmon_required_for_measured_energy\","
                 "\"duration_estimate_spec_nj\":%lu,"
                 "\"duration_estimate_cnn_nj\":%lu,"
                 "\"duration_estimate_total_nj\":%lu}",
                 repeats,
                 (unsigned long)g_last_spec_us,
                 (unsigned long)g_last_latency_us,
                 (unsigned long)g_last_total_us,
                 (unsigned long)total_sum,
                 (unsigned long)(g_last_spec_us * 10),
                 (unsigned long)(g_last_latency_us * 44 / 10),
                 (unsigned long)(g_last_spec_us * 10 + g_last_latency_us * 44 / 10));
        uart_cmd_send(buf);
    }
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
    int16_t *pcm_buf = audio_capture_get_pcm_buf(); /* 48000 int16 */
    uint8_t *raw     = (uint8_t *)pcm_buf;
    int received = 0;

    /* Idle-iteration timeout: abort if no byte arrives for too long.
     * DWT CYCCNT cannot be relied on without a debugger attached, so
     * we count consecutive empty-FIFO iterations instead.
     * At 120 MHz with ~100 cycles per uart_getc call the loop runs
     * ~1.2M times/second.  5 000 000 iterations ≈ 4 s of silence —
     * well above the ~87 µs gap between back-to-back bytes at 115200
     * baud, so normal reception never triggers this. */
    uint32_t idle_count = 0;
    while (received < n_bytes) {
        int c = uart_getc();
        if (c >= 0) {
            raw[received++] = (uint8_t)c;
            idle_count = 0;         /* reset on every byte received */
        } else if (++idle_count > 5000000UL) {
            char _terr[96];
            snprintf(_terr, sizeof(_terr),
                     "{\"status\":\"error\",\"msg\":\"PCM timeout\","
                     "\"received\":%d,\"expected\":%d}",
                     received, n_bytes);
            uart_cmd_send(_terr);
            return;
        }
    }

    int n_samples = n_bytes / 2; /* int16 samples */

    /* Compute spectrogram + run inference */
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"computing_spectrogram\"}");
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"inferring\"}");
    run_full_path_once(pcm_buf, n_samples, 3,
                       &g_last_spec_us, &g_last_latency_us, &g_last_total_us);

    send_topk(g_results, 3, g_last_latency_us);
}

static void handle_load_pcm_only(int n_bytes)
{
    if (n_bytes < 2 || n_bytes > 96000) {
        uart_cmd_send("{\"status\":\"error\",\"msg\":\"LOAD_PCM_ONLY bad byte count\"}");
        return;
    }

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"receiving_pcm\"}");

    int16_t *pcm_buf = audio_capture_get_pcm_buf();
    uint8_t *raw = (uint8_t *)pcm_buf;
    int received = 0;
    uint32_t idle_count = 0;

    while (received < n_bytes) {
        int c = uart_getc();
        if (c >= 0) {
            raw[received++] = (uint8_t)c;
            idle_count = 0;
        } else if (++idle_count > 5000000UL) {
            char terr[96];
            snprintf(terr, sizeof(terr),
                     "{\"status\":\"error\",\"msg\":\"PCM timeout\","
                     "\"received\":%d,\"expected\":%d}",
                     received, n_bytes);
            uart_cmd_send(terr);
            return;
        }
    }

    if (n_bytes < 96000) {
        memset(raw + n_bytes, 0, (size_t)(96000 - n_bytes));
    }

    g_last_spec_us = 0;
    g_last_latency_us = 0;
    g_last_total_us = 0;
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"pcm_loaded\"}");
}

static void handle_prep_spec(void)
{
    uart_cmd_send("{\"status\":\"ok\",\"state\":\"computing_spectrogram\"}");
    MXC_TMR_SW_Start(SPEC_TIMER);
    spectrogram_compute(audio_capture_get_buffer(), AUDIO_CLIP_SAMPLES, g_cnn_input);
    g_last_spec_us = MXC_TMR_SW_Stop(SPEC_TIMER);
    g_last_latency_us = 0;
    g_last_total_us = g_last_spec_us;

    {
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "{\"status\":\"ok\",\"state\":\"spec_ready\",\"spec_us\":%lu}",
                 (unsigned long)g_last_spec_us);
        uart_cmd_send(buf);
    }
}

/* ------------------------------------------------------------------ */
/* LOAD_SPEC handler — bypass spectrogram, load int8 tensor directly  */
/* ------------------------------------------------------------------ */

/*
 * Receive 8192 raw int8 bytes directly into g_cnn_input, then run inference.
 * Protocol: host sends "LOAD_SPEC 8192\n" then 8192 bytes.
 * This bypasses spectrogram_compute entirely — used to isolate whether
 * wrong predictions come from the spectrogram pipeline or the CNN weights.
 */
static void handle_load_spec(int n_bytes)
{
    if (n_bytes != 8192) {
        uart_cmd_send("{\"status\":\"error\",\"msg\":\"LOAD_SPEC expects exactly 8192 bytes\"}");
        return;
    }

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"receiving_spec\"}");

    uint8_t *raw = (uint8_t *)g_cnn_input;
    int received = 0;
    uint32_t idle_count = 0;

    while (received < n_bytes) {
        int c = uart_getc();
        if (c >= 0) {
            raw[received++] = (uint8_t)c;
            idle_count = 0;
        } else if (++idle_count > 5000000UL) {
            char _terr[96];
            snprintf(_terr, sizeof(_terr),
                     "{\"status\":\"error\",\"msg\":\"SPEC timeout\","
                     "\"received\":%d,\"expected\":%d}",
                     received, n_bytes);
            uart_cmd_send(_terr);
            return;
        }
    }

    uart_cmd_send("{\"status\":\"ok\",\"state\":\"inferring\"}");
    g_last_spec_us = 0;
    g_last_latency_us = inference_run(g_cnn_input, g_results, 3);
    g_last_total_us = g_last_latency_us;
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
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"idle\",\"classes\":51}");

    } else if (strcmp(line, "INFER") == 0) {
        /* Run inference on whatever is already in g_cnn_input */
        g_last_spec_us = 0;
        g_last_latency_us = inference_run(g_cnn_input, g_results, 3);
        g_last_total_us = g_last_latency_us;
        send_topk(g_results, 3, g_last_latency_us);

    } else if (strncmp(line, "GET_TOPK", 8) == 0) {
        int k = 3;
        if (line[8] == ' ') k = atoi(line + 9);
        if (k < 1) k = 1;
        if (k > INFERENCE_TOP_K_MAX) k = INFERENCE_TOP_K_MAX;
        send_topk(g_results, k, g_last_latency_us);

    } else if (strncmp(line, "LOAD_SPEC", 9) == 0) {
        int n_bytes = 0;
        if (line[9] == ' ') n_bytes = atoi(line + 10);
        handle_load_spec(n_bytes);

    } else if (strncmp(line, "LOAD_PCM_ONLY", 13) == 0) {
        int n_bytes = 0;
        if (line[13] == ' ') n_bytes = atoi(line + 14);
        handle_load_pcm_only(n_bytes);

    } else if (strncmp(line, "LOAD_PCM", 8) == 0) {
        int n_bytes = 0;
        if (line[8] == ' ') n_bytes = atoi(line + 9);
        handle_load_pcm(n_bytes);

    } else if (strcmp(line, "PREP_SPEC") == 0) {
        handle_prep_spec();

    } else if (strcmp(line, "BATCH") == 0) {
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"batch_start\"}");
        sd_batch_run(0);

    } else if (strcmp(line, "PMON_INFO") == 0) {
        handle_pmon_info();

    } else if (strcmp(line, "PMON_CNN") == 0) {
        handle_pmon_cnn();

    } else if (strncmp(line, "PMON_FULL", 9) == 0) {
        int repeats = PMON_FULL_REPEATS_DEFAULT;
        if (line[9] == ' ') repeats = atoi(line + 10);
        handle_pmon_full(repeats);

    } else if (strcmp(line, "REC") == 0) {
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"recording\"}");
        audio_capture_start();
        while (!audio_capture_is_done()) {}

        /* RMS silence gate */
        float rms = audio_capture_rms();
        if (rms < AUDIO_RMS_THRESHOLD) {
            uart_cmd_send("{\"status\":\"ok\",\"state\":\"silence\"}");
            return;
        }

        uart_cmd_send("{\"status\":\"ok\",\"state\":\"computing_spectrogram\"}");
        uart_cmd_send("{\"status\":\"ok\",\"state\":\"inferring\"}");
        run_full_path_once(audio_capture_get_buffer(), 48000, 3,
                           &g_last_spec_us, &g_last_latency_us, &g_last_total_us);
        log_energy_metrics("rec", g_last_spec_us, g_last_latency_us);
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
    /* Console_Init() (called in Board_Init before main) already initialised
     * UART0 with MXC_UART_IBRO_CLK at 115200 baud.  Re-initialising here
     * with MXC_UART_APB_CLK would set a wrong divisor and break the port. */
    printf("[uart_cmd] UART0 ready at %u baud (IBRO clock).\n",
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
