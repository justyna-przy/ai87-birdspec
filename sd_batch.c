#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mxc.h"
#include "ff.h"          /* FatFS from MSDK SDHC library */
#include "sd_batch.h"
#include "inference.h"
#include "uart_cmd.h"

#define SD_LABELS_PATH   "0:/BIRDSPEC/labels.csv"
#define SD_TENSOR_DIR    "0:/BIRDSPEC/"
#define TENSOR_SIZE      (64 * 128)   /* 8192 bytes */

/* FatFS filesystem object */
static FATFS fat_fs;
static bool  sd_mounted = false;

/* Per-sample CNN input buffer (8192 bytes) */
static int8_t tensor_buf[TENSOR_SIZE];

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

/*
 * Read one line from a FatFS file without using f_gets (which requires
 * FF_USE_STRFUNC enabled in ffconf.h).  Reads byte-by-byte until '\n'
 * or EOF.  Returns buf on success, NULL at EOF.
 */
static char *fatfs_readline(FIL *fp, char *buf, int maxlen)
{
    UINT br;
    int  n = 0;
    while (n < maxlen - 1) {
        if (f_read(fp, &buf[n], 1, &br) != FR_OK || br == 0)
            break;
        if (buf[n] == '\n') { n++; break; }
        if (buf[n] != '\r') n++;   /* strip CR from CRLF */
    }
    if (n == 0) return NULL;
    buf[n] = '\0';
    return buf;
}

static bool sd_mount(void)
{
    FRESULT res = f_mount(&fat_fs, "0:", 1);
    if (res != FR_OK) {
        char err[64];
        snprintf(err, sizeof(err),
                 "{\"status\":\"error\",\"msg\":\"SD mount failed: %d\"}", (int)res);
        uart_cmd_send(err);
        return false;
    }
    sd_mounted = true;
    return true;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void sd_batch_init(void)
{
    /* SDHC peripheral is initialised by the MSDK SDHC library when
     * LIB_SDHC=1 is set in project.mk.  We just mount here. */
    if (!sd_mounted)
        sd_mount();
}

void sd_batch_run(int max_samples)
{
    if (!sd_mounted && !sd_mount()) return;

    FIL      labels_file;
    FRESULT  res;
    char     line[128];

    res = f_open(&labels_file, SD_LABELS_PATH, FA_READ);
    if (res != FR_OK) {
        char err[80];
        snprintf(err, sizeof(err),
                 "{\"status\":\"error\",\"msg\":\"Cannot open %s: %d\"}",
                 SD_LABELS_PATH, (int)res);
        uart_cmd_send(err);
        return;
    }

    int n_total   = 0;
    int n_correct = 0;
    uint64_t lat_sum_us = 0;
    result_t results[INFERENCE_TOP_K_MAX];

    /* Skip header line */
    fatfs_readline(&labels_file, line, sizeof(line));

    while (fatfs_readline(&labels_file, line, sizeof(line)) != NULL) {
        if (max_samples > 0 && n_total >= max_samples) break;

        /* Parse "filename,class_idx" */
        char *comma = strchr(line, ',');
        if (!comma) continue;
        *comma = '\0';
        char   *filename  = line;
        int     truth_idx = atoi(comma + 1);

        /* Build full path */
        char path[64];
        snprintf(path, sizeof(path), "%s%s", SD_TENSOR_DIR, filename);

        /* Open and read tensor */
        FIL   tensor_file;
        UINT  br;
        res = f_open(&tensor_file, path, FA_READ);
        if (res != FR_OK) {
            char err[80];
            snprintf(err, sizeof(err),
                     "{\"status\":\"error\",\"msg\":\"Cannot open %s\"}", path);
            uart_cmd_send(err);
            continue;
        }
        f_read(&tensor_file, tensor_buf, TENSOR_SIZE, &br);
        f_close(&tensor_file);

        if ((int)br != TENSOR_SIZE) continue; /* skip malformed files */

        /* Run inference */
        uint32_t lat_us = inference_run(tensor_buf, results, 1);
        lat_sum_us += lat_us;
        n_total++;

        int pred_idx = results[0].class_idx;
        if (pred_idx == truth_idx) n_correct++;

        /* Emit per-sample JSON */
        int digs = (int)results[0].confidence;
        int tens = (int)((results[0].confidence - digs) * 10.0f + 0.5f);
        char out[256];
        snprintf(out, sizeof(out),
                 "{\"idx\":%d,\"file\":\"%s\",\"truth\":%d,\"pred\":%d,"
                 "\"conf\":%d.%d,\"lat_us\":%lu}",
                 n_total - 1, filename, truth_idx, pred_idx,
                 digs, tens, (unsigned long)lat_us);
        uart_cmd_send(out);
    }

    f_close(&labels_file);

    /* Summary */
    if (n_total > 0) {
        int acc_digs = (n_correct * 1000) / n_total;
        int acc_tens = acc_digs % 10;
        acc_digs = acc_digs / 10;
        uint32_t lat_mean = (uint32_t)(lat_sum_us / n_total);

        char summary[256];
        snprintf(summary, sizeof(summary),
                 "{\"batch_done\":true,\"n\":%d,\"correct\":%d,"
                 "\"accuracy\":%d.%d,\"lat_mean_us\":%lu}",
                 n_total, n_correct, acc_digs, acc_tens,
                 (unsigned long)lat_mean);
        uart_cmd_send(summary);
    } else {
        uart_cmd_send("{\"status\":\"error\",\"msg\":\"No samples evaluated\"}");
    }
}
