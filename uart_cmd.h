#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

/*
 * UART command protocol (text lines, newline-terminated).
 *
 * Host → Device:
 *   REC\n               — trigger 3-second recording + inference (mic mode)
 *   INFER\n             — run inference on the last loaded PCM buffer
 *   GET_TOPK <k>\n      — return top-k results from the last inference as JSON
 *   STATUS\n            — return current device state as JSON
 *   LOAD_PCM <bytes>\n  — followed immediately by <bytes> raw int16-LE PCM
 *                         (48000 samples = 96000 bytes for a 3 s clip at 16 kHz)
 *   BATCH\n             — mount SD card and run batch evaluation
 *   KAT\n               — run known-answer test
 *
 * Device → Host (all responses are JSON on a single line + '\n'):
 *   {"status":"ok","state":"idle"}
 *   {"status":"ok","top3":[{"idx":47,"label":"turdus_merula","common":"Common Blackbird","conf":87.3},...], "latency_us":41823}
 *   {"status":"error","msg":"..."}
 */

/* Baud rates --------------------------------------------------------- */
#define UART_CMD_BAUD_NORMAL  115200U
#define UART_CMD_BAUD_PCM     921600U   /* used during LOAD_PCM transfer */

/*
 * Call once at boot.  Initialises the UART peripheral.
 */
void uart_cmd_init(void);

/*
 * Non-blocking poll.  Call from the main loop.  Reads available bytes,
 * builds a command line, and dispatches when '\n' is received.
 */
void uart_cmd_poll(void);

/*
 * Send a JSON string followed by '\n' over UART.
 * msg must be a null-terminated C string.
 */
void uart_cmd_send(const char *msg);

#endif /* UART_CMD_H */
