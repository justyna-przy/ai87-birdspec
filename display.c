#include <stdio.h>
#include <string.h>
#include "mxc.h"
#include "display.h"
#include "species_labels.h"

/* ------------------------------------------------------------------ */
/* Font                                                                 */
/* ------------------------------------------------------------------ */

/* LiberationSans16x16 is compiled in because project.mk has:
 *   FONTS = LiberationSans16x16
 * The font array name matches the file name. */
extern const unsigned char Liberation_Sans16x16[];

/* Cast to int as expected by MXC_TFT_PrintFont() */
#define FONT16  ((int)&Liberation_Sans16x16[0])

/* ------------------------------------------------------------------ */
/* Layout constants                                                     */
/* ------------------------------------------------------------------ */

/* Display is 320×240 in landscape (ROTATE_270 applied at init) */
#define TFT_W   320
#define TFT_H   240
#define LINE_H  20      /* ~16px font + 4px gap                       */

#define HEADER_Y        2
#define SPEC_Y          (HEADER_Y + LINE_H + 4)
#define SPEC_DRAW_W     256   /* 128 cols × 2                         */
#define SPEC_DRAW_H     128   /* 64 rows × 2                          */
#define SPEC_X          ((TFT_W - SPEC_DRAW_W) / 2)
#define RESULTS_Y       (SPEC_Y + SPEC_DRAW_H + 4)
#define RESULT_LINE_H   LINE_H
#define FOOTER_Y        (RESULTS_Y + 3 * RESULT_LINE_H + 4)

/* ------------------------------------------------------------------ */
/* Internal helper: draw one line of text                               */
/* ------------------------------------------------------------------ */

static void tft_print(int x, int y, const char *str)
{
    text_t t;
    t.data = (char *)str;
    t.len  = (int)strlen(str);
    MXC_TFT_PrintFont(x, y, FONT16, &t, NULL);
}

/* Clear a horizontal band and then print a string into it */
static void tft_clear_print(int x, int y, int w, int h, const char *str)
{
    area_t a = { (uint16_t)x, (uint16_t)y, (uint16_t)w, (uint16_t)h };
    MXC_TFT_FillRect(&a, BLACK);
    tft_print(x, y, str);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

/* Map int8 to grayscale RGB565 */
static inline uint16_t gray_rgb565(int8_t v)
{
    uint8_t g = (uint8_t)((int)v + 128);
    return (uint16_t)(((g >> 3) << 11) | ((g >> 2) << 5) | (g >> 3));
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void display_init(void)
{
    MXC_TFT_Init(NULL, NULL);

    /* The NewHaven TFT on the MAX78002 EV kit is physically mounted in
     * portrait orientation.  ROTATE_270 maps it to 320×240 landscape. */
    MXC_TFT_SetRotation(ROTATE_270);

    MXC_TFT_SetBackGroundColor(BLACK);
    MXC_TFT_SetForeGroundColor(WHITE);
    MXC_TFT_ClearScreen();

    /* Static header */
    tft_print(2, HEADER_Y, "BirdSpec - Irish Bird Classifier");

    /* Separator line */
    MXC_TFT_Line(0, HEADER_Y + LINE_H, TFT_W - 1, HEADER_Y + LINE_H, WHITE);
}

void display_status(const char *msg)
{
    /* Clear the footer band and write the new message */
    tft_clear_print(2, FOOTER_Y, TFT_W - 4, LINE_H, msg);
}

void display_spectrogram(const int8_t *tensor)
{
    area_t area = { SPEC_X, SPEC_Y, SPEC_DRAW_W, SPEC_DRAW_H };
    MXC_TFT_FillRect(&area, BLACK);

    uint16_t row_buf[256];

    for (int row = 0; row < 64; row++) {
        for (int col = 0; col < 128; col++) {
            uint16_t c = gray_rgb565(tensor[row * 128 + col]);
            row_buf[col * 2]     = c;
            row_buf[col * 2 + 1] = c;
        }
        int y = SPEC_Y + row * 2;
        MXC_TFT_WriteBufferRGB565(SPEC_X, y,     (uint8_t *)row_buf, 256, 1);
        MXC_TFT_WriteBufferRGB565(SPEC_X, y + 1, (uint8_t *)row_buf, 256, 1);
    }
}

void display_results(const result_t *results, int k, uint32_t latency_us)
{
    /* Clear results area */
    area_t area = { 0, RESULTS_Y, TFT_W, FOOTER_Y - RESULTS_Y };
    MXC_TFT_FillRect(&area, BLACK);

    char line[80];

    for (int i = 0; i < k && i < 3; i++) {
        int idx  = results[i].class_idx;
        int digs = (int)results[i].confidence;
        int tens = (int)((results[i].confidence - digs) * 10.0f + 0.5f);
        snprintf(line, sizeof(line), "%d. %-22s %2d.%d%%",
                 i + 1, species_common_name[idx], digs, tens);
        tft_print(2, RESULTS_Y + i * RESULT_LINE_H, line);
    }

    snprintf(line, sizeof(line), "Latency: %lu ms",
             (unsigned long)(latency_us / 1000));
    tft_clear_print(2, FOOTER_Y, TFT_W - 4, LINE_H, line);
}
