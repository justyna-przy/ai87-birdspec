#include <stdio.h>
#include <string.h>
/* NOTE: do NOT include tft_ili9341.h — board.h (via mxc.h) already pulls in
 * tft_st7789v.h for the MAX78002 EV kit.  Including the wrong header causes
 * duplicate-type errors for area_t, text_t, and all MXC_TFT_* prototypes. */
#include "mxc.h"
#include "board.h"
#include "fonts.h"
#include "display.h"
#include "species_labels.h"

/* Font provided by FONTS=LiberationSans16x16 in project.mk */
extern const unsigned char Liberation_Sans16x16[];

/* ------------------------------------------------------------------ */
/* Layout constants                                                     */
/* ------------------------------------------------------------------ */
#define TFT_W           320
#define TFT_H           240

#define HEADER_Y        0
#define HEADER_H        20
#define SPEC_Y          (HEADER_Y + HEADER_H + 2)
#define SPEC_DRAW_W     256   /* 128 cols × 2 */
#define SPEC_DRAW_H     128   /* 64 rows  × 2 */
#define SPEC_X          ((TFT_W - SPEC_DRAW_W) / 2)
#define RESULTS_Y       (SPEC_Y + SPEC_DRAW_H + 4)
#define RESULT_LINE_H   20
#define FOOTER_Y        (RESULTS_Y + 3 * RESULT_LINE_H + 4)

/* Foreground / background colours (RGB565) */
#define COLOR_BG        0x0000  /* black */
#define COLOR_FG        0xFFFF  /* white */

/* ------------------------------------------------------------------ */
/* Display-name overrides (display only — does not affect JSON output) */
/* NULL = use species_common_name as-is                                */
/* ------------------------------------------------------------------ */
static const char *display_name[51] = {
    "Sparrowhawk",       /*  0 Eurasian Sparrowhawk        (20) */
    NULL,                /*  1 Sedge Warbler */
    NULL,                /*  2 Long-tailed Tit */
    NULL,                /*  3 Common Kingfisher */
    NULL,                /*  4 Meadow Pipit */
    NULL,                /*  5 Common Swift */
    NULL,                /*  6 Common Buzzard */
    NULL,                /*  7 European Goldfinch */
    NULL,                /*  8 European Greenfinch */
    "Dipper",            /*  9 White-throated Dipper       (21) */
    NULL,                /* 10 Western Jackdaw */
    NULL,                /* 11 Rock Pigeon */
    NULL,                /* 12 Wood Pigeon */
    NULL,                /* 13 Common Raven */
    NULL,                /* 14 Hooded Crow */
    NULL,                /* 15 Rook */
    NULL,                /* 16 Common Cuckoo */
    NULL,                /* 17 Blue Tit */
    "House Martin",      /* 18 Northern House Martin       (21) */
    "GS Woodpecker",     /* 19 Great Spotted Woodpecker    (24) */
    NULL,                /* 20 Yellowhammer */
    NULL,                /* 21 Reed Bunting */
    NULL,                /* 22 European Robin */
    NULL,                /* 23 Peregrine Falcon */
    NULL,                /* 24 Common Kestrel */
    NULL,                /* 25 Common Chaffinch */
    NULL,                /* 26 Eurasian Jay */
    NULL,                /* 27 Barn Swallow */
    NULL,                /* 28 White Wagtail */
    NULL,                /* 29 Grey Wagtail */
    NULL,                /* 30 Spotted Flycatcher */
    "Non-bird",          /* 31 Non-bird / Background       (21) */
    NULL,                /* 32 Northern Wheatear */
    NULL,                /* 33 Great Tit */
    NULL,                /* 34 House Sparrow */
    NULL,                /* 35 Coal Tit */
    NULL,                /* 36 Common Pheasant */
    NULL,                /* 37 Chiffchaff */
    NULL,                /* 38 Willow Warbler */
    NULL,                /* 39 Eurasian Magpie */
    NULL,                /* 40 Dunnock */
    NULL,                /* 41 Goldcrest */
    NULL,                /* 42 European Stonechat */
    NULL,                /* 43 European Siskin */
    "Collared Dove",     /* 44 Eurasian Collared Dove      (22) */
    NULL,                /* 45 Common Starling */
    NULL,                /* 46 Eurasian Blackcap */
    NULL,                /* 47 Eurasian Wren */
    NULL,                /* 48 Common Blackbird */
    NULL,                /* 49 Song Thrush */
    NULL,                /* 50 Barn Owl */
};

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

/* Map int8 → grayscale RGB565 */
static inline uint16_t gray_rgb565(int8_t v)
{
    uint8_t g = (uint8_t)((int)v + 128); /* [-128..127] → [0..255] */
    return (uint16_t)(((g >> 3) << 11) | ((g >> 2) << 5) | (g >> 3));
}

/*
 * Print a string at pixel position (x, y) using the LiberationSans16x16 font.
 * MXC_TFT_ConfigPrintf / MXC_TFT_Printf are documented empty stubs in this
 * MSDK version; MXC_TFT_PrintFont is the real implementation.
 */
static void tft_print_at(int x, int y, const char *msg)
{
    text_t t = { (char *)msg, (int)strlen(msg) };
    MXC_TFT_PrintFont(x, y, (int)&Liberation_Sans16x16[0], &t, NULL);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void display_init(void)
{
    MXC_TFT_Init(NULL, NULL);
    MXC_TFT_SetBackGroundColor(COLOR_BG);
    MXC_TFT_ClearScreen();

    tft_print_at(2, HEADER_Y + 2, "MicroBird");
}

void display_status(const char *msg)
{
    area_t area = { 0, (short)FOOTER_Y, (short)TFT_W, (short)(TFT_H - FOOTER_Y) };
    MXC_TFT_FillRect(&area, COLOR_BG);
    tft_print_at(2, FOOTER_Y + 2, msg);
}

void display_spectrogram(const int8_t *tensor)
{
    /* Clear spectrogram panel */
    area_t area = { (short)SPEC_X, (short)SPEC_Y, (short)SPEC_DRAW_W, (short)SPEC_DRAW_H };
    MXC_TFT_FillRect(&area, COLOR_BG);

    /*
     * Build one 256-pixel wide RGB565 row at a time (512 bytes on stack),
     * then blit it twice (2× height scale) using MXC_TFT_WriteBufferRGB565.
     * This avoids 32768 individual WritePixel calls.
     */
    uint16_t row_buf[256];

    for (int row = 0; row < 64; row++) {
        /* Fill row: each source pixel is doubled horizontally.
         * MXC_TFT_WriteBufferRGB565 sends bytes in memory order (LSB first),
         * but ST7789V expects MSB first — byte-swap each pixel to match
         * what fill_rect does: store (c<<8)|(c>>8) so wire order is correct. */
        for (int col = 0; col < 128; col++) {
            uint16_t c = gray_rgb565(tensor[row * 128 + col]);
            c = (uint16_t)((c << 8) | (c >> 8)); /* byte-swap for ST7789V */
            row_buf[col * 2]     = c;
            row_buf[col * 2 + 1] = c;
        }
        int y = SPEC_Y + row * 2;
        /* Blit twice for 2× vertical scale */
        MXC_TFT_WriteBufferRGB565(SPEC_X, y,     (uint8_t *)row_buf, 256, 1);
        MXC_TFT_WriteBufferRGB565(SPEC_X, y + 1, (uint8_t *)row_buf, 256, 1);
    }
}

void display_results(const result_t *results, int k, uint32_t latency_us)
{
    /* Clear results + footer area */
    area_t area = { 0, (short)RESULTS_Y, (short)TFT_W, (short)(TFT_H - RESULTS_Y) };
    MXC_TFT_FillRect(&area, COLOR_BG);

    char line[80];

    for (int i = 0; i < k && i < 3; i++) {
        int idx  = results[i].class_idx;
        int digs = (int)results[i].confidence;
        const char *name = (idx < 51 && display_name[idx]) ? display_name[idx]
                                                            : species_common_name[idx];
        snprintf(line, sizeof(line), "%d. %s (%d%%)", i + 1, name, digs);
        tft_print_at(2, RESULTS_Y + i * RESULT_LINE_H + 2, line);
    }

    snprintf(line, sizeof(line), "Latency: %lu ms",
             (unsigned long)(latency_us / 1000));
    tft_print_at(2, FOOTER_Y + 2, line);
}
