# PMON Measurement Notes

This project now supports real PMON trigger windows on the MAX78002EVKIT.

## Current Status

- Firmware git hash: `04a57d4ced1f8029385b53ddb6e97b8353ad3f15`
- Deployed CNN generation command:

```text
ai8xize.py --verbose --overwrite --test-dir demos --prefix ai87-birdspec-dw1 --checkpoint-file trained/birdspec_dw1_q8.pth.tar --config-file networks/birdspec-dw1-ai87.yaml --sample-input tests/sample_birdspec.npy --device MAX78002 --compact-data --softmax
```

- Current generated DW1 CNN was **not** produced with `--energy`.
- Existing serial `*_nj` values remain **duration-based energy estimates**.
- Real measured energy is only available from PMON logs/screenshots.

## Trigger Wiring

- `SYS_START` / `SYS_COMPLETE` drive `TRIG1` on `P1.6`
- `CNN_START` / `CNN_COMPLETE` drive `TRIG2` on `P1.7`
- Install `JP18` for `TRIG1`
- Install `JP19` for `TRIG2`

Without `JP18` and `JP19`, PMON-triggered measurements are not valid.

## Build

Use the exact command you run and save it in the paper notes.

Typical command:

```text
make -j8
```

## UART Commands

Enter UART mode with `SW4`, then use:

- `PMON_INFO`
- `LOAD_PCM 96000` followed by the raw 3-second PCM clip
- `PMON_CNN`
- `PMON_FULL 10`

`PMON_CNN`:

- Intended for PMON `CNN Power Mode`
- Emits the official-style sequence:
  - 1 second idle window on `SYS_START..SYS_COMPLETE`
  - kernel-load window on `CNN_START..CNN_COMPLETE`
  - input-load window on `CNN_START..CNN_COMPLETE`
  - input+inference window on `CNN_START..CNN_COMPLETE`
- Use the PMON-reported `input+inference` energy and note whether you subtract the `input-load` stage to isolate inference-only energy.

`PMON_FULL 10`:

- Intended for PMON `System Power Mode`
- Measures the instrumented `spectrogram -> CNN -> top-k` compute window on the currently loaded PCM buffer
- Excludes UART transfer time because the PCM is loaded before the PMON window starts
- Divide the PMON total energy by the repeat count for per-clip energy

## What To Save

- Firmware git hash or folder timestamp
- Exact build command
- Exact ai8xize command if you regenerate
- PMON mode used: `CNN Power Mode` or `System Power Mode`
- PMON firmware version if shown
- Jumper configuration, especially `JP18` and `JP19`
- Serial log from `PMON_INFO`
- Serial logs from `PMON_CNN` / `PMON_FULL`
- PMON display photo or PMON serial log for each measurement
- Repeat count
- Whether the PMON number is per inference or total window
- Whether input loading was included or subtracted
- Whether spectrogram computation was included

## Paper Wording

If PMON logs are captured:

- `CNN inference energy was measured using the MAX78002EVKIT PMON in windowed CNN power mode.`
- `Full spectrogram-plus-CNN compute energy was measured using MAX78002EVKIT PMON system power mode over the instrumented processing window.`

If PMON is not captured:

- Keep the paper wording as `duration-based energy estimate`.
