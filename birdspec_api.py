#!/usr/bin/env python3
"""
BirdSpec REST bridge for the MAX78002 firmware.

This script talks to two serial interfaces on Windows:
1. DUT UART: the MAX78002 application console / command port
2. PMON UART: the MAX78002EVKIT PMON USB virtual serial port

The DUT UART is used for inference and measurement control.
The PMON UART is used to capture the measured energy logs emitted by the PMON.
"""

import argparse
import io
import json
import threading
import time
from typing import Callable, Optional

import librosa
import numpy as np
import serial
import soundfile as sf
from flask import Flask, jsonify, request
from flask_cors import CORS


TARGET_SR = 16000
WINDOW_SAMPLES = 48000
WINDOW_BYTES = WINDOW_SAMPLES * 2
DUT_BAUD = 115200
DUT_TIMEOUT = 20.0
PMON_DEFAULT_BAUD = 115200

SPEC_N_FFT = 1024
SPEC_HOP = 256
SPEC_N_MELS = 128
SPEC_FMIN = 120.0
SPEC_FMAX = 8000.0
SPEC_DB_MIN = -80.0
SPEC_OUT_MELS = 64
SPEC_OUT_FRAMES = 128


app = Flask(__name__)
CORS(app)

dut_ser: Optional[serial.Serial] = None
pmon_ser: Optional[serial.Serial] = None
serial_lock = threading.Lock()


def dut_send(line: str) -> None:
    dut_ser.write((line + "\n").encode())


def dut_read_json(timeout: float = DUT_TIMEOUT,
                  accept: Optional[Callable[[dict], bool]] = None) -> dict:
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = dut_ser.readline()
        if not raw:
            continue
        text = raw.decode(errors="replace").strip()
        if not text:
            continue
        print(f"[dut] {text}", flush=True)
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            continue
        if accept is None or accept(obj):
            return obj
    raise TimeoutError(f"DUT did not respond within {timeout:.1f} s")


def dut_read_result(timeout: float = DUT_TIMEOUT) -> dict:
    def accept(obj: dict) -> bool:
        if obj.get("status") == "error":
            return True
        if obj.get("state") in ("silence", "spec_ready", "pcm_loaded"):
            return True
        return any(k.startswith("top") for k in obj)

    return dut_read_json(timeout=timeout, accept=accept)


def dut_require_uart_mode() -> dict:
    dut_ser.reset_input_buffer()
    dut_send("STATUS")
    return dut_read_json(
        timeout=3.0,
        accept=lambda obj: obj.get("status") == "ok",
    )


def dut_stream_bytes(data: bytes, chunk: int = 4096) -> None:
    for off in range(0, len(data), chunk):
        dut_ser.write(data[off: off + chunk])
    dut_ser.flush()


def dut_load_pcm_only(pcm_bytes: bytes) -> dict:
    if len(pcm_bytes) != WINDOW_BYTES:
        raise ValueError(f"PCM payload must be exactly {WINDOW_BYTES} bytes")

    dut_send(f"LOAD_PCM_ONLY {len(pcm_bytes)}")
    dut_read_json(
        timeout=3.0,
        accept=lambda obj: obj.get("state") == "receiving_pcm" or obj.get("status") == "error",
    )
    dut_stream_bytes(pcm_bytes)
    return dut_read_json(
        timeout=5.0,
        accept=lambda obj: obj.get("state") == "pcm_loaded" or obj.get("status") == "error",
    )


def dut_prep_spec() -> dict:
    dut_send("PREP_SPEC")
    return dut_read_json(
        timeout=10.0,
        accept=lambda obj: obj.get("state") == "spec_ready" or obj.get("status") == "error",
    )


def dut_load_spec_and_infer(spec_bytes: bytes) -> dict:
    dut_send(f"LOAD_SPEC {len(spec_bytes)}")
    dut_read_json(
        timeout=3.0,
        accept=lambda obj: obj.get("state") == "receiving_spec" or obj.get("status") == "error",
    )
    dut_stream_bytes(spec_bytes)
    return dut_read_result(timeout=DUT_TIMEOUT)


def pmon_available() -> bool:
    return pmon_ser is not None


def pmon_send_key(ch: str) -> None:
    if not pmon_available():
        raise RuntimeError("PMON serial port is not configured")
    pmon_ser.write(ch.encode())
    pmon_ser.flush()


def pmon_drain(seconds: float = 0.3) -> None:
    if not pmon_available():
        return

    deadline = time.time() + seconds
    while time.time() < deadline:
        raw = pmon_ser.readline()
        if not raw:
            continue
        text = raw.decode(errors="replace").strip()
        if text:
            print(f"[pmon-drop] {text}", flush=True)


def pmon_collect(seconds: float = 2.0) -> list[str]:
    if not pmon_available():
        raise RuntimeError("PMON serial port is not configured")

    lines: list[str] = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        raw = pmon_ser.readline()
        if not raw:
            continue
        text = raw.decode(errors="replace").strip()
        if text:
            print(f"[pmon-probe] {text}", flush=True)
            lines.append(text)
    return lines


def pmon_capture_window(trigger_fn: Callable[[], dict],
                        pre_key: Optional[str] = None,
                        quiet_s: float = 2.0,
                        max_s: float = 30.0) -> tuple[dict, list[str]]:
    if not pmon_available():
        raise RuntimeError("PMON serial port is required for measured energy capture")

    pmon_ser.reset_input_buffer()
    if pre_key:
        print(f"[bridge] PMON capture: sending pre-key '{pre_key}'", flush=True)
        try:
            pmon_send_key(pre_key)
            time.sleep(0.2)
            pmon_drain()
        except serial.SerialTimeoutException:
            print("[bridge] PMON capture: pre-key write timed out, continuing in passive read-only mode", flush=True)
    else:
        print("[bridge] PMON capture: passive read-only mode", flush=True)

    print("[bridge] PMON capture: triggering DUT command", flush=True)
    result = trigger_fn()
    print("[bridge] PMON capture: DUT command completed, collecting PMON output", flush=True)

    lines: list[str] = []
    deadline = time.time() + max_s
    last_rx = time.time()
    while time.time() < deadline:
        raw = pmon_ser.readline()
        if raw:
            text = raw.decode(errors="replace").strip()
            if text:
                print(f"[pmon] {text}", flush=True)
                lines.append(text)
                last_rx = time.time()
                continue
        if time.time() - last_rx >= quiet_s:
            break

    return result, lines


def decode_audio(audio_bytes: bytes) -> tuple[np.ndarray, int]:
    try:
        pcm, sr = sf.read(io.BytesIO(audio_bytes), dtype="float32", always_2d=False)
        if getattr(pcm, "ndim", 1) == 2:
            pcm = pcm.mean(axis=1)
        return pcm, sr
    except Exception:
        pcm, sr = librosa.load(io.BytesIO(audio_bytes), sr=None, mono=True)
        return pcm, sr


def resample_and_pad_to_window(audio_bytes: bytes) -> tuple[np.ndarray, bytes]:
    pcm_float, sr = decode_audio(audio_bytes)
    if sr != TARGET_SR:
        pcm_float = librosa.resample(pcm_float, orig_sr=sr, target_sr=TARGET_SR)

    if len(pcm_float) < WINDOW_SAMPLES:
        pcm_float = np.pad(pcm_float, (0, WINDOW_SAMPLES - len(pcm_float)))
    else:
        pcm_float = pcm_float[:WINDOW_SAMPLES]

    peak = float(np.max(np.abs(pcm_float))) if len(pcm_float) else 0.0
    if peak > 0:
        pcm_float = pcm_float / peak * 0.95

    pcm_i16 = np.clip(pcm_float * 32767.0, -32768, 32767).astype(np.int16)
    return pcm_float, pcm_i16.tobytes()


def compute_spectrogram(pcm_float: np.ndarray) -> bytes:
    peak = float(np.max(np.abs(pcm_float))) if len(pcm_float) else 0.0
    if peak > 0:
        pcm_float = pcm_float / peak * 0.95

    pcm16 = (pcm_float * 32767.0).astype(np.int16).astype(np.float32)
    stft = np.abs(
        librosa.stft(
            pcm16,
            n_fft=SPEC_N_FFT,
            hop_length=SPEC_HOP,
            window="hann",
            center=False,
        )
    ) ** 2
    mel_fb = librosa.filters.mel(
        sr=TARGET_SR,
        n_fft=SPEC_N_FFT,
        n_mels=SPEC_N_MELS,
        fmin=SPEC_FMIN,
        fmax=SPEC_FMAX,
    )
    mel_s = mel_fb @ stft
    ref_val = float(np.max(mel_s)) if np.max(mel_s) > 0 else 1.0
    mel_db = 10.0 * np.log10(np.maximum(mel_s, 1e-10) / ref_val)
    mel_db = np.clip(mel_db, SPEC_DB_MIN, 0.0)

    n_frames = mel_db.shape[1]
    mel_u8 = ((mel_db - SPEC_DB_MIN) / (-SPEC_DB_MIN) * 255.0).astype(np.float32)
    spec64 = np.zeros((SPEC_OUT_MELS, SPEC_OUT_FRAMES), dtype=np.float32)
    for col in range(SPEC_OUT_FRAMES):
        src = min((col * n_frames + n_frames // 2) // SPEC_OUT_FRAMES, n_frames - 1)
        for row in range(SPEC_OUT_MELS):
            spec64[row, col] = (mel_u8[row * 2, src] + mel_u8[row * 2 + 1, src]) * 0.5

    spec64 = spec64[::-1, :]
    spec_i8 = spec64 - 128.0
    mean = float(spec_i8.mean())
    std = max(float(spec_i8.std()), 1e-6)
    spec_z = np.clip((spec_i8 - mean) / std, -3.0, 3.0) / 3.0 * 127.0
    return np.clip(spec_z, -127, 127).astype(np.int8).tobytes()


@app.route("/api/status")
def api_status():
    try:
        with serial_lock:
            result = dut_require_uart_mode()
        return jsonify({"ok": True, "device": result})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/kat")
def api_kat():
    try:
        with serial_lock:
            dut_require_uart_mode()
            dut_send("KAT")
            result = dut_read_json(timeout=10.0, accept=lambda obj: "kat" in obj or obj.get("status") == "error")
        return jsonify({"ok": result.get("kat") == "pass", "result": result})
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/record", methods=["POST"])
def api_record():
    top = min(max(int(request.args.get("top", 3)), 1), 5)
    metrics = request.args.get("metrics", "0") not in ("0", "false", "no", "")

    try:
        with serial_lock:
            dut_require_uart_mode()
            dut_send("REC")
            result = dut_read_result(timeout=DUT_TIMEOUT)
    except TimeoutError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 504
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500

    if result.get("status") == "error":
        return jsonify({"ok": False, "error": result.get("msg", "device error")}), 500

    if result.get("state") == "silence":
        return jsonify({"ok": True, "silence": True, "predictions": []})

    key = next((k for k in result if k.startswith("top")), None)
    out = {"ok": True, "silence": False, "predictions": result.get(key, [])[:top] if key else []}
    if metrics:
        out["cnn_latency_us"] = result.get("cnn_latency_us", result.get("latency_us"))
        out["spec_us"] = result.get("spec_us")
        out["total_us"] = result.get("total_us")
        out["cnn_nj"] = result.get("cnn_nj")
        out["spec_nj"] = result.get("spec_nj")
        out["total_nj"] = result.get("total_nj")
        out["energy_kind"] = result.get("energy_kind")
    return jsonify(out)


@app.route("/api/infer", methods=["POST"])
def api_infer():
    if "audio" not in request.files:
        return jsonify({"ok": False, "error": "No 'audio' field in request"}), 400

    top = min(max(int(request.args.get("top", 3)), 1), 5)
    metrics = request.args.get("metrics", "0") not in ("0", "false", "no", "")
    audio_bytes = request.files["audio"].read()

    try:
        pcm_float, sr = decode_audio(audio_bytes)
        if sr != TARGET_SR:
            pcm_float = librosa.resample(pcm_float, orig_sr=sr, target_sr=TARGET_SR)
        if len(pcm_float) < WINDOW_SAMPLES:
            pcm_float = np.pad(pcm_float, (0, WINDOW_SAMPLES - len(pcm_float)))
        else:
            pcm_float = pcm_float[:WINDOW_SAMPLES]

        t0 = time.perf_counter()
        spec_bytes = compute_spectrogram(pcm_float)
        host_spec_us = int((time.perf_counter() - t0) * 1e6)

        with serial_lock:
            dut_require_uart_mode()
            result = dut_load_spec_and_infer(spec_bytes)
    except TimeoutError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 504
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500

    if result.get("status") == "error":
        return jsonify({"ok": False, "error": result.get("msg", "device error")}), 500

    key = next((k for k in result if k.startswith("top")), None)
    predictions = result.get(key, [])[:top] if key else []
    out = {"ok": True, "predictions": predictions}
    if metrics:
        out["cnn_latency_us"] = result.get("cnn_latency_us", result.get("latency_us"))
        out["device_spec_us"] = result.get("spec_us")
        out["host_spec_us"] = host_spec_us
        out["total_us"] = result.get("total_us")
        out["cnn_nj"] = result.get("cnn_nj")
        out["spec_nj"] = result.get("spec_nj")
        out["total_nj"] = result.get("total_nj")
        out["energy_kind"] = result.get("energy_kind")
    return jsonify(out)


@app.route("/api/pmon/info")
def api_pmon_info():
    try:
        with serial_lock:
            dut_require_uart_mode()
            dut_send("PMON_INFO")
            dut_info = dut_read_json(timeout=5.0, accept=lambda obj: obj.get("state") == "pmon_info" or obj.get("status") == "error")
        return jsonify({
            "ok": True,
            "dut": dut_info,
            "pmon_port_configured": pmon_available(),
        })
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/pmon/probe")
def api_pmon_probe():
    seconds = max(1.0, min(float(request.args.get("seconds", 2.0)), 10.0))

    try:
        with serial_lock:
            if not pmon_available():
                raise RuntimeError("PMON serial port is not configured")
            pmon_ser.reset_input_buffer()
            lines = pmon_collect(seconds=seconds)
        return jsonify({
            "ok": True,
            "seconds": seconds,
            "line_count": len(lines),
            "pmon_lines": lines,
        })
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/pmon/cnn", methods=["POST"])
def api_pmon_cnn():
    audio_bytes = request.files["audio"].read() if "audio" in request.files else None

    try:
        with serial_lock:
            dut_require_uart_mode()
            if audio_bytes is not None:
                _, pcm_bytes = resample_and_pad_to_window(audio_bytes)
                load_res = dut_load_pcm_only(pcm_bytes)
                if load_res.get("status") == "error":
                    raise RuntimeError(load_res.get("msg", load_res))
                spec_res = dut_prep_spec()
                if spec_res.get("status") == "error":
                    raise RuntimeError(spec_res.get("msg", spec_res))
            else:
                spec_res = None

            dut_result, pmon_lines = pmon_capture_window(
                trigger_fn=lambda: _dut_run_pmon_command("PMON_CNN", "pmon_cnn_complete"),
                pre_key=None,
            )
        return jsonify({
            "ok": True,
            "mode": "cnn_power",
            "dut": dut_result,
            "prep_spec": spec_res,
            "pmon_lines": pmon_lines,
            "pmon_captured": True,
        })
    except TimeoutError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 504
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


@app.route("/api/pmon/full", methods=["POST"])
def api_pmon_full():
    if "audio" not in request.files:
        return jsonify({"ok": False, "error": "No 'audio' field in request"}), 400

    repeats = max(1, min(int(request.args.get("repeats", 10)), 50))
    audio_bytes = request.files["audio"].read()

    try:
        _, pcm_bytes = resample_and_pad_to_window(audio_bytes)
        with serial_lock:
            dut_require_uart_mode()
            load_res = dut_load_pcm_only(pcm_bytes)
            if load_res.get("status") == "error":
                raise RuntimeError(load_res.get("msg", load_res))

            dut_result, pmon_lines = pmon_capture_window(
                trigger_fn=lambda: _dut_run_pmon_command(f"PMON_FULL {repeats}", "pmon_full_complete"),
                pre_key=None,
            )
        return jsonify({
            "ok": True,
            "mode": "system_power",
            "repeats": repeats,
            "dut": dut_result,
            "pmon_lines": pmon_lines,
            "pmon_captured": True,
        })
    except TimeoutError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 504
    except Exception as exc:
        return jsonify({"ok": False, "error": str(exc)}), 500


def _dut_run_pmon_command(command: str, done_state: str) -> dict:
    dut_send(command)
    dut_read_json(timeout=5.0, accept=lambda obj: obj.get("status") == "error" or obj.get("state", "").endswith("_begin"))
    return dut_read_json(
        timeout=DUT_TIMEOUT,
        accept=lambda obj: obj.get("status") == "error" or obj.get("state") == done_state,
    )


def open_serial(port: str, baud: int, timeout: float,
                dtr: Optional[bool] = None,
                rts: Optional[bool] = None) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 1.0
    if dtr is not None:
        ser.dtr = dtr
    if rts is not None:
        ser.rts = rts
    ser.open()
    time.sleep(0.5)
    ser.reset_input_buffer()
    return ser


def main() -> None:
    parser = argparse.ArgumentParser(description="BirdSpec API and PMON bridge")
    parser.add_argument("--dut-port", required=True, help="DUT UART port, e.g. COM18")
    parser.add_argument("--pmon-port", help="PMON USB virtual serial port, e.g. COM19")
    parser.add_argument("--dut-baud", type=int, default=DUT_BAUD)
    parser.add_argument("--pmon-baud", type=int, default=PMON_DEFAULT_BAUD)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--flask-port", type=int, default=5000)
    args = parser.parse_args()

    global dut_ser, pmon_ser
    dut_ser = open_serial(args.dut_port, args.dut_baud, timeout=1.0, dtr=False, rts=False)
    if args.pmon_port:
        pmon_ser = open_serial(args.pmon_port, args.pmon_baud, timeout=0.5)

    print(f"[birdspec_api] DUT UART:  {args.dut_port} @ {args.dut_baud}")
    print(f"[birdspec_api] PMON UART: {args.pmon_port or 'disabled'} @ {args.pmon_baud}")
    print(f"[birdspec_api] HTTP:      http://{args.host}:{args.flask_port}")
    print("[birdspec_api] Press SW4 on the EVKIT to enter UART mode before calling the API.")
    print("[birdspec_api] Existing inference endpoints remain available.")
    print("[birdspec_api] PMON endpoints require the PMON USB serial port and PMON triggered mode support.")

    app.run(host=args.host, port=args.flask_port, debug=False)


if __name__ == "__main__":
    main()
