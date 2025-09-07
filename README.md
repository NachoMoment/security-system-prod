# ESP32‑S3 Person Detection + SD Recorder

Small, single‑purpose app that runs **person/no‑person** inference on an ESP32‑S3 camera board and **records JPEGs to microSD** when the model says a person is present. Recording is triggered by **ESP‑NOW motion packets** from a PIR sensor node to save power and reduce false captures. Designed for night use with an external IR illuminator.

> Camera runs in one owner at a time: **inference OR recorder**. While the recorder is active, inference pauses so the camera isn’t double‑used.

---

## Features

- **ESP‑NOW trigger → inference → gated recording** (hysteresis to avoid flapping)
- **SD card (1‑bit SDMMC)**: mounts `/sdcard` and writes to `/sdcard/REC/*.JPG`
- **TFLite Micro** person detection model (int8)
- **Graceful camera ownership**: inference yields to recorder
- **Configurable thresholds** and channels (see `Kconfig.projbuild`)
- **S3‑friendly defaults**: XCLK 20 MHz, PSRAM arena, simple logging

---

## Hardware

- **MCU:** ESP32‑S3‑CAM‑class board (tested on S3 camera modules)
- **microSD:** 1‑bit SDMMC
  - Pins used by this build: **CLK=GPIO39, CMD=GPIO38, D0=GPIO40**
- **Trigger source (optional but intended):** ESP‑NOW PIR sender on channel 1
- **IR lighting:** external IR illuminator for night scenes (not controlled here)

---

## Repo layout (key parts)

```
main/
  app_camera_esp.c        # camera init + acquire helpers (detect/record profiles)
  app_now_rx.c            # ESP‑NOW receiver; calls on_external_trigger()
  detection_responder.cc  # converts scores → start/stop decisions via gate
  image_provider.cc       # grabs frame, crops to 96×96, int8 grayscale
  main.cc                 # boot flow, TF task, stop timer, ownership rules
  main_functions.cc       # TFLM setup + loop()
  model_settings.*        # tensor dims + labels
  person_detect_model_data.cc  # embedded model
  record_gate.c           # hysteresis gate
  recorder.c              # SD mount + JPEG writer (/sdcard/REC)
  psram_probe.c           # PSRAM sanity logging
  Kconfig.projbuild       # menuconfig knobs (thresholds, channels, etc.)
```

> Removed from public/lean build for now (easy to restore later): `esp_cli.*`, `session_manager.*`, `motion_espnow.*`.

---

## Quick start

```bash
# ESP‑IDF v5.3 recommended
idf.py set-target esp32s3

# Use your defaults explicitly (adjust file name if needed)
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults.esp32s3" reconfigure

idf.py build
idf.py flash monitor
```

**Notes**
- Ensure microSD is formatted FAT/FAT32; the app creates `/sdcard/REC` if missing.
- If asked about long file names buffer: using **heap** is safest for large paths.
- If you see CMake re‑running repeatedly: check `main/CMakeLists.txt` is syntactically correct and that `build/` is nuked (`idf.py fullclean`).

---

## Configure

Most useful tweaks live in **`menuconfig → Project Configuration`** (from `Kconfig.projbuild`):

- `PERSON_TRIG_PCT` — start threshold (default 55%)
- `PERSON_STOP_PCT` — stop threshold (default 40%)
- `PERSON_START_CONSEC_FRAMES` / `PERSON_STOP_CONSEC_FRAMES`
- `ESPNOW_CHANNEL` — channel for the PIR sender (default 1)
- `XCLK_FREQ_HZ` — camera clock (default 20 MHz)

You can also define these via `-D` or in source if you prefer.

---

## How it works

1. **Boot**: camera initializes in **DETECT** profile (fast grayscale capture).
2. **Listen**: ESP‑NOW receiver arms the system on the first motion packet.
3. **Infer**: TensorFlow Lite Micro runs `loop()`; `image_provider` feeds 96×96 int8 frames.
4. **Decide**: `detection_responder` updates a **hysteresis gate** with the latest scores.
5. **Record**: when gate allows and not already recording, `recorder` mounts SD (if needed) and writes JPEGs to `/sdcard/REC`.
6. **Yield**: while recording, inference pauses so the camera has a single owner.
7. **Cooldown**: if no further triggers arrive, a **stop timer** shuts down inference after a short idle window (camera stays up).

---

## Lessons learned

- **CMake loops** are usually a malformed `main/CMakeLists.txt` or stale `build/` directory. Keep the SRCS list explicit and clean.
- **Camera exclusivity** matters. Let **only one** module own `esp_camera` at a time; pausing TF during recording is the simplest approach.
- **Hysteresis beats single thresholds.** Start/stop bands prevent “recorder flapping” on borderline frames.
- **SD gotchas**: create the mount point and ensure the **REC** directory exists; verify 1‑bit SDMMC pins; handle `errno=22` (bad path/args) by logging full paths.
- **PSRAM budgeting**: put the TFLM tensor arena in PSRAM and watch heap fragmentation; keep logs informative but not spammy.
- **Long file names**: prefer heap buffering for LFN to avoid blowing the task stack.
- **Ignore artifacts**: don’t commit `build/`, `managed_components/`, `sdkconfig`, etc. (see `.gitignore`).

---

## Immediate TODO

- **Stop timer re‑arm behavior:** on every motion trigger, **stop + start_once** the timer so it truly extends the idle window. Audit that we never create multiple timers and that we always stop it before re‑arming.
- **Gate constants → Kconfig:** expose `PERSON_*` thresholds and consecutive‑frame counts so users can tune without editing code.
- **SD path safety:** ensure `/sdcard/REC` exists at mount and **fsync** or close properly on stop to avoid partial images.
- **Recording ownership checks:** double‑check that no frame capture occurs in the TF loop while `recorder_is_recording()` is true.
- **Log levels for release:** default global to `WARN`, bump module‑specific logs (`detect`, `recorder`, `app_camera`) when debugging.

---

## Near‑future TODO

- **Timestamped filenames** (RTC/NTP) instead of sequential names.
- **Model updates**: retrain or swap in a night‑biased model; expose a switch for future models.
- **Board profiles**: add `CAMERA_MODEL_AI_THINKER` path and a pin map switch.
- **ESP‑NOW pairing/handshake** for more robust multi‑sensor setups.
- **IR control output**: drive a relay/MOSFET for the illuminator when gate opens.
- **CI**: GitLab CI build job (idf docker image) + artifact of the `.bin` on tags.
- **OTA**: simple HTTPS OTA so field updates don’t require a cable.
- **Telemetry (optional)**: ring buffer of recent scores + events to SD for offline debug.

---

## Troubleshooting

- **Endless “Configuring done” loop**: delete `build/`; validate `main/CMakeLists.txt` syntax and that `idf_component_register()` closes with `)`.
- **`fopen` errno=22**: path or mode is invalid. Log the exact path; ensure `/sdcard/REC` exists; avoid writing while unmounted.
- **“Invalid head of packet” flashing**: confirm the correct port/USB cable; use `idf.py -p <PORT> flash`.
- **FB‑OVF (frame buffer overflow)**: lower XCLK, reduce frame size, or free memory before capture.

---

## License & Credits

- **License:** Apache‑2.0 (inherits from the TensorFlow Micro example base).
- **Third‑party**: Espressif `esp32-camera`, Google TFLite Micro.

If you use this as a base, please keep the NOTICE for upstream projects.