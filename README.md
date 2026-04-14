# Lote-stm32
learn on the edge - stm32g431cbu6 open source hil

## Setup repository 
Install following vscode extensions: clangd, pylance, c/c++

```
sudo apt install gcc-arm-none-eabi cmake ninja-build dfu-util clangd git
```

## Build

configure
```
cmake -B build-debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=tools/arm-none-eabi-toolchain.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build-debug
```

## Tools
Downloaded from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads

## Put board in dfu mode
Hold BOOT
Press RESET
Release RESET
Release BOOT

```
dfu-util -l
```

## Flash
```
./tools/flash.sh
```

or manually:
```
dfu-util -a 0 -s 0x08000000:leave -D build-debug/firmware.bin
```

---

## HIL (Hardware-in-the-Loop) testing

### Setup Python environment

Create and activate a virtual environment, then install the required packages:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r tools/requirements.txt
```

### Connect the board

Plug the STM32 board via USB. The `start_hil.sh` script auto-detects the STM32 CDC serial port by its USB vendor ID (0x0483) — no manual port configuration needed.

### Run HIL

Use the provided launcher script from the repo root. It sources the virtual environment and sets up the Python path automatically:

```bash
./tools/start_hil.sh --data-root <path/to/kitti/drive_folder> [OPTIONS]
```

The `--data-root` must point to a KITTI drive folder that contains `image_00/data/` and `image_01/data/` subdirectories, e.g.:

```bash
./tools/start_hil.sh \
  --data-root ../datasets/kitti/2011_09_26_drive_0001_sync \
  --write-freq 6
```

### Options

| Flag | Type | Default | Description |
|---|---|---|---|
| `--data-root PATH` | string | *(required)* | Path to the KITTI drive folder |
| `--write-freq HZ` | float | `30` | Frame send rate to the STM32 (Hz) |
| `--playback DELAY_MS` | int | — | After the run, replay frames with a fixed delay (ms) between each |
| `--playback-realtime` | flag | — | After the run, replay frames using the original inter-frame timings |

### Examples

Run at 6 Hz and replay with original timings:
```bash
./tools/start_hil.sh \
  --data-root ../datasets/kitti/2011_09_26_drive_0001_sync \
  --write-freq 6 \
  --playback-realtime
```

Run at maximum throughput with a 100 ms fixed playback delay:
```bash
./tools/start_hil.sh \
  --data-root ../datasets/kitti/2011_09_26_drive_0001_sync \
  --playback 100
```

Press **Q** during playback to quit.
