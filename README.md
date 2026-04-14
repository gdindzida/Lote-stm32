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

### Download the UAV dataset

The HIL test uses the **ALTO UAV dataset** ([MetaSLAM/ALTO](https://github.com/MetaSLAM/ALTO)).

1. Visit the [ALTO GitHub page](https://github.com/MetaSLAM/ALTO) and follow the dataset request / download instructions to obtain the dataset.
2. Extract the archive so you have a folder layout like:
   ```
   alto-dataset/
   └── UAV/
       ├── Train/
       │   ├── query_images/
       │   └── reference_images/
       │       └── offset_0_None/
       ├── Val/
       └── Test/
   ```
3. Pass the path to one of the split folders (e.g. `UAV/Train`) as `--data-root` when running the HIL script.

### Connect the board

Plug the STM32 board via USB. The `start_hil.sh` script auto-detects the STM32 CDC serial port by its USB vendor ID (0x0483) — no manual port configuration needed.

### Run HIL

Use the provided launcher script from the repo root. It sources the virtual environment and sets up the Python path automatically:

```bash
./tools/start_hil.sh --data-root <path/to/UAV/split_folder> [OPTIONS]
```

The `--data-root` must point to one of the UAV dataset split folders (e.g. `Train`, `Val`) that contains `query_images/` and `reference_images/offset_0_None/` subdirectories, e.g.:

```bash
./tools/start_hil.sh \
  --data-root ../alto-dataset/UAV/Train \
  --write-freq 6
```

### Options

| Flag | Type | Default | Description |
|---|---|---|---|
| `--data-root PATH` | string | *(required)* | Path to a UAV split folder (e.g. `UAV/Train`) |
| `--write-freq HZ` | float | `30` | Frame send rate to the STM32 (Hz) |
| `--playback DELAY_MS` | int | — | After the run, replay frames with a fixed delay (ms) between each |
| `--playback-realtime` | flag | — | After the run, replay frames using the original inter-frame timings |

### Examples

Run at 6 Hz and replay with original timings:
```bash
./tools/start_hil.sh \
  --data-root ../alto-dataset/UAV/Train \
  --write-freq 6 \
  --playback-realtime
```

Run at maximum throughput with a 100 ms fixed playback delay:
```bash
./tools/start_hil.sh \
  --data-root ../alto-dataset/UAV/Train \
  --playback 100
```

Press **Q** + Enter during streaming to stop early and proceed to playback.

Press **Q** during playback to quit.
