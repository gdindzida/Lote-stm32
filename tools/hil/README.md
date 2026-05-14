# HIL Testing Framework

## Overview

The HIL (Hardware-in-the-Loop) testing framework for STM32 optical flow processing. The framework uses CSV-based datasets with matched rectified images and sensor data for accurate, reproducible testing.

## Features

### CSV Dataset Streamer

Reads from `dataset.csv` files created by `tools/calibration/create_dataset.py`:
- Loads rectified images based on paths in the CSV
- Provides camera timestamps for accurate frame timing
- Validates image file existence during initialization

### Timestamp-Based Frame Writing

The writer thread supports two timing modes:

**Timestamp Mode** (when `--write-freq` is provided):
- Uses actual camera timestamps from dataset.csv
- Preserves original inter-frame timing from the dataset
- Computes delays based on consecutive frame timestamps
- Ideal for replaying real-world scenarios

**Maximum Throughput Mode** (when `--write-freq` is omitted):
- Sends frames as fast as possible
- No throttling or delays
- Limited only by STM32 processing speed

## Usage

### Basic Example

```bash
python -m hil.run_hil \
    --dataset-csv /path/to/dataset.csv \
    --data-root /path/to/dataset/root \
    --start-frame 1 \
    --write-freq 30
```

### With Playback and KPI

```bash
python -m hil.run_hil \
    --dataset-csv /path/to/dataset.csv \
    --data-root /path/to/dataset/root \
    --start-frame 1 \
    --write-freq 30 \
    --playback-realtime \
    --kpi
```

### Arguments

**Required:**
- `--dataset-csv PATH`: Path to dataset.csv file

**Optional:**
- `--data-root PATH`: Root directory for resolving relative image paths
- `--start-frame N`: 1-based frame number to start from (default: 1000)
- `--write-freq HZ`: Frame write frequency (default: 30 Hz). Omit for max throughput
- `--playback DELAY_MS`: Replay frames with fixed delay after run
- `--playback-realtime`: Replay frames with original timing after run
- `--kpi`: Compute accuracy metrics
- `--plot-kpi`: Display velocity timeseries plots
- `--plot`: Display timing bar chart
- `--save-dir PATH`: Save annotated frames during playback
- `--timeout MS`: Serial connection timeout

## Dataset CSV Format

The `dataset.csv` file must contain:

```csv
timestamp_cam,timestamp_sensor,delta_t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,roll,pitch,yaw,image_path
1614007547.996533394,1614007547.991618395,0.004914999,-0.054541063,-0.017789572,0.143988267,-0.999777675,0.018020593,-0.006256673,0.008990357,-0.036156233,0.012186843,-0.018204571,rectified_img/44.png
```

**Required columns:**
- `timestamp_cam`: Camera timestamp (seconds)
- `image_path`: Relative or absolute path to rectified image

**Optional columns (for ground truth / KPI):**
- `timestamp_sensor`: Matched sensor timestamp
- `p_x, p_y, p_z`: Position (meters)
- `q_w, q_x, q_y, q_z`: Quaternion orientation
- `roll, pitch, yaw`: Euler angles (radians)

## Creating a Dataset CSV

Use the provided calibration tool:

```bash
python tools/calibration/create_dataset.py \
    /path/to/nav_cam_root \
    /path/to/sensors_root \
    dataset.csv
```

This tool:
1. Loads camera timestamps from `nav_cam_timestamps.csv`
2. Loads sensor data from `mocap_vehicle_data.csv`
3. Matches each rectified image to the latest sensor reading
4. Computes roll/pitch/yaw from quaternions
5. Writes matched dataset to CSV

## Implementation Details

### Timing Algorithm

**Timestamp Mode:**
```python
# Get timestamps from dataset
curr_ts = streamer.get_timestamp(curr_idx)
prev_ts = streamer.get_timestamp(prev_idx)

# Calculate inter-frame delay
dataset_delay = curr_ts - prev_ts

# Schedule relative to last frame
deadline = frame_write_times[-1] + dataset_delay
sleep_time = deadline - time.time()
if sleep_time > 0:
    time.sleep(sleep_time)
```

**Max Throughput Mode:**
```python
# No delay - send immediately
deadline = time.time()
```

### File Structure

```
tools/hil/
├── __init__.py
├── csv_dataset.py          # CSV dataset streamer
├── frames.py               # Frame data structures
├── kpi.py                  # KPI computation
├── playback.py             # Frame playback with visualization
├── plot.py                 # Timing plots
├── protocol.py             # Serial protocol definitions
├── run_hil.py              # Main entry point
├── stats.py                # Statistics printing
├── stm32.py                # STM32 port detection
├── streamer.py             # Base streamer interface
├── threads.py              # Reader/writer threads
└── README.md               # This file
```

## Benefits

1. **Accuracy**: Preserve original timing from dataset
2. **Reproducibility**: Consistent frame timing across runs
3. **Flexibility**: Use pre-calibrated, rectified images
4. **Modularity**: Clean separation between data loading and HIL logic
5. **Validation**: Image existence checked at load time

## Notes

- The STM32 has a 2-frame receive buffer
- Frames may be skipped if processing falls behind
- Statistics and timing plots help identify performance issues
- KPI computation requires sensor ground truth data in the CSV
