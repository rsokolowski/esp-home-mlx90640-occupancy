# MLX90640 Occupancy Detector for ESPHome

This component provides occupancy detection using the MLX90640 thermal camera with ESPHome. It uses computer vision techniques to detect human presence based on thermal signatures.

> **⚠️ IMPORTANT**: The KNN algorithm implementation is experimental and has not been tested. It likely does not work correctly at this time. Please use the baseline algorithm instead.

> **✅ Proven Use Case**: The author of this component has successfully used it for bathroom occupancy detection with the baseline algorithm.

## Features

- **Thermal Occupancy Detection**: Uses MLX90640 thermal camera (32x24 pixel resolution) to detect human presence
- **Multiple Detection Algorithms**:
  - **Baseline**: Simple background subtraction with thermal thresholding
  - **KNN**: K-Nearest Neighbors background subtraction for improved detection
- **Auto-Calibration**: Automatically calibrates background model on startup
- **Customizable Parameters**: Configure sensitivity, thresholds, and debounce timing
- **Monitoring Sensors**: Optional sensors to monitor detection performance
- **ESPHome Integration**: Integrates with Home Assistant through ESPHome

## Hardware Requirements

- ESP32-based board (tested with ESP32-C3 Mini Wemos)
- MLX90640 thermal camera module
- I2C connection between ESP32 and MLX90640

## Real-World Usage

This component has been successfully deployed and tested for bathroom occupancy detection. The thermal camera can reliably detect human presence without privacy concerns associated with traditional cameras. 

A few tips for bathroom installation:
- Mount the sensor on the ceiling for best coverage
- Position it to capture the entire usable area
- Adjust the temperature thresholds based on your bathroom's typical ambient temperature
- Consider using a waterproof enclosure for humidity protection

## Installation

1. Copy the `components/mlx90640` directory to your ESPHome `components` folder
2. Configure your YAML file (see example below)
3. Deploy to your ESP32 device using ESPHome

## Configuration

Example configuration:

```yaml
mlx90640:
  id: thermal_cam
  update_interval: 0.3s
  sda: 6  # I2C SDA PIN
  scl: 10  # I2C SCL PIN
  frequency: 400000  # I2C Clock Frequency
  address: 0x33  # MLX90640 Address
  refresh_rate: 4  # 0x04 for 8Hz (recommended) or 0x05 for 16Hz

  # Occupancy detection
  algorithm_type: baseline  # Use "baseline" (KNN is experimental and untested)
  temp_diff_threshold: 1.0  # Temperature difference threshold in °C
  blob_size_threshold: 10   # Size threshold for a blob to be considered a person
  human_temp_diff_min: 1.0  # Minimum average temperature difference for a human
  human_temp_diff_max: 15.0  # Maximum average temperature difference for a human
  calibration_frames: 5     # Number of frames to use for calibration
  exit_debounce_frames: 3   # Number of consecutive empty frames for unoccupied state
  blob_proximity_threshold: 1  # Pixels of proximity to exclude from background updates
  
  # Occupancy binary sensor
  occupancy:
    name: occupancy

  # Optional monitoring sensors
  enable_monitoring: true
  blob_size:
    name: blob_size
  temp_diff:
    name: blob_temp_diff
  blob_details:
    name: blob_details
    icon: "mdi:motion-sensor"
  loop_duration:
    name: loop_duration

# Optional calibration button
button:
  - platform: template
    name: calibrate_background
    icon: "mdi:refresh"
    entity_category: "config"
    on_press:
      then:
        - lambda: 'id(thermal_cam).start_calibration();'
```

## Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `update_interval` | How often to poll the sensor | 0.3s |
| `sda` | I2C SDA pin | - |
| `scl` | I2C SCL pin | - |
| `frequency` | I2C clock frequency | 400000 |
| `address` | MLX90640 I2C address | 0x33 |
| `refresh_rate` | MLX90640 refresh rate (0x04=8Hz, 0x05=16Hz) | 0x05 |
| `algorithm_type` | Detection algorithm (baseline or knn) | baseline |
| `temp_diff_threshold` | Temperature difference threshold (°C) | 2.0 |
| `blob_size_threshold` | Size threshold for blobs (pixels) | 50 |
| `human_temp_diff_min` | Minimum temp difference for humans (°C) | 2.0 |
| `human_temp_diff_max` | Maximum temp difference for humans (°C) | 5.0 |
| `calibration_frames` | Number of frames used for calibration | 10 |
| `exit_debounce_frames` | Frames needed to transition to unoccupied | 5 |
| `blob_proximity_threshold` | Proximity threshold for blob detection | 1 |
| `enable_monitoring` | Enable monitoring sensors | true |

## How It Works

1. **Background Calibration**: On startup, the system captures several frames to build a thermal background model.
2. **Motion Detection**: New frames are compared to the background model to detect thermal anomalies.
3. **Blob Analysis**: Connected components analysis identifies blobs of significant temperature difference.
4. **Human Classification**: Blobs are classified as human based on size and temperature characteristics.
5. **State Management**: Debounce logic prevents rapid state changes from false detections.

### Detection Algorithms

#### Baseline Algorithm
Simple background subtraction that maintains a thermal model of the room and detects significant deviations. This algorithm has been tested and works reliably for bathroom occupancy detection.

#### KNN Algorithm
More advanced K-Nearest Neighbors approach that maintains a history of temperature values for each pixel and compares new readings against this history. **Note: This algorithm is experimental and has not been thoroughly tested. It may not function correctly.**

## Troubleshooting

- **False Positives**: Adjust `temp_diff_threshold` and `blob_size_threshold` higher
- **False Negatives**: Lower thresholds or increase `human_temp_diff_max`
- **Slow Response**: Decrease `exit_debounce_frames` and ensure `update_interval` isn't too long
- **High CPU Usage**: Increase `update_interval` or reduce `refresh_rate`

## Performance Considerations

- The MLX90640 sensor and occupancy detection algorithms are computationally intensive
- Recommended to use ESP32 (not ESP8266) for adequate performance
- Monitoring the `loop_duration` sensor can help diagnose performance issues

## License

This project includes code from the MLX90640 library and is provided under the same license terms.

## Credits

- Original MLX90640 driver from Melexis
- ESPHome integration and occupancy detection algorithms