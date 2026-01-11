# Hailo Model Files

This directory contains pre-compiled HEF (Hailo Executable Format) models for the Hailo-8L AI accelerator.

## Required Model: YOLOv8s

The Dog Follower feature requires a YOLOv8s model compiled for Hailo-8L.

### Download Options

**Option 1: Hailo Model Zoo (Recommended)**
```bash
# Download pre-compiled YOLOv8s for Hailo-8L
wget -O ~/ros2_ws/src/rover1/models/yolov8s.hef \
  https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.14.0/hailo8l/yolov8s.hef
```

**Option 2: Hailo Developer Zone**
1. Visit https://hailo.ai/developer-zone/
2. Create an account or log in
3. Navigate to Model Zoo > Object Detection > YOLOv8
4. Download the Hailo-8L compatible HEF file
5. Place it in this directory as `yolov8s.hef`

**Option 3: Raspberry Pi AI Kit Models**
If using Raspberry Pi AI Kit, models may be available at:
```bash
ls /usr/share/hailo-models/
```

## Model Details

| Model | Input Size | COCO Classes | Dog Class ID |
|-------|------------|--------------|--------------|
| yolov8s.hef | 640x640 | 80 | 16 |

## Verification

After downloading, verify the model works:
```bash
# Check model info
hailortcli parse-hef yolov8s.hef

# Test inference (requires hailo_platform Python SDK)
python3 -c "from hailo_platform import HEF; h = HEF('yolov8s.hef'); print('Model loaded successfully')"
```

## Hailo Python SDK Installation

The `hailo_platform` Python package is required for inference. It's not on PyPI - download from Hailo Developer Zone.

**1. Download the wheel:**
- Visit https://hailo.ai/developer-zone/software-downloads/ (account required)
- Download: `hailort-4.23.0-cp312-cp312-linux_aarch64.whl` (for Pi 5 / Python 3.12)

**2. Install:**
```bash
pip3 install ~/hailort-4.23.0-cp312-cp312-linux_aarch64.whl
```

**3. Verify installation:**
```bash
# Check Hailo device is visible
hailortcli scan

# Get device info (serial, firmware version)
hailortcli fw-control identify

# Verify Python SDK works
python3 -c "from hailo_platform import HEF; print('Hailo SDK OK')"
```

## Notes

- HEF files are large (typically 10-50MB) and are gitignored
- The model must be compiled specifically for Hailo-8L (not Hailo-8)
- The dog_follower node will run in mock mode if the model is missing
- **Important:** PyHailoRT requires numpy 1.x (numpy 2.x is not supported)
