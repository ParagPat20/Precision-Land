# Precision Landing Camera Configuration (OV9281 Mono & 64MP Pi Camera)

## Supported Camera Configurations

### 1. OV9281 USB Monochrome Camera (12MP / 120° Wide FOV) - Recommended Default
- **Type**: USB Global Shutter Monochrome (Black & White) Camera
- **Resolution**: 1280x720 (Full HD / 720p 16:9 120° Wide FOV)
- **FPS**: 60 - 120 FPS (High Performance MJPG throughput)
- **Focus**: Fixed Focus (No Autofocus needed/supported)
- **Processing Advantage**: Ultra-fast grayscale capture with minimal CPU load

### 2. Arducam 64MP / Pi Camera Module 3 (Picamera2)
- **Type**: CSI Pi Camera Module
- **Resolution**: 640x360 (Default for reduced CPU load on high-MP sensor)
- **FPS**: 30 FPS
- **Focus**: Continuous Autofocus (`AfMode: 2`) enabled via Picamera2

## Camera Calibration

### Step 1: Capture Calibration Images

```bash
cd opencv

# Capture calibration images at 640x360 (recommended)
python save_snapshots.py --folder snaps --name snapshot --dwidth 640 --dheight 360

# OR capture at native resolution (slower but more accurate)
python save_snapshots.py --folder snaps --name snapshot --dwidth 2304 --dheight 1296
```

**Instructions**:
- Press `s` to save snapshot
- Press `q` to quit
- Capture at least **20-25 images** with the chessboard at different:
  - Angles (tilted, rotated)
  - Distances (near and far)
  - Positions (center, corners, edges)

### Step 2: Run Camera Calibration

```bash
# For 640x360 calibration images
python cameracalib.py snaps jpg 9 6 25

# The script will generate:
# - cameraMatrix.txt
# - cameraDistortion.txt
# - calibresult.png (verification image)
```

### Step 3: Rename Calibration Files

```bash
# Rename the generated files
mv snaps/cameraMatrix.txt cameraMatrix_webcam.txt
mv snaps/cameraDistortion.txt cameraDistortion_webcam.txt
```

## Testing ArUco Detection

### Test Camera and ArUco Detection

```bash
# Test ArUco detection at 640x360
python aruco_pose_estimation.py
```

This will:
- Open camera at 640x360 resolution
- Detect ArUco markers (ID: 72 by default)
- Show position and orientation
- Display 120° wide field of view

### Test with Main Script

```bash
cd ../src

# Test precision landing (requires drone connection)
python main.py --connect <connection_string>
```

## Configuration Files Updated

All files have been updated to support the new camera:

1. ✅ **lib_aruco_pose.py** - Default: 640x360
2. ✅ **aruco_pose_estimation.py** - Default: 640x360
3. ✅ **save_snapshots.py** - Default: 640x360
4. ✅ **main.py** - Uses 640x360 for processing
5. ✅ **cameracalib.py** - Uses whatever images you provide

## Performance Comparison

| Resolution | ArUco Detection | Total FPS | Memory Usage |
|------------|----------------|-----------|--------------|
| 2304x1296  | ~1-2 Hz        | 1-2 Hz    | High         |
| 640x360    | ~10-15 Hz      | 10-15 Hz  | Low          |

**Recommendation**: Use **640x360** for real-time precision landing while maintaining the full 120° field of view.

## Switching Between Resolutions

### To use native resolution (2304x1296):

```python
# In main.py
camera_resolution = [2304, 1296]

# In aruco_pose_estimation.py
REQ_W, REQ_H = 2304, 1296
```

### To use optimized resolution (640x360) - RECOMMENDED:

```python
# In main.py (already configured)
camera_resolution = [640, 360]

# In aruco_pose_estimation.py (already configured)
REQ_W, REQ_H = 640, 360
```

## Camera Module Installation

### Enable Camera Interface

```bash
sudo raspi-config
# Select: Interface Options -> Camera -> Enable
```

### Install Picamera2 Library

```bash
sudo apt-get update
sudo apt-get install -y python3-picamera2
```

### Verify Camera Detection

```bash
# Check if camera is detected
vcgencmd get_camera

# Should output: supported=1 detected=1

# List camera modes
libcamera-hello --list-cameras
```

## Troubleshooting

### Camera not detected
```bash
# Enable legacy camera support if needed
sudo modprobe bcm2835-v4l2

# Or add to /etc/modules
echo "bcm2835-v4l2" | sudo tee -a /etc/modules
```

### Wrong aspect ratio or FOV
- Make sure you're using 16:9 resolution (640x360, not 640x480)
- Recalibrate camera at the correct resolution

### Low FPS
- Use 640x360 instead of native resolution
- Disable unnecessary video display during processing
- Consider image downscaling for ArUco detection

## Field of View Visualization

```
┌─────────────────────────────────────────┐
│         120° Wide Field of View          │
│                                          │
│    ╱                              ╲      │
│   ╱                                ╲     │
│  ╱           ArUco Marker           ╲    │
│ ╱              [ID: 72]              ╲   │
│╱                                      ╲  │
│          Visible Landing Area            │
│                                          │
│   16:9 Aspect Ratio (640x360)            │
└─────────────────────────────────────────┘
```

The wide 120° FOV allows you to see more of the landing area, making it easier to detect and track the ArUco marker during descent.

## Additional Notes

- **NoIR (No Infrared Filter)**: Better for low-light conditions and IR illumination
- **Wide Angle**: 120° FOV is excellent for precision landing as it provides better situational awareness
- **16:9 Aspect Ratio**: Modern standard that matches the sensor's native capabilities

---

**Last Updated**: Based on Raspberry Pi Module 3 NoIR Wide specifications

