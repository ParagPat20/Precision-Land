# Precision Landing Camera Configuration (OV9281 Mono & 64MP Pi Camera)

## Tuned Hardware Camera Profile (OV9281 Monochrome 120° Wide FOV)

All vision scripts (`lib_aruco_pose.py`, `aruco_pose_estimation.py`, `save_snapshots.py`, `main.py`) are pre-configured with the tuned Guvcview profile for crisp, high-altitude checkerboard and ArUco marker tracking without floor glare:

- **Resolution**: `1280x720` (720p 16:9 Full Resolution)
- **FPS**: `120 FPS` (High throughput Motion-JPEG)
- **Camera Output**: `MJPG - Motion-JPEG`
- **Brightness**: `-15` (Reduces blown-out tile highlights)
- **Contrast**: `35` (Sharpened black & white square contrast)
- **Gamma**: `75`
- **Gain**: `33`
- **Sharpness**: `20`
- **Exposure Time Absolute**: `20`
- **Backlight Compensation**: `1`

## Camera Calibration

### Step 1: Capture Calibration Images

```bash
cd opencv

# Capture 1280x720 calibration snapshots with tuned high-contrast profile
python save_snapshots.py --folder snaps --name snapshot --dwidth 1280 --dheight 720
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
# For 1280x720 calibration images
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
# Test ArUco detection at 1280x720 @ 120 FPS
python aruco_pose_estimation.py
```

This will:
- Open camera at 1280x720 resolution @ 120 FPS
- Detect ArUco markers (ID: 72 by default)
- Show position and orientation with high-contrast anti-glare tuned profile

### Test with Main Script

```bash
cd ../src

# Test precision landing (requires drone connection)
python main.py --connect /dev/ttyACM0
```

## Configuration Files Updated

All files have been updated to support the tuned high-resolution hardware profile:

1. ✅ **lib_aruco_pose.py** - Default: `1280x720 @ 120 FPS` (Brightness -15, Contrast 35, Gain 33, Exposure 20)
2. ✅ **aruco_pose_estimation.py** - Default: `1280x720 @ 120 FPS` (Tuned Guvcview profile)
3. ✅ **save_snapshots.py** - Default: `1280x720 @ 120 FPS` (Tuned Guvcview profile)
4. ✅ **main.py** - Default: `1280x720 @ 120 FPS`
5. ✅ **cameracalib.py** - Processes 1280x720 high-contrast snapshots

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

