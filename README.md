# Project 4: Camera Calibration and Augmented Reality
**CS5330 — Computer Vision and Pattern Recognition**
Northeastern University, Khoury College of Computer Sciences

**Author:** Drumil Kotecha
**Date:** March 2026


---

## Time Travel Days
None used.

---

## Project Overview
This project implements a full camera calibration and augmented reality pipeline using OpenCV in C++. It consists of four separate executables:

| Executable | Description |
|---|---|
| `calibration` | Detects checkerboard corners in live video, accumulates calibration frames, computes camera intrinsic parameters, and saves them to a YAML file |
| `ar_viewer` | Loads saved intrinsics, estimates checkerboard pose in real time using solvePnP, and overlays a 3D wireframe tower onto the live video feed |
| `features` | Runs Harris corner detection on a live video stream with interactive trackbars to control threshold and block size |
| `static_ar` | Applies the full AR pipeline to pre-captured static images rather than live video |

---

## Environment and Dependencies

| Item | Details |
|---|---|
| OS | macOS (Apple Silicon, arm64) |
| IDE | CLion |
| Build System | CMake 3.20+ |
| Language | C++14 |
| Dependencies | OpenCV 4.13.0 (Homebrew) |

Install OpenCV:
```bash
brew install opencv
```

---

## Build Instructions

**Via CLion:**
1. Open the project in CLion
2. CLion auto-detects `CMakeLists.txt` and configures the project
3. Select **Build → Rebuild Project**
4. All four executables appear in the run target dropdown

**Via command line:**
```bash
cd <project_root>
mkdir cmake-build-debug && cd cmake-build-debug
cmake -DCMAKE_PREFIX_PATH=/opt/homebrew ..
make -j8
```

---

## Running the Project

> **Important:** Always run `calibration` first. All other executables depend on `intrinsics.yaml` which calibration generates.

### Step 1 — `calibration`
- Display `checkerboard.png` fullscreen on a phone or tablet
- Hold it in front of the webcam at various angles and distances
- Press `s` at least 8 times from different positions to save frames
- Press `c` to compute calibration
- `intrinsics.yaml` is saved to `cmake-build-debug/` automatically

| Key | Action |
|---|---|
| `s` | Save current frame (checkerboard must be detected) |
| `c` | Run calibration (minimum 5 frames required) |
| `q` | Quit |

### Step 2 — `ar_viewer`
- Point the webcam at the checkerboard
- The wireframe tower appears floating above the board
- Press `h` to replace the checkerboard with a synthetic green ground plane

| Key | Action |
|---|---|
| `a` | Toggle 3D axes (X=red, Y=green, Z=blue) |
| `c` | Toggle outer corner projection |
| `v` | Toggle virtual tower object |
| `h` | Toggle hide-checkerboard mode |
| `q` | Quit |

### Step 3 — `features`
- Point the webcam at any scene with edges and corners
- Red dots mark strong Harris corners
- A second window displays the Harris response as a color heatmap

| Control | Action |
|---|---|
| Threshold trackbar | Adjust Harris response threshold (0–255) |
| Block Size trackbar | Adjust Harris neighborhood size (2–7) |
| `q` | Quit |

### Step 4 — `static_ar`
- Requires `intrinsics.yaml` and `calib_*.jpg` files in `cmake-build-debug/`
- Automatically loads each calibration image and overlays the AR tower
- Press any key to advance to the next image
- Output saved as `static_ar_*.jpg` in `cmake-build-debug/`

---

## File Structure

```
project4/
├── CMakeLists.txt            Build configuration
├── main_calibration.cpp      Tasks 1, 2, 3 — corner detection and calibration
├── main_ar.cpp               Tasks 4, 5, 6 — pose estimation and AR overlay
├── main_features.cpp         Task 7 — Harris corner feature detection
├── main_static.cpp           Extension 2 — static image AR
├── report.pdf                Project report
└── README.md                 This file

Generated at runtime (cmake-build-debug/):
├── intrinsics.yaml           Camera matrix and distortion coefficients
├── calib_*.jpg               Clean calibration frames
└── static_ar_*.jpg           AR output images from static_ar
```

---

## Checkerboard Target
The checkerboard used is the provided `checkerboard.png` with **9 columns and 6 rows of internal corners** (54 total), displayed fullscreen on a phone screen. This is defined in each source file as:
```cpp
const cv::Size BOARD(9, 6);
```
If a different checkerboard is used, this value must be updated in all four source files.

---

## Calibration Results

**Camera Matrix:**
```
[1071.74,    0,       947.86]
[0,          1071.74, 537.62]
[0,          0,       1     ]
```

**Distortion Coefficients (k1, k2, p1, p2, k3):**
```
[-0.0370, -0.0079, -0.0009, -0.0012, 0.0161]
```

**Re-projection Error:** 0.33 pixels (excellent — below 1.0 is acceptable)

---

## Extensions Implemented

### 1. Interactive GUI with Real-Time Trackbars
**File:** `main_features.cpp`
Two OpenCV trackbars (Threshold, Block Size) allow real-time adjustment of Harris corner detection parameters. Changes are reflected instantly in the live video feed.

### 2. AR on Static Images
**File:** `main_static.cpp`
The full AR pipeline is applied to pre-captured static images rather than live video, demonstrating that the calibration generalizes beyond a live camera stream.

### 3. Hiding the Checkerboard Target
**File:** `main_ar.cpp` — press `h` during `ar_viewer`
The checkerboard is concealed at runtime by filling its projected boundary with a synthetic green ground plane, making the AR scene appear without any visible printed target.

---

## Known Limitations
- The checkerboard must be fully visible in frame for detection to succeed. Partial occlusion or extreme angles will cause detection to fail.
- Phone screen glare can interfere with detection. Tilting the phone 10–15 degrees away from direct light resolves this in most cases.
- Only one executable can access the webcam at a time. Fully stop the current target before running another.
- `static_ar` requires clean calibration images. Images with corner overlays drawn on them will fail re-detection.

---

## Acknowledgements
- OpenCV documentation: https://docs.opencv.org
- Calibration tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Checkerboard target provided by course (`checkerboard.png`)
- Implementation guidance and debugging: Claude (Anthropic)