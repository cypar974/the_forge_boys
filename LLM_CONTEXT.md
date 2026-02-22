# Project Context: Computer Vision for Line-Following Robot

## Project Goal
Developing a computer vision system using Python and OpenCV for a car robot (controlled by an Arduino Uno R4 WiFi) to follow an electrical tape line on the floor.

## Current Progress

### Environment Setup
- **Venv**: Python virtual environment configured in `.venv`.
- **Dependencies**: `opencv-python` (v4.13.0) and `numpy` installed.

### Core Capabilities (`main.py`)
- **Video Capture**: Captures from `VideoCapture(1)` (adjustable via code).
- **Video Transformation**: Rotates footage 90 degrees clockwise for correct robot orientation.
- **Robust Preprocessing**:
  - **Lighting Normalization**: Uses CLAHE (Contrast Limited Adaptive Histogram Equalization) to maintain stability under changing light.
  - **Noise Reduction**: Gaussian blur and morphological operations (Open/Close).
- **Advanced Line Detection**:
  - **HSV Filtering**: Isolates the line based on color.
  - **Detection Modes**:
    - `Mode 0 (Manual)`: Full control via HSV sliders.
    - `Mode 1 (Red)`: Optimized for red tape (handles hue wrap-around).
    - `Mode 2 (White)`: Optimized for white tape.
  - **Line Segment Logic**: Projects a line segment between two centroids (from the bottom 25% and middle 25%-50% of the frame).
- **Robot Feedback**:
  - Defines a "Middle" zone centered in the frame (width adjustable via "Mid Width" slider).
  - Status updates:
    - **Fully Contained**: Entire segment is within the middle zone.
    - **Partial/Outside**: Part or all of the segment is outside the bounds.
    - **Left/Right/Good**: Fallback simple logic when only the bottom part is visible.

### Tuning Interface
A dedicated "Tuning" window with real-time trackbars for:
- HSV Boundaries (L-H, L-S, L-V, U-H, U-S, U-V)
- Middle Zone Width (`Mid Width`)
- Detection Mode (`Mode`)

## Planned Future Work
- **Hardware Integration**: Implement communication with the Arduino Uno R4 WiFi (likely via Serial or Wi-Fi/UDP).
- **Navigation Logic**: Optimize robot steering commands based on the "Fully Contained" status and segment orientation.
- **Robustness**: Handling intersections, sharp turns, and line loss scenarios.

## Technical Details
- **Main Script**: `main.py`
- **Virtual Environment**: `.venv/`
- **Tuning UI**: OpenCV highgui windows ("Robot view", "Tuning", "Mask").
