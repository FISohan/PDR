# AeroWalk Collect - PDR Trajectory Data Collector

AeroWalk Collect is a lightweight, minimal web utility designed for recording and logging Pedestrian Dead Reckoning (PDR) walking paths $(X, Y)$ using mobile device sensors. 

By removing all visual graphs and secondary diagnostic tables, the interface is optimized for full-screen trajectory mapping, enabling clear visibility and smooth panning/zooming during collection runs.

---

## 🚀 Key Features

*   **Fullscreen Interactive Trajectory Map:** Zoomable and draggable coordinate grid canvas showing the recorded path, steps, start marker, and orientation pointer.
*   **Start & Stop Logging Workflow:** Simple start/stop recording button. Once clicked, the app connects to raw hardware sensors and plots movements. Stopping the tracking instantly compiles the log and triggers a file download.
*   **Sensor Data Logging:** Logs high-frequency accelerometer and gyroscope measurements (timestamps, raw forces, rotation rates) alongside the resolved $(X, Y)$ step coordinates for offline validation and research.
*   **Calibrated Stride Length:** Easy input in the header to calibrate displacements.
*   **Mobile Touch Gesture Optimization:** Prevented window scroll bouncing when panning the map canvas on a touchscreen.

---

## 📐 Inertial Math Formulation

### 1. Vector Displacements
On every step detected at relative heading $\theta$ (yaw), the displacement coordinates increase by:

$$\Delta X = L \cdot \sin(\theta)$$

$$\Delta Y = L \cdot \cos(\theta)$$

Where:
*   $L$ is the user's calibrated **Stride Length** (e.g. $0.75$ meters).
*   $X$ tracks East-West displacement ($+X$ is East).
*   $Y$ tracks North-South displacement ($+Y$ is North).

### 2. Step Detection
Uses a software-level hardware step emulator. It calculates the length of the 3D acceleration vector, filters high-frequency noise with a low-pass filter, computes a moving baseline average, and uses hysteresis crossing triggers to detect walking strides while ignoring phone tremors.

---

## 📊 Collected Data Format (JSON Export)

When you stop recording, a JSON log file named `aerowalk_collect_pdr_[timestamp].json` is automatically downloaded. The structure of the logged file is formatted as:

```json
{
  "metadata": {
    "userAgent": "Mozilla/5.0 ...",
    "strideLengthMeters": 0.75,
    "startTimeISO": "2026-05-31T23:50:00.000Z",
    "endTimeISO": "2026-05-31T23:52:30.000Z",
    "durationSeconds": 150,
    "totalSteps": 184,
    "totalDistanceMeters": 138,
    "sensorConfigurations": {
      "sensitivity": 1.25,
      "refractoryMs": 350,
      "lowPassAlpha": 0.15
    }
  },
  "path": [
    {
      "stepIndex": 0,
      "x": 0,
      "y": 0,
      "heading": 0,
      "relativeTimeMs": 0
    },
    {
      "stepIndex": 1,
      "x": 0.25,
      "y": 0.707,
      "heading": 20,
      "relativeTimeMs": 620
    }
  ],
  "sensors": [
    {
      "relativeTimeMs": 10,
      "accelRaw": {
        "x": 0.12,
        "y": 9.78,
        "z": 1.45,
        "magnitude": 9.88
      },
      "gyroRate": {
        "alpha": 0.05,
        "beta": -0.02,
        "gamma": 0.1
      }
    }
  ]
}
```

---

## 🏃 Running and Deploying

### Option 1: Start the Local Development Server
Because browsers restrict motion sensor APIs to secure context hosts, running a server locally is recommended:
```bash
python3 -m http.server 8080
```
Navigate to: **`http://localhost:8080`**

### Option 2: Deploy to Mobile Devices
1. Deploy the directory contents to any static host supporting HTTPS (e.g. Netlify, GitHub Pages, Vercel).
2. Open the URL on your phone's browser.
3. Tap **"Grant Access & Begin"** to authorize sensors.
4. Input your stride length, tap **"Start Recording"**, hold your phone flat pointing forward, and walk!
5. Tap **"Stop & Save Log"** when finished to receive your file.
