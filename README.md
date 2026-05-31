# AeroWalk PDR - Pedestrian Dead Reckoning Inertial Tracker

AeroWalk PDR is a high-fidelity web application designed for mobile devices that calculates vectors and plots real-time walking paths $(X, Y)$ using the device's hardware sensors: the accelerometer (for step detection) and orientation sensors (gyroscope and magnetometer for heading).

It also includes a built-in **Workspace Test Simulator** so that developers and users can fully test the tracking, vector calculations, panning/zooming, and visual chart plotting directly from a desktop browser.

---

## 🚀 Key Features

*   **Real-time Sensor Processing:** Translates physical motion readings directly into relative displacements in real-time.
*   **Emulated Hardware Step Detection:** Employs a low-pass filter and peak-detection algorithm to isolate the rhythmic vertical acceleration of steps while filtering noise.
*   **Directional Heading Vectors:** Integrates yaw/compass angles (via gyro and magnetometer) to compute step displacement vectors ($\Delta X, \Delta Y$).
*   **Interactive 2D Path Plotter:** A zoomable, draggable grid canvas plotting steps, start positions, paths, and direction vectors.
*   **Sensor Telemetry Charting:** Custom high-performance canvas waveform chart showing raw magnitude, smoothed magnitude, and the dynamic step detection threshold.
*   **Desk Workspace Test Simulator:** Trigger steps, customize heading angles, auto-walk, or execute a perfect "Square Demo" to verify coordinate closure ($X=0, Y=0$) on desktop.

---

## 📐 How It Works (The Mathematics)

### 1. Step Detection
The app monitors the magnitude of the 3D acceleration vector:

$$a_{\text{mag}} = \sqrt{a_x^2 + a_y^2 + a_z^2}$$

To remove high-frequency noise, a **low-pass filter (LPF)** is applied with a smoothing coefficient $\alpha$ (typically $0.15$):

$$a_{\text{filtered}}[t] = \alpha \cdot a_{\text{mag}}[t] + (1 - \alpha) \cdot a_{\text{filtered}}[t-1]$$

A step is registered when $a_{\text{filtered}}$ crosses above a dynamic threshold (the running baseline average plus a sensitivity margin) and subsequently drops below a hysteresis threshold, subject to a refractory cooldown period (e.g., $350\text{ ms}$) to prevent double-counting.

### 2. Heading Determination
The orientation sensors extract the yaw angle ($\theta$ in degrees).
*   **Compass (Absolute) Mode:** Aligns directly with true geomagnetic North ($0^\circ$).
*   **Relative Mode:** Sets the direction of the device at the start of tracking as the reference direction ($0^\circ$ on screen).

### 3. Pedestrian Dead Reckoning (PDR) Coordinates
For every step detected at heading angle $\theta$ (converted to radians), the displacement vector is calculated as:

$$\Delta X = L \cdot \sin(\theta)$$

$$\Delta Y = L \cdot \cos(\theta)$$

Where:
*   $L$ is the user's calibrated **Stride Length** (e.g., $0.75\text{ meters}$).
*   $X$ coordinates track East-West displacement ($+X$ is East).
*   $Y$ coordinates track North-South displacement ($+Y$ is North).

The new position is accumulated:

$$X_{t} = X_{t-1} + \Delta X$$

$$Y_{t} = Y_{t-1} + \Delta Y$$

---

## 📂 File Structure

*   [index.html](file:///home/sohan/Downloads/PDR/index.html) - Structural framework, UI panels, metrics cards, map canvas, and simulator controls.
*   [style.css](file:///home/sohan/Downloads/PDR/style.css) - Styling rules, glassmorphism UI, responsive grids, and neon telemetry indicators.
*   [app.js](file:///home/sohan/Downloads/PDR/app.js) - Sensor event bindings, step detection algorithms, vector calculation, canvas renderings, and simulation sequences.

---

## 🏃 Running the Application

### Option 1: Start the Local Development Server
To access motion and orientation sensors, browsers require a secure context (either `HTTPS` or `localhost`). Starting a local server allows you to test it on your local browser immediately.

Run a simple Python HTTP server from the project directory:
```bash
python3 -m http.server 8080
```
Then navigate to: **`http://localhost:8080`**

### Option 2: Test on Mobile Devices
To test with real physical steps:
1. Ensure the server is hosted over `HTTPS` (or configure local port forwarding via SSH/ADB).
2. Open the page on your mobile device.
3. Click **"Enable Device Sensors"** and grant the browser permissions when prompted.
4. Set the phone flat in your hand pointing in your walking direction and begin walking!
