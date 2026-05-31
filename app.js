/**
 * AeroWalk Collect - PDR Trajectory Data Collector
 * Main Application Logic
 */

class PDRTracker {
    constructor() {
        // App State
        this.isTracking = false;
        this.steps = 0;
        this.distance = 0.0;
        this.heading = 0.0; // Current yaw heading (0 = North/Forward, 90 = East, etc.)
        this.startAbsoluteHeading = null; // Used for relative orientation alignment
        
        // Coordinates
        this.currentPos = { x: 0.0, y: 0.0 };
        this.path = [{ x: 0.0, y: 0.0, heading: 0.0, isStep: false, timestamp: Date.now() }];
        
        // Timing
        this.startTime = null;
        this.elapsedTime = 0; // in seconds
        this.timerInterval = null;
        
        // Sensor Calibrations (Adjustable in UI)
        this.strideLength = 0.75; // in meters
        this.stepSensitivity = 1.25; // m/s^2 above average baseline
        this.refractoryPeriod = 350; // ms cooldown
        this.filterAlpha = 0.15; // low-pass filter factor
        
        // Step Filtering Buffer
        this.accelHistory = [];
        this.maxHistorySize = 50; 
        this.lastFilteredMag = 9.81;
        this.lastStepTime = 0;
        this.crossedAbove = false;
        
        // Trajectory Logger Data Store
        this.dataLog = null;
        
        // Web Sensor Checks
        this.hasSensors = 'DeviceMotionEvent' in window && 'DeviceOrientationEvent' in window;
        
        // Visual Map Canvas
        this.mapCanvas = new PathCanvas('path-canvas');
        
        this.initDOM();
        this.initEventListeners();
        this.updateSensorUI();
    }

    initDOM() {
        // Primary Actions
        this.btnRecord = document.getElementById('btn-record-action');
        this.btnReset = document.getElementById('btn-reset-action');
        
        // Header Controls
        this.inputStride = document.getElementById('input-stride');
        this.sensorStatusDot = document.querySelector('#sensor-status .status-dot');
        this.sensorStatusLabel = document.querySelector('#sensor-status .status-label');
        
        // Floating HUD metrics
        this.hudSteps = document.getElementById('hud-steps');
        this.hudDistance = document.getElementById('hud-distance');
        this.hudHeading = document.getElementById('hud-heading');
        this.hudTime = document.getElementById('hud-time');
        
        // Rec Indicators
        this.recDot = document.getElementById('rec-dot');
        this.recText = document.getElementById('rec-text');
        
        // Canvas overlays
        this.compassPointer = document.getElementById('compass-pointer');
        this.introModal = document.getElementById('intro-modal');
        this.modalBtnStart = document.getElementById('modal-btn-start');
    }

    initEventListeners() {
        // Intro permission trigger
        this.modalBtnStart.addEventListener('click', () => {
            this.introModal.classList.add('hidden');
            this.requestSensorPermission();
        });

        // Trigger request manually if clicked status badge
        document.getElementById('sensor-status').addEventListener('click', () => {
            this.requestSensorPermission();
        });

        // Record Button (Unified Start/Stop Toggle)
        this.btnRecord.addEventListener('click', () => {
            if (this.isTracking) {
                this.stopTracking();
            } else {
                this.startTracking();
            }
        });

        // Reset Button
        this.btnReset.addEventListener('click', () => {
            this.resetTracking();
        });

        // Config stride listener
        this.inputStride.addEventListener('change', (e) => {
            const val = parseFloat(e.target.value);
            if (val > 0.1 && val < 2.5) {
                this.strideLength = val;
            }
        });

        // Zoom/Recenter
        document.getElementById('btn-zoom-in').addEventListener('click', () => this.mapCanvas.zoom(1.2));
        document.getElementById('btn-zoom-out').addEventListener('click', () => this.mapCanvas.zoom(0.8));
        document.getElementById('btn-recenter').addEventListener('click', () => this.mapCanvas.recenter());

        // Window resize
        window.addEventListener('resize', () => {
            this.mapCanvas.resize();
        });
    }

    updateSensorUI() {
        if (!this.hasSensors) {
            this.sensorStatusDot.className = 'status-dot inactive';
            this.sensorStatusLabel.innerText = 'Desktop Mode';
        }
    }

    async requestSensorPermission() {
        const needsMotionPerm = typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function';
        const needsOrientPerm = typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function';

        if (needsMotionPerm || needsOrientPerm) {
            try {
                this.sensorStatusLabel.innerText = 'Authorizing...';
                let motionGranted = true;
                let orientGranted = true;

                if (needsMotionPerm) {
                    const status = await DeviceMotionEvent.requestPermission();
                    motionGranted = (status === 'granted');
                }
                if (needsOrientPerm) {
                    const status = await DeviceOrientationEvent.requestPermission();
                    orientGranted = (status === 'granted');
                }

                if (motionGranted && orientGranted) {
                    this.onPermissionGranted();
                } else {
                    this.onPermissionDenied();
                }
            } catch (err) {
                console.error("Inertial sensor authorization failed:", err);
                this.onPermissionDenied();
            }
        } else {
            // Android/Desktop fallback
            this.onPermissionGranted();
        }
    }

    onPermissionGranted() {
        this.sensorStatusDot.className = 'status-dot active';
        this.sensorStatusLabel.innerText = 'Sensors: OK';
    }

    onPermissionDenied() {
        this.sensorStatusDot.className = 'status-dot inactive';
        this.sensorStatusLabel.innerText = 'Sensors: Denied';
        alert("Sensor permission denied. Please grant permission or run on a local secure context (HTTPS) to collect real walk trajectories.");
    }

    // TRACKING & DATA COLLECTION CONTROLS
    startTracking() {
        if (this.isTracking) return;
        
        this.isTracking = true;
        this.startTime = Date.now();
        this.elapsedTime = 0;
        
        // Initialize Trajectory Dataset Log
        this.dataLog = {
            metadata: {
                userAgent: navigator.userAgent,
                strideLengthMeters: this.strideLength,
                startTimeISO: new Date(this.startTime).toISOString(),
                sensorConfigurations: {
                    sensitivity: this.stepSensitivity,
                    refractoryMs: this.refractoryPeriod,
                    lowPassAlpha: this.filterAlpha
                }
            },
            path: [{
                stepIndex: 0,
                x: 0.0,
                y: 0.0,
                heading: this.heading,
                relativeTimeMs: 0
            }],
            sensors: [] // Stores high-frequency accelerometer/gyro timestamps for plotting or validation
        };

        // UI toggles
        this.btnRecord.innerHTML = '<i class="fa-solid fa-square"></i> Stop & Save Log';
        this.btnRecord.className = 'btn btn-record btn-stop';
        this.btnReset.disabled = true; // Lock reset during active session
        
        // HUD labels
        this.recDot.className = 'rec-dot recording';
        this.recText.innerText = 'RECORDING';
        this.recText.className = 'rec-text recording';

        // Duration clock
        this.timerInterval = setInterval(() => {
            if (this.startTime) {
                this.elapsedTime = Math.floor((Date.now() - this.startTime) / 1000);
                const minutes = Math.floor(this.elapsedTime / 60).toString().padStart(2, '0');
                const seconds = (this.elapsedTime % 60).toString().padStart(2, '0');
                this.hudTime.innerText = `${minutes}:${seconds}`;
            }
        }, 1000);

        // Bind raw device sensors
        this.bindSensorEvents();
        
        // Redraw grid
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    stopTracking() {
        if (!this.isTracking) return;
        
        this.isTracking = false;
        clearInterval(this.timerInterval);
        this.unbindSensorEvents();
        
        // Close data log
        const endTime = Date.now();
        this.dataLog.metadata.endTimeISO = new Date(endTime).toISOString();
        this.dataLog.metadata.durationSeconds = this.elapsedTime;
        this.dataLog.metadata.totalSteps = this.steps;
        this.dataLog.metadata.totalDistanceMeters = this.distance;
        
        // UI toggles
        this.btnRecord.innerHTML = '<i class="fa-solid fa-circle"></i> Start Recording';
        this.btnRecord.className = 'btn btn-record btn-start';
        this.btnReset.disabled = false; // Unlock reset now
        
        this.recDot.className = 'rec-dot';
        this.recText.innerText = 'IDLE';
        this.recText.className = 'rec-text';

        // Export data immediately
        this.downloadTrajectoryLog();
    }

    resetTracking() {
        this.steps = 0;
        this.distance = 0.0;
        this.heading = 0.0;
        this.startAbsoluteHeading = null;
        this.currentPos = { x: 0.0, y: 0.0 };
        this.path = [{ x: 0.0, y: 0.0, heading: 0.0, isStep: false, timestamp: Date.now() }];
        this.elapsedTime = 0;
        this.accelHistory = [];
        this.lastFilteredMag = 9.81;
        this.crossedAbove = false;
        this.dataLog = null;
        
        // Clear HUD
        this.hudSteps.innerText = '0';
        this.hudDistance.innerText = '0.0 m';
        this.hudHeading.innerText = '0°';
        this.hudTime.innerText = '00:00';
        
        this.compassPointer.style.transform = 'rotate(0deg)';
        
        this.btnReset.disabled = true; // Lock reset again
        
        this.mapCanvas.recenter();
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    bindSensorEvents() {
        this.deviceMotionHandler = this.handleDeviceMotion.bind(this);
        this.deviceOrientationHandler = this.handleDeviceOrientation.bind(this);
        window.addEventListener('devicemotion', this.deviceMotionHandler, true);
        window.addEventListener('deviceorientation', this.deviceOrientationHandler, true);
    }

    unbindSensorEvents() {
        if (this.deviceMotionHandler) {
            window.removeEventListener('devicemotion', this.deviceMotionHandler, true);
        }
        if (this.deviceOrientationHandler) {
            window.removeEventListener('deviceorientation', this.deviceOrientationHandler, true);
        }
    }

    // MOTION SIGNAL PROCESSING & STEP DETECTION
    handleDeviceMotion(event) {
        if (!this.isTracking) return;
        
        const accel = event.acceleration || event.accelerationIncludingGravity;
        if (!accel || accel.x === null) return;
        
        const x = accel.x || 0;
        const y = accel.y || 0;
        const z = accel.z || 0;
        const rawMag = Math.sqrt(x*x + y*y + z*z);

        // Smooth output via Low Pass filter
        const filteredMag = this.filterAlpha * rawMag + (1 - this.filterAlpha) * this.lastFilteredMag;
        this.lastFilteredMag = filteredMag;

        // Populate baseline queue
        this.accelHistory.push(filteredMag);
        if (this.accelHistory.length > this.maxHistorySize) {
            this.accelHistory.shift();
        }
        const baseline = this.accelHistory.reduce((a, b) => a + b, 0) / this.accelHistory.length;
        const upperThreshold = baseline + this.stepSensitivity;

        // Record high-resolution raw telemetry into data collector log
        const timestamp = Date.now();
        const relTime = timestamp - this.startTime;
        
        // Capture gyro rates if available
        let gyroAlpha = 0, gyroBeta = 0, gyroGamma = 0;
        if (event.rotationRate) {
            gyroAlpha = event.rotationRate.alpha || 0;
            gyroBeta = event.rotationRate.beta || 0;
            gyroGamma = event.rotationRate.gamma || 0;
        }

        this.dataLog.sensors.push({
            relativeTimeMs: relTime,
            accelRaw: { x, y, z, magnitude: rawMag },
            gyroRate: { alpha: gyroAlpha, beta: gyroBeta, gamma: gyroGamma }
        });

        // Core Step Detection Conditions
        if (filteredMag > upperThreshold && !this.crossedAbove) {
            if (timestamp - this.lastStepTime > this.refractoryPeriod) {
                this.crossedAbove = true;
            }
        }
        if (this.crossedAbove && filteredMag < baseline + (this.stepSensitivity * 0.3)) {
            this.registerStep(timestamp);
            this.crossedAbove = false;
        }
    }

    // ORIENTATION HEADING ESTIMATES
    handleDeviceOrientation(event) {
        if (!this.isTracking) return;
        
        let absoluteHeading = 0.0;
        if (event.webkitCompassHeading) {
            absoluteHeading = event.webkitCompassHeading;
        } else if (event.alpha !== null) {
            absoluteHeading = (360 - event.alpha) % 360;
        }

        // Keep standard relative heading mode (Start = 0)
        if (this.startAbsoluteHeading === null) {
            this.startAbsoluteHeading = absoluteHeading;
        }
        this.heading = (absoluteHeading - this.startAbsoluteHeading + 360) % 360;
        this.updateCompassUI(this.heading);
    }

    updateCompassUI(headingAngle) {
        this.hudHeading.innerText = `${headingAngle.toFixed(0)}°`;
        this.compassPointer.style.transform = `rotate(${headingAngle}deg)`;
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    // DISPLACEMENT MATHEMATICS
    registerStep(timestamp) {
        this.steps++;
        this.distance += this.strideLength;
        this.lastStepTime = timestamp;

        // UI Telemetry updates
        this.hudSteps.innerText = this.steps;
        this.hudDistance.innerText = `${this.distance.toFixed(1)} m`;
        this.hudSteps.classList.add('flash');
        setTimeout(() => this.hudSteps.classList.remove('flash'), 250);

        // Vector calculation: North is +Y, East is +X
        const rad = (this.heading * Math.PI) / 180;
        const dx = this.strideLength * Math.sin(rad);
        const dy = this.strideLength * Math.cos(rad);

        this.currentPos.x += dx;
        this.currentPos.y += dy;

        const relTime = timestamp - this.startTime;
        const stepPt = {
            x: this.currentPos.x,
            y: this.currentPos.y,
            heading: this.heading,
            isStep: true,
            timestamp: timestamp
        };
        
        this.path.push(stepPt);

        // Log this step inside the dataset coordinates list
        this.dataLog.path.push({
            stepIndex: this.steps,
            x: this.currentPos.x,
            y: this.currentPos.y,
            heading: this.heading,
            relativeTimeMs: relTime
        });

        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    // TRAJECTORY EXPORTS
    downloadTrajectoryLog() {
        if (!this.dataLog || this.dataLog.path.length === 0) {
            alert('No path data was logged.');
            return;
        }

        const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(this.dataLog, null, 2));
        const downloadAnchor = document.createElement('a');
        downloadAnchor.setAttribute("href", dataStr);
        downloadAnchor.setAttribute("download", `aerowalk_collect_pdr_${Date.now()}.json`);
        document.body.appendChild(downloadAnchor);
        downloadAnchor.click();
        downloadAnchor.remove();
    }
}

/**
 * PATH CANVAS RENDERER
 */
class PathCanvas {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.zoomFactor = 40; // Pixels per meter
        this.panOffset = { x: 0, y: 0 };
        this.isDragging = false;
        this.dragStart = { x: 0, y: 0 };
        
        this.initEvents();
        this.resize();
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        if (rect.width > 0 && rect.height > 0) {
            this.canvas.width = rect.width * dpr;
            this.canvas.height = rect.height * dpr;
            this.canvas.style.width = `${rect.width}px`;
            this.canvas.style.height = `${rect.height}px`;
            this.ctx.scale(dpr, dpr);
        }
        if (this.lastDrawData) {
            this.draw(...this.lastDrawData);
        }
    }

    initEvents() {
        // Desktop Pan
        this.canvas.addEventListener('mousedown', (e) => {
            this.isDragging = true;
            this.dragStart.x = e.clientX - this.panOffset.x;
            this.dragStart.y = e.clientY - this.panOffset.y;
        });
        window.addEventListener('mousemove', (e) => {
            if (!this.isDragging) return;
            this.panOffset.x = e.clientX - this.dragStart.x;
            this.panOffset.y = e.clientY - this.dragStart.y;
            if (this.lastDrawData) this.draw(...this.lastDrawData);
        });
        window.addEventListener('mouseup', () => {
            this.isDragging = false;
        });

        // Touch Pan (Mobile)
        this.canvas.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                this.isDragging = true;
                this.dragStart.x = e.touches[0].clientX - this.panOffset.x;
                this.dragStart.y = e.touches[0].clientY - this.panOffset.y;
                e.preventDefault();
            }
        }, { passive: false });

        this.canvas.addEventListener('touchmove', (e) => {
            if (!this.isDragging || e.touches.length !== 1) return;
            this.panOffset.x = e.touches[0].clientX - this.dragStart.x;
            this.panOffset.y = e.touches[0].clientY - this.dragStart.y;
            if (this.lastDrawData) this.draw(...this.lastDrawData);
            e.preventDefault();
        }, { passive: false });

        this.canvas.addEventListener('touchend', () => {
            this.isDragging = false;
        });

        // Wheel Zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomMultiplier = e.deltaY < 0 ? 1.15 : 0.85;
            this.zoom(zoomMultiplier);
        }, { passive: false });
    }

    zoom(multiplier) {
        const targetZoom = this.zoomFactor * multiplier;
        if (targetZoom >= 5 && targetZoom <= 200) {
            this.zoomFactor = targetZoom;
            
            const scaleOverlay = document.getElementById('scale-indicator');
            const metersPerGrid = this.zoomFactor > 80 ? 0.5 : (this.zoomFactor < 20 ? 5 : 1);
            scaleOverlay.innerText = `Grid line = ${metersPerGrid} meter${metersPerGrid === 1 ? '' : 's'}`;
            
            if (this.lastDrawData) this.draw(...this.lastDrawData);
        }
    }

    recenter() {
        this.panOffset = { x: 0, y: 0 };
        this.zoomFactor = 40;
        document.getElementById('scale-indicator').innerText = 'Grid = 1 meter';
        if (this.lastDrawData) this.draw(...this.lastDrawData);
    }

    draw(path, currentPos, heading) {
        this.lastDrawData = [path, currentPos, heading];
        const width = this.canvas.width / (window.devicePixelRatio || 1);
        const height = this.canvas.height / (window.devicePixelRatio || 1);
        
        if (width === 0 || height === 0) return;
        
        // Clean slate
        this.ctx.fillStyle = '#04060a';
        this.ctx.fillRect(0, 0, width, height);
        
        const startX = width / 2 + this.panOffset.x;
        const startY = height / 2 + this.panOffset.y;
        
        let gridSize = 1;
        if (this.zoomFactor > 80) gridSize = 0.5;
        else if (this.zoomFactor < 20) gridSize = 5.0;
        
        const stepPx = gridSize * this.zoomFactor;
        
        this.ctx.lineWidth = 1;
        
        // Draw vertical axes
        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
        let startGridX = startX % stepPx;
        for (let x = startGridX; x < width; x += stepPx) {
            this.ctx.beginPath();
            this.ctx.strokeStyle = Math.abs(x - startX) < 1.0 ? 'rgba(0, 240, 255, 0.12)' : 'rgba(255, 255, 255, 0.03)';
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, height);
            this.ctx.stroke();
        }
        
        // Draw horizontal axes
        let startGridY = startY % stepPx;
        for (let y = startGridY; y < height; y += stepPx) {
            this.ctx.beginPath();
            this.ctx.strokeStyle = Math.abs(y - startY) < 1.0 ? 'rgba(0, 240, 255, 0.12)' : 'rgba(255, 255, 255, 0.03)';
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(width, y);
            this.ctx.stroke();
        }

        const toCanvasCoords = (pt) => {
            return {
                x: startX + pt.x * this.zoomFactor,
                y: startY - pt.y * this.zoomFactor // Canvas Y increases downwards, PDR increases upwards (North)
            };
        };

        // Draw Line Path
        if (path.length > 1) {
            this.ctx.beginPath();
            const startPt = toCanvasCoords(path[0]);
            this.ctx.moveTo(startPt.x, startPt.y);
            
            for (let i = 1; i < path.length; i++) {
                const pt = toCanvasCoords(path[i]);
                this.ctx.lineTo(pt.x, pt.y);
            }
            
            this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.85)';
            this.ctx.lineWidth = 3.5;
            this.ctx.lineCap = 'round';
            this.ctx.lineJoin = 'round';
            this.ctx.shadowBlur = 8;
            this.ctx.shadowColor = 'rgba(0, 240, 255, 0.3)';
            this.ctx.stroke();
            this.ctx.shadowBlur = 0;
        }

        // Draw Step Coordinates
        path.forEach((pt, index) => {
            if (index === 0) return;
            const px = toCanvasCoords(pt);
            
            this.ctx.beginPath();
            this.ctx.arc(px.x, px.y, 4.5, 0, 2 * Math.PI);
            this.ctx.fillStyle = '#bf5af2';
            this.ctx.strokeStyle = '#04060a';
            this.ctx.lineWidth = 1;
            this.ctx.fill();
            this.ctx.stroke();
        });

        // Draw Start Position Marker (Green)
        this.ctx.beginPath();
        this.ctx.arc(startX, startY, 7, 0, 2 * Math.PI);
        this.ctx.fillStyle = '#30d158';
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 1.5;
        this.ctx.fill();
        this.ctx.stroke();
        
        this.ctx.font = 'bold 9px Outfit, sans-serif';
        this.ctx.fillStyle = '#30d158';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('START', startX, startY - 11);

        // Draw Current Directional Triangle
        const curPx = toCanvasCoords(currentPos);
        
        this.ctx.save();
        this.ctx.translate(curPx.x, curPx.y);
        
        // PDR Heading is 0 deg = North (+Y). Converting to canvas clockwise rotation.
        const canvasAngle = (heading * Math.PI) / 180 - Math.PI / 2;
        this.ctx.rotate(canvasAngle);
        
        this.ctx.shadowBlur = 15;
        this.ctx.shadowColor = 'rgba(0, 240, 255, 0.8)';
        
        this.ctx.beginPath();
        this.ctx.moveTo(12, 0);
        this.ctx.lineTo(-8, -8);
        this.ctx.lineTo(-4, 0);
        this.ctx.lineTo(-8, 8);
        this.ctx.closePath();
        
        this.ctx.fillStyle = '#00f0ff';
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 1.5;
        this.ctx.fill();
        this.ctx.stroke();
        this.ctx.restore();
    }
}

// Instantiate App
window.addEventListener('DOMContentLoaded', () => {
    window.app = new PDRTracker();
});
