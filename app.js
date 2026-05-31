/**
 * AeroWalk PDR - Pedestrian Dead Reckoning Inertial Tracker
 * Core Application Logic
 */

class PDRTracker {
    constructor() {
        // App State
        this.isTracking = false;
        this.steps = 0;
        this.distance = 0.0;
        this.heading = 0.0; // Current heading in degrees (0 = North, 90 = East, etc.)
        this.headingMode = 'relative'; // 'relative' or 'absolute'
        this.startAbsoluteHeading = null; // Yaw offset for relative tracking
        
        // Coordinates and Path
        this.currentPos = { x: 0.0, y: 0.0 }; // in meters relative to start (0,0)
        this.path = [{ x: 0.0, y: 0.0, heading: 0.0, isStep: false, timestamp: Date.now() }];
        
        // Timing
        this.startTime = null;
        this.elapsedTime = 0; // in seconds
        this.timerInterval = null;
        
        // Sensor Parameters
        this.strideLength = 0.75; // in meters
        this.stepSensitivity = 1.2; // m/s^2 above gravity/baseline
        this.refractoryPeriod = 350; // min time between steps in ms
        this.filterAlpha = 0.15; // low-pass filter smoothing coefficient
        
        // Sensor Data Buffer for Filtering & Step Detection
        this.accelHistory = [];
        this.maxHistorySize = 50; // ~1 second of data at 50Hz
        this.lastFilteredMag = 9.81; // starts at gravity
        this.lastStepTime = 0;
        this.crossedAbove = false;
        
        // Live Telemetry Values
        this.rawAccel = { x: 0, y: 0, z: 0 };
        this.rawGyro = { alpha: 0, beta: 0, gamma: 0 };
        this.orientation = { alpha: 0, beta: 0, gamma: 0 };
        
        // Web APIs Availability
        this.hasAccel = 'DeviceMotionEvent' in window;
        this.hasOrient = 'DeviceOrientationEvent' in window;
        
        // Visual Components
        this.mapCanvas = new PathCanvas('path-canvas');
        this.sensorChart = new SensorChart('sensor-chart');
        
        this.initDOM();
        this.initEventListeners();
        this.updateSensorAvailabilityUI();
        
        // Set initial UI values
        this.updateConfigValues();
    }

    initDOM() {
        // Core buttons
        this.btnPermission = document.getElementById('btn-request-permission');
        this.btnStart = document.getElementById('btn-start-tracking');
        this.btnStop = document.getElementById('btn-stop-tracking');
        this.btnReset = document.getElementById('btn-reset-path');
        
        // Config inputs
        this.inputStride = document.getElementById('input-stride');
        this.inputSensitivity = document.getElementById('input-sensitivity');
        this.inputRefractory = document.getElementById('input-refractory');
        this.radioHeadingRelative = document.querySelector('input[name="heading-mode"][value="relative"]');
        this.radioHeadingAbsolute = document.querySelector('input[name="heading-mode"][value="absolute"]');
        
        // Config text labels
        this.valStride = document.getElementById('val-stride');
        this.valSensitivity = document.getElementById('val-sensitivity');
        this.valRefractory = document.getElementById('val-refractory');
        
        // Metrics displays
        this.metricSteps = document.getElementById('metric-steps');
        this.metricDistance = document.getElementById('metric-distance');
        this.metricHeading = document.getElementById('metric-heading');
        this.metricHeadingDir = document.getElementById('metric-heading-dir');
        this.metricTime = document.getElementById('metric-time');
        
        // Live table values
        this.valAccel = document.getElementById('val-accel');
        this.valAccelMag = document.getElementById('val-accel-mag');
        this.valGyro = document.getElementById('val-gyro');
        this.valOrient = document.getElementById('val-orient');
        
        // Status Indicators
        this.sensorStatusDot = document.querySelector('#sensor-status .status-dot');
        this.sensorStatusLabel = document.querySelector('#sensor-status .status-label');
        this.trackingStatusDot = document.querySelector('#tracking-status .status-dot');
        this.trackingStatusLabel = document.querySelector('#tracking-status .status-label');
        
        // Compass Overlay
        this.compassPointer = document.getElementById('compass-pointer');
        
        // Modal
        this.introModal = document.getElementById('intro-modal');
        this.modalBtnStart = document.getElementById('modal-btn-start');
        
        // Simulator elements
        this.btnSimStep = document.getElementById('btn-sim-step');
        this.inputSimHeading = document.getElementById('input-sim-heading');
        this.valSimHeading = document.getElementById('val-sim-heading');
        this.btnSimAutoWalk = document.getElementById('btn-sim-auto-walk');
        this.btnSimDemoSquare = document.getElementById('btn-sim-demo-square');
    }

    initEventListeners() {
        // Modal Event
        this.modalBtnStart.addEventListener('click', () => {
            this.introModal.classList.add('hidden');
            // Try triggering permission check to make it seamless
            this.requestSensorPermission();
        });

        // Permission Button
        this.btnPermission.addEventListener('click', () => this.requestSensorPermission());

        // Start/Stop
        this.btnStart.addEventListener('click', () => this.startTracking());
        this.btnStop.addEventListener('click', () => this.stopTracking());
        this.btnReset.addEventListener('click', () => this.resetTracking());

        // Config Sliders
        this.inputStride.addEventListener('input', (e) => {
            this.strideLength = parseFloat(e.target.value);
            this.valStride.innerText = `${this.strideLength.toFixed(2)} m`;
        });
        
        this.inputSensitivity.addEventListener('input', (e) => {
            this.stepSensitivity = parseFloat(e.target.value);
            this.valSensitivity.innerText = `${this.stepSensitivity.toFixed(1)} m/s²`;
        });

        this.inputRefractory.addEventListener('input', (e) => {
            this.refractoryPeriod = parseInt(e.target.value);
            this.valRefractory.innerText = `${this.refractoryPeriod} ms`;
        });

        // Heading Mode Radios
        const handleHeadingModeChange = (e) => {
            this.headingMode = e.target.value;
            if (this.isTracking) {
                // reset starting heading if switching modes mid-run
                this.startAbsoluteHeading = null; 
            }
        };
        this.radioHeadingRelative.addEventListener('change', handleHeadingModeChange);
        this.radioHeadingAbsolute.addEventListener('change', handleHeadingModeChange);

        // Simulator controls
        this.btnSimStep.addEventListener('click', () => {
            const angle = parseFloat(this.inputSimHeading.value);
            this.triggerSimulatedStep(angle);
        });

        this.inputSimHeading.addEventListener('input', (e) => {
            const angle = parseInt(e.target.value);
            this.valSimHeading.innerText = `${angle}° (${this.getHeadingDirection(angle)})`;
            // Also update main heading metric & compass pointer during simulation
            this.heading = angle;
            this.updateOrientationUI(angle);
        });

        // Simulator Auto-Walk
        this.simWalkInterval = null;
        this.btnSimAutoWalk.addEventListener('click', () => {
            if (this.simWalkInterval) {
                clearInterval(this.simWalkInterval);
                this.simWalkInterval = null;
                this.btnSimAutoWalk.innerHTML = '<i class="fa-solid fa-person-walking"></i> Auto-Walk';
                this.btnSimAutoWalk.classList.remove('active');
            } else {
                if (!this.isTracking) this.startTracking();
                this.btnSimAutoWalk.innerHTML = '<i class="fa-solid fa-stop"></i> Stop Walk';
                this.btnSimAutoWalk.classList.add('active');
                this.simWalkInterval = setInterval(() => {
                    const angle = parseFloat(this.inputSimHeading.value);
                    this.triggerSimulatedStep(angle);
                }, 800);
            }
        });

        // Simulator Square Demo
        this.btnSimDemoSquare.addEventListener('click', () => {
            this.runSquareDemo();
        });

        // Map Canvas Controls
        document.getElementById('btn-zoom-in').addEventListener('click', () => this.mapCanvas.zoom(1.2));
        document.getElementById('btn-zoom-out').addEventListener('click', () => this.mapCanvas.zoom(0.8));
        document.getElementById('btn-recenter').addEventListener('click', () => this.mapCanvas.recenter());
        document.getElementById('btn-export').addEventListener('click', () => this.exportPathData());

        // Handle window resizing
        window.addEventListener('resize', () => {
            this.mapCanvas.resize();
            this.sensorChart.resize();
        });
    }

    updateConfigValues() {
        this.valStride.innerText = `${this.strideLength.toFixed(2)} m`;
        this.valSensitivity.innerText = `${this.stepSensitivity.toFixed(1)} m/s²`;
        this.valRefractory.innerText = `${this.refractoryPeriod} ms`;
        
        const simAngle = parseInt(this.inputSimHeading.value);
        this.valSimHeading.innerText = `${simAngle}° (${this.getHeadingDirection(simAngle)})`;
    }

    updateSensorAvailabilityUI() {
        // If not running on a mobile browser or missing sensors, warn
        if (!this.hasAccel && !this.hasOrient) {
            this.sensorStatusDot.className = 'status-dot inactive';
            this.sensorStatusLabel.innerText = 'Sensors: Not Available';
            this.btnPermission.innerHTML = '<i class="fa-solid fa-triangle-exclamation"></i> Desktop Mode (Sensors Not Found)';
            this.btnPermission.classList.add('btn-danger');
            this.btnStart.disabled = false; // Allow starting tracking to test with simulator
        }
    }

    async requestSensorPermission() {
        // For iOS devices requiring explicit permissions
        const needsMotionPerm = typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function';
        const needsOrientPerm = typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function';

        if (needsMotionPerm || needsOrientPerm) {
            try {
                this.sensorStatusLabel.innerText = 'Requesting permissions...';
                
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
                    this.onPermissionDenied("Permissions rejected by user.");
                }
            } catch (err) {
                console.error("Error requesting device sensor permissions:", err);
                this.onPermissionDenied(err.message);
            }
        } else {
            // Standard browser without requestPermission model (e.g. Chrome on Android, Desktop browsers)
            // Just bind the listeners and check if events actually fire
            this.onPermissionGranted();
        }
    }

    onPermissionGranted() {
        this.sensorStatusDot.className = 'status-dot active';
        this.sensorStatusLabel.innerText = 'Sensors: Access Granted';
        this.btnPermission.innerHTML = '<i class="fa-solid fa-circle-check"></i> Sensors Configured';
        this.btnPermission.className = 'btn btn-outline';
        this.btnPermission.disabled = true;
        this.btnStart.disabled = false;
        
        // Listeners will be attached during startTracking
    }

    onPermissionDenied(message) {
        this.sensorStatusDot.className = 'status-dot inactive';
        this.sensorStatusLabel.innerText = 'Sensors: Denied';
        alert(`Sensor access was denied: ${message}\n\nYou can still test path plotting using the Workspace Test Simulator.`);
        // Allow using simulator anyway
        this.btnStart.disabled = false;
    }

    // TRACKING LIFECYCLE
    startTracking() {
        if (this.isTracking) return;
        
        this.isTracking = true;
        this.startTime = Date.now() - (this.elapsedTime * 1000);
        
        this.btnStart.disabled = true;
        this.btnStop.disabled = false;
        
        this.trackingStatusDot.className = 'status-dot active';
        this.trackingStatusLabel.innerText = 'Tracking: Active';
        
        // Start duration timer
        this.timerInterval = setInterval(() => {
            if (this.startTime) {
                this.elapsedTime = Math.floor((Date.now() - this.startTime) / 1000);
                const minutes = Math.floor(this.elapsedTime / 60).toString().padStart(2, '0');
                const seconds = (this.elapsedTime % 60).toString().padStart(2, '0');
                this.metricTime.innerText = `${minutes}:${seconds}`;
            }
        }, 1000);
        
        // Bind Hardware Event Listeners
        this.bindSensorEvents();
        
        // Redraw initial canvas state
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    stopTracking() {
        if (!this.isTracking) return;
        
        this.isTracking = false;
        this.btnStart.disabled = false;
        this.btnStop.disabled = true;
        
        this.trackingStatusDot.className = 'status-dot inactive';
        this.trackingStatusLabel.innerText = 'Tracking: Paused';
        
        // Stop timer
        clearInterval(this.timerInterval);
        
        // Stop Simulator loop if running
        if (this.simWalkInterval) {
            clearInterval(this.simWalkInterval);
            this.simWalkInterval = null;
            this.btnSimAutoWalk.innerHTML = '<i class="fa-solid fa-person-walking"></i> Auto-Walk';
            this.btnSimAutoWalk.classList.remove('active');
        }
        
        // Unbind Sensors to conserve battery
        this.unbindSensorEvents();
    }

    resetTracking() {
        this.stopTracking();
        
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
        
        // Reset UI metrics
        this.metricSteps.innerText = '0';
        this.metricDistance.innerText = '0.00 m';
        this.metricHeading.innerText = '0.0°';
        this.metricHeadingDir.innerText = 'North';
        this.metricTime.innerText = '00:00';
        
        // Reset Sensor tables
        this.valAccel.innerText = '0.00, 0.00, 0.00 m/s²';
        this.valAccelMag.innerText = '0.00 m/s²';
        this.valGyro.innerText = '0.0, 0.0, 0.0 °/s';
        this.valOrient.innerText = 'Yaw: 0°, Pitch: 0°, Roll: 0°';
        
        // Reset compass indicator
        this.compassPointer.style.transform = `rotate(0deg)`;
        
        // Reset canvas & charts
        this.mapCanvas.recenter();
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
        this.sensorChart.clear();
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

    // MOTION DATA PROCESSING & STEP DETECTION (Software Step Emulator)
    handleDeviceMotion(event) {
        if (!this.isTracking) return;
        
        // Get linear acceleration (preferred) or total acceleration
        let accel = event.acceleration || event.accelerationIncludingGravity;
        if (!accel || (accel.x === null && accel.y === null && accel.z === null)) {
            // Accelerometer coordinates are missing or null
            return;
        }
        
        this.rawAccel = {
            x: accel.x || 0,
            y: accel.y || 0,
            z: accel.z || 0
        };

        // If gyroscope rotation rate is available, capture it
        if (event.rotationRate) {
            this.rawGyro = {
                alpha: event.rotationRate.alpha || 0,
                beta: event.rotationRate.beta || 0,
                gamma: event.rotationRate.gamma || 0
            };
            this.valGyro.innerText = `${this.rawGyro.alpha.toFixed(1)}, ${this.rawGyro.beta.toFixed(1)}, ${this.rawGyro.gamma.toFixed(1)} °/s`;
        }
        
        // Update raw telemetry display
        this.valAccel.innerText = `${this.rawAccel.x.toFixed(2)}, ${this.rawAccel.y.toFixed(2)}, ${this.rawAccel.z.toFixed(2)} m/s²`;
        
        // Calculate 3D vector magnitude
        const rawMag = Math.sqrt(
            this.rawAccel.x * this.rawAccel.x +
            this.rawAccel.y * this.rawAccel.y +
            this.rawAccel.z * this.rawAccel.z
        );
        this.valAccelMag.innerText = `${rawMag.toFixed(2)} m/s²`;

        // Apply low-pass filter to smooth raw magnitude oscillations
        const filteredMag = this.filterAlpha * rawMag + (1 - this.filterAlpha) * this.lastFilteredMag;
        this.lastFilteredMag = filteredMag;

        // Keep rolling history of filtered acceleration to compute moving baseline
        this.accelHistory.push(filteredMag);
        if (this.accelHistory.length > this.maxHistorySize) {
            this.accelHistory.shift();
        }

        // Calculate moving average baseline
        const sum = this.accelHistory.reduce((a, b) => a + b, 0);
        const baseline = sum / this.accelHistory.length;

        // Dynamic step threshold is based on baseline + sensitivity
        const upperThreshold = baseline + this.stepSensitivity;
        
        // Feed real-time telemetry to chart visualizer
        this.sensorChart.addData(rawMag, filteredMag, upperThreshold);

        // Core Step Detection logic
        const currentTime = Date.now();
        
        // Cross above threshold
        if (filteredMag > upperThreshold && !this.crossedAbove) {
            // Ensure refractory cooldown period has passed to avoid noise doubling
            if (currentTime - this.lastStepTime > this.refractoryPeriod) {
                this.crossedAbove = true;
            }
        }
        
        // If we crossed above, wait for the peak and subsequent downward cross (Hysteresis)
        // A standard drop below threshold (or slightly below upperThreshold) confirms the step
        if (this.crossedAbove && filteredMag < baseline + (this.stepSensitivity * 0.3)) {
            this.registerStep(currentTime);
            this.crossedAbove = false;
        }
    }

    // ORIENTATION DATA PROCESSING
    handleDeviceOrientation(event) {
        if (!this.isTracking) return;
        
        // alpha: rotation around z-axis (yaw) [0, 360]
        // beta: rotation around x-axis (pitch) [-180, 180]
        // gamma: rotation around y-axis (roll) [-90, 90]
        this.orientation = {
            alpha: event.alpha || 0,
            beta: event.beta || 0,
            gamma: event.gamma || 0
        };

        // Update Orientation UI display
        this.valOrient.innerText = `Yaw: ${this.orientation.alpha.toFixed(0)}°, Pitch: ${this.orientation.beta.toFixed(0)}°, Roll: ${this.orientation.gamma.toFixed(0)}°`;

        let absoluteHeading = 0.0;
        
        // Extract Compass/Heading
        if (event.webkitCompassHeading) {
            // iOS provides direct compass heading (clockwise degrees)
            absoluteHeading = event.webkitCompassHeading;
        } else if (event.alpha !== null) {
            // Standard Android: alpha is counter-clockwise rotation around Z-axis
            // North is typically 0, and alpha increases counter-clockwise (West = 90, South = 180, East = 270)
            // Convert to standard compass clockwise degrees (North=0, East=90, South=180, West=270)
            absoluteHeading = (360 - event.alpha) % 360;
        }

        // Apply heading mode offset
        if (this.headingMode === 'absolute') {
            this.heading = absoluteHeading;
        } else {
            // Relative mode: we want the initial heading when starting to align to 0 (North/Forward on screen)
            if (this.startAbsoluteHeading === null) {
                this.startAbsoluteHeading = absoluteHeading;
            }
            this.heading = (absoluteHeading - this.startAbsoluteHeading + 360) % 360;
        }

        this.updateOrientationUI(this.heading);
    }

    updateOrientationUI(headingAngle) {
        // Update visual elements
        this.metricHeading.innerText = `${headingAngle.toFixed(1)}°`;
        this.metricHeadingDir.innerText = this.getHeadingDirection(headingAngle);
        
        // Rotate the compass compass pointer overlay (clockwise)
        this.compassPointer.style.transform = `rotate(${headingAngle}deg)`;
        
        // Redraw canvas with updated direction pointer
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    // REGISTRATION AND VECTOR CALCULATION (PDR Core Equation)
    registerStep(timestamp) {
        this.steps++;
        this.distance += this.strideLength;
        this.lastStepTime = timestamp;

        // Visual flash updates for user delight
        this.metricSteps.innerText = this.steps;
        this.metricDistance.innerText = `${this.distance.toFixed(2)} m`;
        this.metricSteps.classList.add('flash');
        setTimeout(() => this.metricSteps.classList.remove('flash'), 300);

        // Pedestrian Dead Reckoning Vector Math:
        // Convert heading angle to radians. Note standard compass angles: 
        // 0 deg = North (+Y), 90 deg = East (+X), 180 deg = South (-Y), 270 deg = West (-X)
        const rad = (this.heading * Math.PI) / 180;
        const dx = this.strideLength * Math.sin(rad);
        const dy = this.strideLength * Math.cos(rad);

        // Accumulate displacement
        this.currentPos.x += dx;
        this.currentPos.y += dy;

        // Store step coordinates for canvas drawing
        this.path.push({
            x: this.currentPos.x,
            y: this.currentPos.y,
            heading: this.heading,
            isStep: true,
            timestamp: timestamp
        });

        // Trigger pulse on scrolling chart
        this.sensorChart.triggerStepMarker();

        // Draw fresh path
        this.mapCanvas.draw(this.path, this.currentPos, this.heading);
    }

    // WORKSPACE TEST SIMULATOR UTILS
    triggerSimulatedStep(headingAngle) {
        if (!this.isTracking) {
            // Automatically turn on tracking for seamless testing
            this.startTracking();
        }
        
        // Briefly overwrite current heading for this simulation tick
        this.heading = headingAngle;
        this.registerStep(Date.now());
    }

    async runSquareDemo() {
        this.resetTracking();
        this.startTracking();
        
        const stepsPerSide = 5;
        const delayBetweenSteps = 500; // ms
        
        const directions = [
            { angle: 0, label: 'North' },
            { angle: 90, label: 'East' },
            { angle: 180, label: 'South' },
            { angle: 270, label: 'West' }
        ];

        this.btnSimDemoSquare.disabled = true;
        this.btnSimDemoSquare.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Running Demo...';

        let totalStepDelay = 0;

        for (const dir of directions) {
            // Update UI slider to display current heading simulation angle
            setTimeout(() => {
                this.inputSimHeading.value = dir.angle;
                this.valSimHeading.innerText = `${dir.angle}° (${dir.label})`;
                this.heading = dir.angle;
                this.updateOrientationUI(dir.angle);
            }, totalStepDelay);

            for (let i = 0; i < stepsPerSide; i++) {
                setTimeout(() => {
                    this.triggerSimulatedStep(dir.angle);
                }, totalStepDelay + (i * delayBetweenSteps));
            }
            
            totalStepDelay += stepsPerSide * delayBetweenSteps;
        }

        // Clean up buttons when finished
        setTimeout(() => {
            this.btnSimDemoSquare.disabled = false;
            this.btnSimDemoSquare.innerHTML = '<i class="fa-solid fa-square-full"></i> Square Demo';
            // Recenter view once complete to align shape beautifully
            this.mapCanvas.recenter();
        }, totalStepDelay + 100);
    }

    // HELPER FUNCTIONS
    getHeadingDirection(degree) {
        const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
        const index = Math.round(((degree % 360) / 45)) % 8;
        return directions[index];
    }

    exportPathData() {
        if (this.path.length === 0) {
            alert('No path data to export.');
            return;
        }

        const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify({
            metadata: {
                totalSteps: this.steps,
                totalDistanceMeters: this.distance,
                strideLengthMeters: this.strideLength,
                elapsedTimeSeconds: this.elapsedTime,
                exportedAt: new Date().toISOString()
            },
            coordinates: this.path
        }, null, 2));

        const downloadAnchor = document.createElement('a');
        downloadAnchor.setAttribute("href", dataStr);
        downloadAnchor.setAttribute("download", `aerowalk_pdr_path_${Date.now()}.json`);
        document.body.appendChild(downloadAnchor);
        downloadAnchor.click();
        downloadAnchor.remove();
    }
}

/**
 * PATH CANVAS VISUALIZER
 * Renders zoomable and pannable grid map, step dots, path lines, and start/end points.
 */
class PathCanvas {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        
        // Zoom and Pan State
        this.zoomFactor = 40; // Pixels per meter (scale)
        this.panOffset = { x: 0, y: 0 }; // Camera offsets in pixels
        this.isDragging = false;
        this.dragStart = { x: 0, y: 0 };
        
        this.initEvents();
        this.resize();
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        // Account for high-DPI displays
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.canvas.style.width = `${rect.width}px`;
        this.canvas.style.height = `${rect.height}px`;
        this.ctx.scale(dpr, dpr);
        
        if (this.lastDrawData) {
            this.draw(...this.lastDrawData);
        }
    }

    initEvents() {
        // Dragging & Panning
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

        // Touch support for mobile panning
        this.canvas.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                this.isDragging = true;
                this.dragStart.x = e.touches[0].clientX - this.panOffset.x;
                this.dragStart.y = e.touches[0].clientY - this.panOffset.y;
            }
        }, { passive: true });

        window.addEventListener('touchmove', (e) => {
            if (!this.isDragging || e.touches.length !== 1) return;
            this.panOffset.x = e.touches[0].clientX - this.dragStart.x;
            this.panOffset.y = e.touches[0].clientY - this.dragStart.y;
            if (this.lastDrawData) this.draw(...this.lastDrawData);
        }, { passive: true });

        window.addEventListener('touchend', () => {
            this.isDragging = false;
        });

        // Mouse Wheel Zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomMultiplier = e.deltaY < 0 ? 1.15 : 0.85;
            this.zoom(zoomMultiplier);
        }, { passive: false });
    }

    zoom(multiplier) {
        // Bound zoom scale
        const targetZoom = this.zoomFactor * multiplier;
        if (targetZoom >= 5 && targetZoom <= 200) {
            this.zoomFactor = targetZoom;
            
            // Adjust overlay text indicator
            const scaleOverlay = document.getElementById('scale-indicator');
            const metersPerGrid = this.zoomFactor > 80 ? 0.5 : (this.zoomFactor < 20 ? 5 : 1);
            scaleOverlay.innerText = `Grid line density = ${metersPerGrid} meter${metersPerGrid === 1 ? '' : 's'}`;
            
            if (this.lastDrawData) this.draw(...this.lastDrawData);
        }
    }

    recenter() {
        this.panOffset = { x: 0, y: 0 };
        this.zoomFactor = 40;
        document.getElementById('scale-indicator').innerText = `1 grid unit = 1 meter`;
        if (this.lastDrawData) this.draw(...this.lastDrawData);
    }

    // MAIN CANVAS RENDER ENGINE
    draw(path, currentPos, heading) {
        // Cache parameters to handle redraws on pan/resize
        this.lastDrawData = [path, currentPos, heading];
        
        const width = this.canvas.width / (window.devicePixelRatio || 1);
        const height = this.canvas.height / (window.devicePixelRatio || 1);
        
        // Clean slate
        this.ctx.fillStyle = '#080b11';
        this.ctx.fillRect(0, 0, width, height);
        
        // Center coordinates of screen (Start position of PDR tracking)
        const startX = width / 2 + this.panOffset.x;
        const startY = height / 2 + this.panOffset.y;
        
        // Dynamic Grid spacing (varies with zoom level for visual clarity)
        let gridSize = 1; // default 1 meter
        if (this.zoomFactor > 80) gridSize = 0.5; // fine grid
        else if (this.zoomFactor < 20) gridSize = 5.0; // coarse grid
        
        const stepPx = gridSize * this.zoomFactor;
        
        // 1. Draw Grid Lines
        this.ctx.lineWidth = 1;
        
        // Vertical lines
        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
        let startGridX = startX % stepPx;
        for (let x = startGridX; x < width; x += stepPx) {
            this.ctx.beginPath();
            // Highlight axis center line
            this.ctx.strokeStyle = Math.abs(x - startX) < 1.0 ? 'rgba(0, 240, 255, 0.15)' : 'rgba(255, 255, 255, 0.03)';
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, height);
            this.ctx.stroke();
        }
        
        // Horizontal lines
        let startGridY = startY % stepPx;
        for (let y = startGridY; y < height; y += stepPx) {
            this.ctx.beginPath();
            // Highlight axis center line
            this.ctx.strokeStyle = Math.abs(y - startY) < 1.0 ? 'rgba(0, 240, 255, 0.15)' : 'rgba(255, 255, 255, 0.03)';
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(width, y);
            this.ctx.stroke();
        }

        // Helper: Convert PDR meters (North=Y+, East=X+) to Canvas Canvas Pixels
        const toCanvasCoords = (pt) => {
            return {
                x: startX + pt.x * this.zoomFactor,
                y: startY - pt.y * this.zoomFactor // Invert Y because canvas Y-down, but PDR Y-up (North)
            };
        };

        // 2. Draw Vector Path Lines
        if (path.length > 1) {
            this.ctx.beginPath();
            const startPt = toCanvasCoords(path[0]);
            this.ctx.moveTo(startPt.x, startPt.y);
            
            for (let i = 1; i < path.length; i++) {
                const pt = toCanvasCoords(path[i]);
                this.ctx.lineTo(pt.x, pt.y);
            }
            
            // Path stroke gradient/style
            this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.85)';
            this.ctx.lineWidth = 3.5;
            this.ctx.lineCap = 'round';
            this.ctx.lineJoin = 'round';
            this.ctx.shadowBlur = 10;
            this.ctx.shadowColor = 'rgba(0, 240, 255, 0.35)';
            this.ctx.stroke();
            this.ctx.shadowBlur = 0; // reset
        }

        // 3. Draw Step Nodes/Dots
        path.forEach((pt, index) => {
            if (index === 0) return; // start point has custom drawing
            
            const px = toCanvasCoords(pt);
            
            this.ctx.beginPath();
            this.ctx.arc(px.x, px.y, 4.5, 0, 2 * Math.PI);
            this.ctx.fillStyle = '#bf5af2'; // Purple
            this.ctx.strokeStyle = '#080b11';
            this.ctx.lineWidth = 1;
            this.ctx.fill();
            this.ctx.stroke();
        });

        // 4. Draw Start Position Marker (Green)
        this.ctx.beginPath();
        this.ctx.arc(startX, startY, 7.5, 0, 2 * Math.PI);
        this.ctx.fillStyle = '#30d158'; // Success green
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 1.5;
        this.ctx.fill();
        this.ctx.stroke();
        
        // Start text
        this.ctx.font = 'bold 10px Outfit, sans-serif';
        this.ctx.fillStyle = '#30d158';
        this.ctx.textAlign = 'center';
        this.ctx.fillText('START (0,0)', startX, startY - 12);

        // 5. Draw Current Position / Orientation Arrow Pointer
        const curPx = toCanvasCoords(currentPos);
        
        this.ctx.save();
        this.ctx.translate(curPx.x, curPx.y);
        // Rotate canvas matching heading. Note canvas angle is standard clockwise: 0 rad = East (+X).
        // PDR heading is: 0 deg = North (+Y), 90 deg = East (+X). 
        // Therefore, Canvas Angle = PDR Heading (converted to rad) - Math.PI / 2
        const canvasAngle = (heading * Math.PI) / 180 - Math.PI / 2;
        this.ctx.rotate(canvasAngle);
        
        // Glowing halo effect for current position indicator
        this.ctx.shadowBlur = 15;
        this.ctx.shadowColor = 'rgba(0, 240, 255, 0.8)';
        
        // Direction Arrow
        this.ctx.beginPath();
        this.ctx.moveTo(12, 0); // nose tip
        this.ctx.lineTo(-8, -8); // left wing
        this.ctx.lineTo(-4, 0); // tail center indent
        this.ctx.lineTo(-8, 8); // right wing
        this.ctx.closePath();
        
        this.ctx.fillStyle = '#00f0ff'; // Neon Cyan
        this.ctx.strokeStyle = '#ffffff';
        this.ctx.lineWidth = 1.5;
        this.ctx.fill();
        this.ctx.stroke();
        this.ctx.restore();
    }
}

/**
 * LIGHTWEIGHT REAL-TIME SENSOR CHART
 * Custom canvas line graph plotting raw & filtered acceleration waveforms.
 */
class SensorChart {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        
        this.data = [];
        this.maxSamples = 120; // scrolling window of telemetry
        
        this.pulseTriggered = false;
        
        this.resize();
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        this.canvas.style.width = `${rect.width}px`;
        this.canvas.style.height = `${rect.height}px`;
        this.ctx.scale(dpr, dpr);
    }

    addData(raw, filtered, threshold) {
        this.data.push({ raw, filtered, threshold });
        
        if (this.data.length > this.maxSamples) {
            this.data.shift();
        }
        
        this.draw();
    }

    triggerStepMarker() {
        this.pulseTriggered = true;
    }

    clear() {
        this.data = [];
        this.draw();
    }

    draw() {
        const width = this.canvas.width / (window.devicePixelRatio || 1);
        const height = this.canvas.height / (window.devicePixelRatio || 1);
        
        this.ctx.fillStyle = '#0f121d';
        this.ctx.fillRect(0, 0, width, height);

        if (this.data.length < 2) return;

        // Scale values (acceleration goes typically from 6.0 to 16.0 m/s^2)
        // Set dynamic bounds for the chart viewport
        let minVal = 7.5;
        let maxVal = 13.5;
        
        // Expand chart bounds if signals go beyond defaults
        this.data.forEach(d => {
            minVal = Math.min(minVal, d.raw, d.filtered);
            maxVal = Math.max(maxVal, d.raw, d.filtered, d.threshold);
        });
        
        // Add safety padding
        const padding = 0.5;
        minVal -= padding;
        maxVal += padding;
        const scaleRange = maxVal - minVal;

        const getCanvasY = (val) => {
            return height - ((val - minVal) / scaleRange) * height;
        };

        // Draw horizontal grid lines on chart
        this.ctx.lineWidth = 1;
        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
        
        // Gravity baseline line (typically around 9.8)
        const gravityY = getCanvasY(9.81);
        this.ctx.beginPath();
        this.ctx.moveTo(0, gravityY);
        this.ctx.lineTo(width, gravityY);
        this.ctx.stroke();

        // 1. Draw Raw Acceleration (Dull/Background)
        this.ctx.beginPath();
        this.ctx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
        this.ctx.lineWidth = 1;
        
        for (let i = 0; i < this.data.length; i++) {
            const x = (i / (this.maxSamples - 1)) * width;
            const y = getCanvasY(this.data[i].raw);
            if (i === 0) this.ctx.moveTo(x, y);
            else this.ctx.lineTo(x, y);
        }
        this.ctx.stroke();

        // 2. Draw Step Threshold Line (Orange Dashed)
        this.ctx.beginPath();
        this.ctx.strokeStyle = 'rgba(255, 149, 0, 0.6)';
        this.ctx.lineWidth = 1;
        this.ctx.setLineDash([4, 4]);
        
        for (let i = 0; i < this.data.length; i++) {
            const x = (i / (this.maxSamples - 1)) * width;
            const y = getCanvasY(this.data[i].threshold);
            if (i === 0) this.ctx.moveTo(x, y);
            else this.ctx.lineTo(x, y);
        }
        this.ctx.stroke();
        this.ctx.setLineDash([]); // reset

        // 3. Draw Filtered Acceleration (Bright/Foreground)
        this.ctx.beginPath();
        this.ctx.strokeStyle = '#00f0ff';
        this.ctx.lineWidth = 2;
        
        for (let i = 0; i < this.data.length; i++) {
            const x = (i / (this.maxSamples - 1)) * width;
            const y = getCanvasY(this.data[i].filtered);
            if (i === 0) this.ctx.moveTo(x, y);
            else this.ctx.lineTo(x, y);
        }
        this.ctx.stroke();

        // 4. Draw Step detected indicator impulse (Purple flash)
        if (this.pulseTriggered) {
            this.ctx.fillStyle = 'rgba(191, 90, 242, 0.35)';
            this.ctx.fillRect(width - 25, 0, 25, height);
            this.pulseTriggered = false; // consume
        }
    }
}

// Instantiate App
window.addEventListener('DOMContentLoaded', () => {
    window.app = new PDRTracker();
});
