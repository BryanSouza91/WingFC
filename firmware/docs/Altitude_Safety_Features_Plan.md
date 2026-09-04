# Barometer Implementation & Safety Features Plan

Expanding the custom PCB to include a barometer alongside the compass and power regulation is an excellent step. A barometer (like the BMP280, BMP388, or MS5611) measures atmospheric pressure to calculate **Altitude** and **Vertical Velocity (Climb/Sink rate)**. 

Integrating altitude awareness allows WingFC to graduate from basic stabilization to advanced, autonomous safety features.

---

## 1. Planned Safety Features

### Failsafe Altitude Hold (Safe Loiter)
Right now, if the receiver loses signal, WingFC enters the `FAILSAFE` state which centers the servos and cuts the throttle, resulting in a glide to a crash.
* **Feature:** With a barometer, when `FAILSAFE` is triggered, WingFC can level the wings, apply a cruising throttle, and **hold a specific safe altitude** using elevator control. The plane will loiter safely, giving the pilot time to run toward it and regain radio signal.

### "Hard Deck" (Ground Collision Avoidance)
When flying Line-of-Sight (LOS) or FPV, it is easy to lose track of altitude during aggressive maneuvers or dives.
* **Feature:** Program a "Hard Deck" limit. If the barometer detects the plane dropping below a pre-set altitude (e.g., 20 meters), WingFC temporarily overrides the pilot's pitch input to pull up and automatically increases throttle to prevent a crash.

### Legal Ceiling Enforcement (Geofencing)
Aviation authorities (FAA, EASA) limit RC aircraft to a maximum altitude of 400 feet (~120 meters) AGL.
* **Feature:** WingFC can enforce a maximum ceiling. As the plane approaches 120 meters, the flight controller smoothly pitches the nose down or limits throttle to ensure the aircraft never violates airspace regulations.

### Stall & Freefall Detection
A fixed-wing aircraft stalls when it loses airspeed and aerodynamic lift, resulting in a sudden drop.
* **Feature:** If the barometer detects a rapid, uncommanded descent (high negative vertical velocity) while the IMU reports the plane is pitched level or up, the flight controller knows the plane has stalled. It can automatically initiate a stall-recovery maneuver: dropping the nose to regain airspeed and applying full throttle.

### Auto-Launch & Auto-Land Assistance
* **Auto-Launch:** Arm the plane and throw it. The barometer detects the sudden increase in altitude (and accelerometer detects the forward throw), automatically spinning up the motor *after* it leaves the pilot's hand, keeping fingers safe from the prop.
* **Landing Assist:** On approach, the barometer enforces a maximum safe sink rate (e.g., no faster than -1.5 m/s). If the pilot tries to land too steeply, WingFC softly flares the elevator to prevent a hard landing.

---

## 2. Technical Implementation Roadmap

To make this work seamlessly, the raw barometer data must be fused with the existing accelerometer data to prevent noise and latency.

### Phase 1: Hardware & Drivers
* Attach the barometer to the existing `machine.I2C0` bus used by the LSM6DS3TR and the compass.
* Integrate a TinyGo driver for the specific barometer chip (e.g., `tinygo.org/x/drivers/bmp280`).
* Add initialization logic to `hardware.go`.

### Phase 2: Relative Altitude Calculation
* Modify the `IMU_CALIBRATION` state in `main.go` to sample the barometer for several seconds to establish a baseline Ground Level Pressure.
* Add a function to convert current pressure readings against the baseline into relative "Altitude Above Ground Level" (AGL) in meters.

### Phase 3: Sensor Fusion (Vertical Kalman Filter)
* Create a dedicated Vertical Kalman Filter (or expand the existing one).
* **State Vector:** `[Altitude, Vertical_Velocity, Accel_Bias]`
* **Prediction:** Use the IMU's Z-axis accelerometer (rotated to the global frame using pitch/roll) to predict changes in altitude and velocity at 200Hz.
* **Update:** Use the slower, noisy barometer data (e.g., 50Hz) to correct the accelerometer drift.
* *Result:* A highly responsive, drift-free vertical speed indicator (VSI) and altitude estimate.

### Phase 4: Altitude PID Control
* Add an `altitudePID *PIDController`.
* **Outer Loop (Altitude):** Calculates the error between *desired altitude* and *current altitude*, outputting a desired Vertical Velocity (Climb/Sink rate).
* **Inner Loop (Velocity):** Compares desired Vertical Velocity against the Kalman Filter's current Vertical Velocity, outputting a desired pitch angle to the existing Pitch PID loop, and a throttle command.

### Phase 5: State Machine Integration
* Add new flight states or modifiers in `main.go` to trigger the safety features (e.g., changing the `FAILSAFE` logic to engage Altitude Hold rather than zeroing outputs).