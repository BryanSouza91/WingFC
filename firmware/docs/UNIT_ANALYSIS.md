# WingFC Unit Analysis & Consistency Audit

## Current State: Units in the Control Loop

### 1. **Raw Sensor Data → Base Units**

| Data | Raw Format | Conversion | Base Unit | Location |
|------|-----------|-----------|-----------|----------|
| **Accelerometer** | Raw ADC counts | × `microGToMS2` = × 9.80665e-6 | **[m/s²]** | `helpers.go` line 19-21 |
| **Gyroscope** | Raw ADC counts | × `microDPSToRadS` = × π/(180e6) | **[rad/s]** | `helpers.go` line 22-24 |

✅ **Conversion correct:** LSM6DS3 outputs microG and microDPS; conversions are mathematically valid.

---

### 2. **Attitude Estimation from Accelerometer**

```go
// imu.go lines 21-27
Pitch = atan2(-AccelX, sqrt(AccelY² + AccelZ²))  // → [radians]
Roll = atan2(AccelY, AccelZ)                     // → [radians]
```

| Data | Input | Output | Notes |
|------|-------|--------|-------|
| **Pitch** | AccelX [m/s²] | [radians] | atan2 always outputs [radians] |
| **Roll** | AccelY, AccelZ [m/s²] | [radians] | atan2 always outputs [radians] |

✅ **Correct:** Acceleration vectors → angle via trigonometry produces radians.

---

### 3. **Desired Rate from RC Receiver**

```go
// main.go lines 210-211
desiredPitchRate = mapRange(RC_input, MIN_RX_VALUE, MAX_RX_VALUE, 
                             -MAX_PITCH_RATE, MAX_PITCH_RATE)
desiredRollRate = mapRange(RC_input, MIN_RX_VALUE, MAX_RX_VALUE, 
                            -MAX_ROLL_RATE, MAX_ROLL_RATE)
```

Where:
```go
// main.go lines 57-58
MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * math.Pi / 180  // [rad/s]
MAX_ROLL_RATE = MAX_ROLL_RATE_DEG * math.Pi / 180   // [rad/s]
```

| Data | Output | Units | Notes |
|------|--------|-------|-------|
| **Desired Pitch Rate** | [-MAX_PITCH_RATE, MAX_PITCH_RATE] | **[rad/s]** | ✓ Correct |
| **Desired Roll Rate** | [-MAX_ROLL_RATE, MAX_ROLL_RATE] | **[rad/s]** | ✓ Correct |

✅ **Correct:** RC input scales to rate setpoint in radians/second.

---

### 4. **Kalman Filter Input/Output**

```go
// main.go lines 207-208
kf.Predict(imuData.GyroX, imuData.GyroY)        // Both [rad/s] ✓
kf.Update(imuData.Pitch, imuData.Roll)          // Both [radians] ✓
```

**Kalman State:**
```go
// kalman.go
X *Matrix2x1  // [pitch_angle, roll_angle] in [radians]
```

**Predict Step:**
```go
// kalman.go Predict()
gyroVector.Set(0, 0, gyroY * kf.dt)  // [rad/s] × [s] = [rad] ✓
gyroVector.Set(1, 0, gyroX * kf.dt)  // [rad/s] × [s] = [rad] ✓
```

**Update Step:**
```go
// kalman.go Update()
z.Set(0, 0, accelPitch)  // [radians] ✓
z.Set(1, 0, accelRoll)   // [radians] ✓
```

✅ **Correct:** Kalman filter consistently uses [radians] for angles and [rad/s] for rates.

---

### 5. **PID Controller - UNIT MISMATCH DETECTED ⚠️**

```go
// main.go lines 223-224
pitchError := desiredPitchRate - imuData.GyroY  // [rad/s] - [rad/s] = [rad/s] ✓
rollError := desiredRollRate - imuData.GyroX    // [rad/s] - [rad/s] = [rad/s] ✓

// main.go lines 227-228
pitchOutput := pitchPID.Update(pitchError, dt) * PID_WEIGHT
rollOutput := rollPID.Update(rollError, dt) * PID_WEIGHT

// pid.go lines 21-31
proportional := pid.Kp * currentError           // [Kp_units] × [rad/s] = ???
integral := pid.Ki * (currentError * dt)        // [Ki_units] × [rad] = ???
derivative := pid.Kd * (currentError / dt)      // [Kd_units] × [rad/s²] = ???
```

**PROBLEM:** The PID gain units (Kp, Ki, Kd) are **not documented** and their relationship to the output is **ambiguous**.

---

### 6. **Control Output to PWM - CRITICAL MISMATCH ⚠️⚠️⚠️**

```go
// main.go lines 234-235
leftElevon = mapRange(float64(leftElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, 
                      MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
```

**BUG:** This assumes:
```
PID_output ∈ [-MAX_ROLL_RATE, MAX_ROLL_RATE]
            ∈ [rad/s]
```

But what units is `pitchOutput` actually in? **Not defined!**

If Kp = 1.0 (dimensionless), then:
```
pitchOutput = 1.0 × [rad/s] = [rad/s]  ✓ Would work
```

But if Kp = 0.5, then:
```
pitchOutput = 0.5 × [rad/s] = 0.5 × [rad/s]  (what does this mean?)
```

---

## Summary: Unit Consistency Issues

| Component | Status | Issue |
|-----------|--------|-------|
| Sensor → SI units | ✅ OK | Correct conversions, well-defined |
| Accelerometer → Angles | ✅ OK | Trigonometry → radians (correct) |
| RC → Desired Rates | ✅ OK | Maps to [rad/s] (correct) |
| Kalman Filter | ✅ OK | Consistent: [radians] for angles, [rad/s] for rates |
| PID Error Input | ✅ OK | [rad/s] - [rad/s] = [rad/s] (correct) |
| **PID Gain Units** | ❌ **UNDOCUMENTED** | Kp, Ki, Kd units unknown |
| **PID Output Units** | ❌ **AMBIGUOUS** | Should be [rad/s] but not enforced |
| **Output Mapping** | ⚠️ **DEPENDS ON PID** | Assumes pidOutput ∈ [rad/s] |

---

## Recommendations

### 1. **Document PID Gain Units (Required)**

Add comments to PID controller:
```go
type PIDController struct {
	// Kp: Proportional gain [dimensionless]
	// Output units: [rad/s] when error is [rad/s]
	// Example: error [rad/s] × Kp=1.0 → output [rad/s]
	Kp float64
	
	// Ki: Integral gain [1/s]
	// Converts integrated error [rad] to [rad/s]
	// Example: 100 rad × Ki=0.01 [1/s] → 1.0 [rad/s]
	Ki float64
	
	// Kd: Derivative gain [dimensionless]
	// Converts rate of error [rad/s²] to [rad/s]
	// Example: 10 [rad/s²] × Kd=0.1 → 1.0 [rad/s]
	Kd float64
}
```

### 2. **Add Type-Safe Unit Wrapper (Optional, More Work)**

```go
type RadiansPerSecond float64
type Radians float64

type PIDController struct {
	Kp, Ki, Kd float64
	prevError  RadiansPerSecond
	integral   Radians
}

func (pid *PIDController) Update(currentError RadiansPerSecond, dt float64) RadiansPerSecond {
	// Compiler enforces correct units
	proportional := RadiansPerSecond(pid.Kp) * currentError
	// ...
}
```

### 3. **Add Validation Function**

```go
// Check PID gains are reasonable
func ValidatePIDGains(kp, ki, kd float64) error {
	// For rate control with [rad/s] error:
	// Kp should be dimensionless (typically 0.1-5.0)
	// Ki should be [1/s] (typically 0.01-0.5)
	// Kd should be dimensionless (typically 0.01-1.0)
	
	if kp < 0 || ki < 0 || kd < 0 {
		return fmt.Errorf("PID gains must be positive")
	}
	if kp > 10 || ki > 5 || kd > 5 {
		return fmt.Errorf("PID gains seem excessive, check units")
	}
	return nil
}
```

### 4. **Add Debug Output with Units**

```go
// main.go - for debugging
if loopCount%100 == 0 {
	println("[UNITS_DEBUG]")
	println("  DesiredRate [rad/s]:", desiredPitchRate)
	println("  GyroRate [rad/s]:", imuData.GyroY)
	println("  Error [rad/s]:", pitchError)
	println("  PIDOutput [???]:", pitchOutput)  // WHAT UNITS??
	println("  MapRange in: [-MAX_ROLL_RATE:", -MAX_ROLL_RATE, "]")
}
```

---

## Next Steps

1. **Immediate:** Add documentation comments specifying Kp/Ki/Kd units
2. **Short-term:** Add validation function
3. **Long-term:** Consider type wrappers for compile-time unit safety

Would you like me to implement these fixes?
