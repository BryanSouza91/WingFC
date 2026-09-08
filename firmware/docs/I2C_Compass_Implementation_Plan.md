# I2C Compass Implementation Plan

Adding an I2C compass (magnetometer) to the yaw control in WingFC addresses the drift that naturally happens when estimating yaw purely from a gyroscope. 

Currently, the `KalmanFilter` only corrects `pitch` and `roll` using the accelerometer, while `yaw` relies strictly on uncorrected gyro integration (which drifts over time). To use an I2C compass for stabilized yaw/heading control, it needs to be integrated into the hardware configuration, the IMU processing pipeline, the Kalman Filter, and finally the PID loop.

Here is the step-by-step strategy for how this can be implemented in the codebase:

### 1. Hardware Initialization (`hardware.go`)
Attach the compass (e.g., QMC5883L or HMC5883L) to the external pins that are on the `machine.I2C1` bus, and initialize it using a TinyGo driver. The external pins are on a different I2C bus, so for redundancy and safety, we will use `machine.I2C1` for the compass. Update the `FC_Hardware` struct to include a pointer to the compass device.

```go
import "tinygo.org/x/drivers/qmc5883l" // or hmc5883l, depending on your chip

type FC_Hardware struct {
    // ... existing fields ...
    Compass *qmc5883l.Device
}

func initCompass(hw *FC_Hardware) error {
    // You can initialize the device using the raw *machine.I2C instance
    compass := qmc5883l.New(machine.I2C1) 
    if err := compass.Configure(); err != nil {
        return fmt.Errorf("Compass configure failed: %w", err)
    }
    hw.Compass = compass
    return nil
}
```
*Note: Make sure to call `initCompass(hw)` inside `InitHardware()`.*

### 2. Tilt-Compensated Heading (`imu.go` & `helpers.go`)
An accelerometer measures gravity to find pitch and roll, but a magnetometer measures the earth's magnetic field. If the aircraft tilts, the raw compass reading skews. You must compute a **tilt-compensated heading** using the pitch/roll values you already have.

In `imu.go`, add magnetometer variables to the `IMU` struct:
```go
type IMU struct {
    // ...
    MagX float32
    MagY float32
    MagZ float32
}

func (i *IMU) yawMag() float32 {
    // Tilt compensation formula mapping Earth's magnetic vectors back to the flat horizontal plane
    // (Signs and axis layout depend on your specific chip's orientation)
    cosPitch := math.Cos(i.Pitch)
    sinPitch := math.Sin(i.Pitch)
    cosRoll := math.Cos(i.Roll)
    sinRoll := math.Sin(i.Roll)

    Xh := i.MagX * cosPitch + i.MagZ * sinPitch
    Yh := i.MagX * sinRoll * sinPitch + i.MagY * cosRoll - i.MagZ * sinRoll * cosPitch
    
    return math.Atan2(Yh, Xh)
}
```
In `helpers.go`'s `readIMUData()` and `processIMUData()`, you'll poll the compass over I2C, convert the raw axes, and apply calibration (hard-iron/soft-iron biases) just like you do with `Accel` and `Gyro`.

### 3. Upgrading the Kalman Filter (`kalman.go`)
Currently, your `KalmanFilter` uses a 2D measurement vector (Z = [pitch, roll]) and matrices like `Matrix2x2` and `Matrix2x3`. Thankfully, the `matrix.go` library already contains full support for 3x3 operations (like `Inverse3x3()` and `Multiply3x3()`).

You'll elevate the Kalman observation to a 3-dimensional vector (Z = [pitch, roll, yaw]).

```go
func NewKalmanFilter(dt float32) *KalmanFilter {
    // ... 
    // Measurement noise covariance (R) now needs to include the magnetometer's variance
    r := Identity3x3()
    r.Set(0, 0, 0.5) // Pitch Accel noise
    r.Set(1, 1, 0.5) // Roll Accel noise
    r.Set(2, 2, 2.0) // Yaw Mag noise (Compass data is often noisier)

    return &KalmanFilter{
        X: NewMatrix3x1(),
        // H maps [pitch, roll, yaw] states to [pitch_accel, roll_accel, yaw_mag] measurements
        H: Identity3x3(), 
        R: r,
        // ...
    }
}

// Expand Update signature to accept the new yawMag parameter
func (kf *KalmanFilter) Update(accelPitch, accelRoll, magYaw float32) {
    z := NewMatrix3x1()
    z.Set(0, 0, accelPitch)
    z.Set(1, 0, accelRoll)
    z.Set(2, 0, magYaw)

    // Your existing S, K, and P math easily transitions here
    // as matrix.go natively supports 3x3 matrix multiplication and inverses.
}
```

### 4. Cascade PID Loops for Heading Control (`main.go` & `pid.go`)
Right now, the RC loop commands a *Yaw Rate*, and the PID attempts to achieve that target.

If you are using an absolute heading from a compass, your system transitions from "rate control" to "position/heading control." This is usually done with a cascaded PID loop:
1. **Outer Loop (Heading)**: Takes the difference between the desired compass heading and the Kalman Filter's absolute yaw state, outputting a desired *rate of turn*.
2. **Inner Loop (Rate)**: Takes the desired *rate of turn* from the outer loop and compares it against `imuData.GyroZ`, outputting the final servo/motor `yawMix`.

```go
// Example logic in FLIGHT_MODE inside main.go:

kf.Predict(imuData.GyroX, imuData.GyroY, imuData.GyroZ)
kf.Update(imuData.Pitch, imuData.Roll, imuData.yawMag())

// 1. Calculate Absolute Heading Error (accounting for 360-degree wrap-around)
headingError := desiredHeading - kf.X.At(2, 0) 

// 2. Outer Loop: Calculate desired yaw rate to correct the heading
desiredYawRate := headingPID.Update(headingError, dt)

// 3. Inner Loop: Execute standard rate control on the desired rate
rateError := desiredYawRate - imuData.GyroZ
yawMix = ratePID.Update(rateError, dt) * PID_WEIGHT
```

By fusing the compass into the Kalman Filter, the gyro will continue to handle the fast, high-frequency rotational movements of the plane smoothly, while the magnetometer will provide the low-frequency absolute reference to correct drift and keep the aircraft on a solid, straight heading.
