# **WingFC Math Optimization Report**

**Target Hardware:** Seeed Studio XIAO nRF52840 Sense / Sense Plus (ARM Cortex-M4F)

## **Executive Summary**

The nRF52840 microcontroller features an ARM Cortex-M4F processor. The "F" denotes a hardware Floating-Point Unit (FPU), but crucially, **it only supports single-precision (float32) natively**.

Because Go defaults to float64, your current implementation forces the TinyGo compiler to inject software emulation for every float64 operation. This software emulation can take 10x to 50x more clock cycles than native instructions. By transitioning to float32 and fixed-point integer math, you will dramatically reduce control loop execution time, minimize latency, and free up cycles for other tasks like telemetry, logging, or LED management.

## ---

**Phase 1: The Single-Precision Downgrade (Highest ROI)**

The most immediate performance gain requires no algorithmic changes. We simply stop the hardware from emulating double-precision math.

### **1\. Update Global Variables**

In main.go, redefine all floating-point globals explicitly as float32:

Go

// main.go  
var (  
    // Force float32 for dt  
    dt float32 \= 0.005 

    // IMU calibration \- explicit float32  
    accelXSum, accelYSum, accelZSum, accelBiasX, accelBiasY, accelBiasZ float32 \= 0\., 0\., 0\., 0\., 0\., 0\.  
    gyroXSum, gyroYSum, gyroZSum, gyroBiasX, gyroBiasY, gyroBiasZ       float32 \= 0\., 0\., 0\., 0\., 0\., 0\.  
    desiredPitchRate, desiredRollRate                                   float32  
      
    leftServoSubtrim  float32 \= LEFT\_SERVO\_SUBTRIM  
    rightServoSubtrim float32 \= RIGHT\_SERVO\_SUBTRIM  
)

### **2\. Update Constants**

In main.go, explicitly cast your math constants to float32 so they don't coerce your variables back to float64 during operations:

Go

const (  
    microGToMS2    float32 \= 9.80665 / 1e6  
    microDPSToRadS float32 \= math.Pi / (180 \* 1e6)  
    MAX\_ROLL\_RATE  float32 \= MAX\_ROLL\_RATE\_DEG \* math.Pi / 180  
    MAX\_PITCH\_RATE float32 \= MAX\_PITCH\_RATE\_DEG \* math.Pi / 180  
)

### **3\. Update the PID Controller Struct**

In pid.go, downgrade the internal state to float32:

Go

type PIDController struct {  
    Kp, Ki, Kd float32  
    prevError  float32  
    integral   float32  
}

func NewPIDController(Kp, Ki, Kd float32) \*PIDController {  
    return \&PIDController{Kp: Kp, Ki: Ki, Kd: Kd}  
}

func (pid \*PIDController) Update(currentError, dt float32) float32 {  
    // Math remains exactly the same, but now runs natively on the FPU  
    // ...  
}

## ---

**Phase 2: Integer-Based Low-Pass Filter**

Floating-point multiplication in the IMU filter (helpers.go) is costly. Since the LSM6DS3TR gives you raw int16 values, you can filter them directly using fast bitwise shifts before doing any unit conversions.

### **The Bitwise LPF**

Instead of LPF\_ALPHA \= 0.2, we can use an alpha of 0.25 ($1/4$). Dividing by 4 is the same as a bitwise right-shift by 2 (\>\> 2), which executes in a single clock cycle.

Go

// In helpers.go or main.go, track raw integers instead of floats  
var (  
    filteredAccelX, filteredAccelY, filteredAccelZ int32  
    filteredGyroX, filteredGyroY, filteredGyroZ int32  
)

func readLSMData() {  
    rawAccelX, rawAccelY, rawAccelZ, \_ := hw.IMU.ReadAccel()  
    rawGyroX, rawGyroY, rawGyroZ, \_ := hw.IMU.ReadGyro()

    // Fast Integer LPF (Alpha \~0.25)  
    filteredAccelX \= filteredAccelX \- (filteredAccelX \>\> 2) \+ (int32(rawAccelX) \>\> 2)  
    filteredAccelY \= filteredAccelY \- (filteredAccelY \>\> 2) \+ (int32(rawAccelY) \>\> 2)  
    filteredAccelZ \= filteredAccelZ \- (filteredAccelZ \>\> 2) \+ (int32(rawAccelZ) \>\> 2)  
      
    filteredGyroX \= filteredGyroX \- (filteredGyroX \>\> 2) \+ (int32(rawGyroX) \>\> 2)  
    filteredGyroY \= filteredGyroY \- (filteredGyroY \>\> 2) \+ (int32(rawGyroY) \>\> 2)  
    filteredGyroZ \= filteredGyroZ \- (filteredGyroZ \>\> 2) \+ (int32(rawGyroZ) \>\> 2)  
      
    // Only convert to float32 at the very end when feeding the Kalman filter or PID  
    imuData.AccelX \= float32(filteredAccelX) \* microGToMS2  
    // ... etc  
}

## ---

**Phase 3: Optimizing RC Input Mapping**

Currently, mapRange takes float64 inputs. Since your RC channels come in as uint16 (988 to 2012), mapping them to control targets shouldn't require floating-point math.

### **Integer Mapping Function**

Create a dedicated mapRangeInt in helpers.go:

Go

// Pure integer math: no FPU required  
func mapRangeInt(value, fromMin, fromMax, toMin, toMax int32) int32 {  
    return (value \- fromMin) \* (toMax \- toMin) / (fromMax \- fromMin) \+ toMin  
}

When you calculate desiredPitchRate and desiredRollRate in main.go, do it in integer space first (e.g., mapping 1000-2000 to \-200 to \+200 degrees/sec), and only convert to float32 radians per second when subtracting from the gyro error.

## ---

**Phase 4: Fixed-Point PID Implementation (Optional, Advanced)**

If you want to squeeze out every last cycle, you can replace the FPU entirely in your control loop by scaling your PID parameters and doing everything in int32.

Instead of Kp \= 1.0, you use Kp \= 1000\. You perform your math, and then divide the final output by 1000\.

Go

const PID\_SCALE int32 \= 1024 // 1024 is better than 1000 because division is a bitshift (\>\> 10\)

type IntPIDController struct {  
    Kp, Ki, Kd int32   
    prevError  int32   
    integral   int32   
}

// dt is now passed as milliseconds (e.g., 5 for 200Hz) instead of 0.005 seconds  
func (pid \*IntPIDController) Update(currentError int32, dtMs int32) int32 {  
    proportional := (pid.Kp \* currentError) \>\> 10

    pid.integral \+= currentError \* dtMs  
    integral := (pid.Ki \* pid.integral) \>\> 10

    derivative := (pid.Kd \* (currentError \- pid.prevError) / dtMs) \>\> 10  
    pid.prevError \= currentError

    return proportional \+ integral \+ derivative  
}

*Note: While highly efficient, tuning a fixed-point PID requires you to re-evaluate your base PID numbers. Only implement this if float32 proves too slow (unlikely on the M4F).*

## **Summary of Recommendations**

1. **Mandatory:** Change all float64 types and constants to float32. The M4F handles single-precision natively, providing an immediate speedup.  
2. **Highly Recommended:** Implement the integer-based LPF using bitwise shifts. It's computationally "free."  
3. **Highly Recommended:** Strip floats out of the RC channel mapRange functions.  
4. **Evaluate Later:** Fixed-point PID. The M4F's hardware FPU should be fast enough to handle float32 PID loops comfortably at 200Hz, making full fixed-point math optional rather than strictly necessary.