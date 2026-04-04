package main

// PIDController holds the state for a PID controller optimized for rate control.
//
// Unit System (for rate control with gyroscope feedback):
//   - Input: currentError in [rad/s] (desired_rate - measured_rate)
//   - Output: control signal in [rad/s] (mapped to servo control)
//   - Time step dt in [s]
//
// Gain Definitions:
//
//   - Kp (Proportional gain): [dimensionless]
//     Output contribution: Kp × error[rad/s] → [rad/s]
//     Typical range: 0.5-2.0
//
//   - Ki (Integral gain): [1/s]
//     Accumulates integrated error over time
//     Output contribution: Ki × integral(error in [rad]) → [rad/s]
//     Typical range: 0.01-0.5
//     Ki = Kp / Ki_divisor (tune Ki_divisor to 10-50×Kp for smooth response)
//
//   - Kd (Derivative gain): [dimensionless]
//     Responds to rate of error change
//     Output contribution: Kd × d(error)/dt[rad/s²] → [rad/s]
//     Typical range: 0.01-0.5
//     Kd ≈ 0.1-0.25 × Kp (adds damping without oscillation)
type PIDController struct {
	Kp, Ki, Kd float64 // Gains with units as documented above
	prevError  float64 // Previous error [rad/s] for derivative calculation
	integral   float64 // Accumulated integral [rad] over time
}

// NewPIDController creates and initializes a new PIDController.
func NewPIDController(Kp, Ki, Kd float64) *PIDController {
	return &PIDController{
		Kp: Kp,
		Ki: Ki,
		Kd: Kd,
	}
}

// Update calculates the new control output from PID terms.
//
// Input:
//
//	currentError: [rad/s] (desired_rate - measured_rate from gyro)
//	dt: [s] (time step since last update, typ. 0.005 at 200Hz)
//
// Output:
//
//	control signal: [rad/s] (servo control rate command)
//	Expected range: [-MAX_ROLL_RATE, MAX_ROLL_RATE]
//
// Unit Flow (for verification):
//
//	P = Kp [1/1] × currentError [rad/s] → [rad/s]
//	I = Ki [1/s] × integral [rad] → [rad/s]
//	D = Kd [1/1] × (error_rate [rad/s²] / dt⁻¹ [s⁻¹]) → [rad/s]

// For rate control with [rad/s] error:
// Kp should be dimensionless (typically 0.1-5.0)
// Ki should be [1/s] (typically 0.01-0.5)
// Kd should be dimensionless (typically 0.01-1.0)

func (pid *PIDController) Update(currentError, dt float64) float64 {
	// Proportional term: Kp × error[rad/s] → [rad/s]
	proportional := pid.Kp * currentError

	// Integral term: accumulate error, convert via Ki [1/s]
	// accumulation: currentError[rad/s] × dt[s] → [rad]
	// conversion: Ki[1/s] × integrated_error[rad] → [rad/s]
	pid.integral += currentError * dt
	integral := pid.Ki * pid.integral

	// Derivative term: Kd × rate_of_error_change [rad/s²] → [rad/s]
	// rate calculation: (error[rad/s] - prev_error[rad/s]) / dt[s] → [rad/s²]
	derivative := pid.Kd * (currentError - pid.prevError) / dt
	pid.prevError = currentError

	// Sum of all terms: all in [rad/s]
	output := proportional + integral + derivative

	return output
}

// Reset clears the integral and derivative state of the PID controller.
func (pid *PIDController) Reset() {
	pid.integral = 0
	pid.prevError = 0
}
