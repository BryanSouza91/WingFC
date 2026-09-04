package main

// PIDController holds the state for a PID controller.
//
// Unit System:
//   - Input: normalized error in [-1.0, 1.0] range
//   - Output: normalized control mix signal in [-1.0, 1.0] range
//   - Time step dt in seconds (e.g. 0.005 for 200Hz)
type PIDController struct {
	Kp, Ki, Kd float32 // Proportional, Integral, Derivative gains
	prevError  float32 // Previous error for derivative calculation
	integral   float32 // Accumulated integral with anti-windup
}

// NewPIDController creates and initializes a new PIDController.
func NewPIDController(Kp, Ki, Kd float32) *PIDController {
	return &PIDController{
		Kp: Kp,
		Ki: Ki,
		Kd: Kd,
	}
}

// Update calculates the new control output from PID terms with anti-windup.
func (pid *PIDController) Update(currentError, dt float32) float32 {
	// Proportional term
	proportional := pid.Kp * currentError

	// Integral term with anti-windup clamping to [-1.0, 1.0]
	pid.integral = constrain(pid.integral+currentError*dt, -1.0, 1.0)
	integral := pid.Ki * pid.integral

	// Derivative term
	var derivative float32
	if dt > 0 {
		derivative = pid.Kd * (currentError - pid.prevError) / dt
	}
	pid.prevError = currentError

	// Sum of all terms
	output := proportional + integral + derivative

	return output
}

// Reset clears the integral and derivative state of the PID controller.
func (pid *PIDController) Reset() {
	pid.integral = 0
	pid.prevError = 0
}
