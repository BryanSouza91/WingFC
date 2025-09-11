package main

// PIDController holds the state for a PID controller.
type PIDController struct {
	Kp, Ki, Kd float64
	prevError  float64
	integral   float64
}

// NewPIDController creates and initializes a new PIDController.
func NewPIDController(Kp, Ki, Kd float64) *PIDController {
	return &PIDController{
		Kp: Kp,
		Ki: Ki,
		Kd: Kd,
	}
}

// Update calculates the new control output.
func (pid *PIDController) Update(currentError, dt float64) float64 {
	// Proportional term
	proportional := pid.Kp * currentError

	// Integral term
	pid.integral += currentError * dt
	integral := pid.Ki * pid.integral

	// Derivative term
	derivative := pid.Kd * (currentError - pid.prevError) / dt
	pid.prevError = currentError

	// Sum of all terms
	output := proportional + integral + derivative

	return output
}
