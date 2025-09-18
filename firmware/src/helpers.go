package main

import "golang.org/x/exp/constraints"

// Helper function to constrain a value within min and max bounds.
func constrain(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// Helper function to map a value from one range to another.
func mapRange[T constraints.Float](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// Helper function to set servo PWM outputs using global variables.
func setServoPWM(leftPulse, rightPulse uint32) {
	// Set the PWM duty cycle for left elevon (CH1)
	pwm0.Set(pwmCh1, leftPulse)
	// Set the PWM duty cycle for right elevon (CH2)
	pwm0.Set(pwmCh2, rightPulse)
}

// Helper function to set the ESC's PWM output using a global variable.
func setESC(pulseWidth uint32) {
	// The ESC's channel is typically CH3
	pwm1.Set(pwmCh3, pulseWidth)
}
