package main

import "machine"

type LEDState int

const (
	LEDOFF LEDState = iota
	PWMCONFIG
	PWMERROR
	SERVOINIT
	SERVOERROR
	ESCINIT
	ESCERROR
	IMUCONFIG
	IMUINIT
	IMUERROR
	CALIBRATE
	DISARMED
	ARMED
	FAILSAFED
)

// LEDController handles the onboard RGB LED
type LEDController struct {
	state LEDState
	red   machine.Pin
	green machine.Pin
	blue  machine.Pin
}

// NewLEDController creates a new LED controller with the given pins.
func NewLEDController() *LEDController {
	// Configure and wire up hardware with dependency injection
	redLED := machine.LED_RED
	greenLED := machine.LED_GREEN
	blueLED := machine.LED_BLUE

	// Initialize LED pins
	redLED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	greenLED.Configure(machine.PinConfig{Mode: machine.PinOutput})
	blueLED.Configure(machine.PinConfig{Mode: machine.PinOutput})

	return &LEDController{
		state: LEDOFF,
		red:   redLED,
		green: greenLED,
		blue:  blueLED,
	}
}

// SetState updates the LED state.
func (l *LEDController) SetState(state LEDState) {
	l.state = state
}

// Update applies the current LED state.
func (l *LEDController) Update() {
	l.updateLED(l.state)
}

// updateLED sets the LED pins based on the current state.
func (l *LEDController) updateLED(condition LEDState) {
	// Update LEDState with condition
	l.state = condition

	// activate appropriate LED sequence
	switch l.state {
	case LEDOFF:
		l.red.High()
		l.green.High()
		l.blue.High()
	case PWMCONFIG: // R
		l.red.Low()
		l.green.High()
		l.blue.High()
	case PWMERROR: // RG
		l.red.Low()
		l.green.Low()
		l.blue.High()
	case SERVOINIT: // G
		l.red.High()
		l.green.Low()
		l.blue.High()
	case SERVOERROR: // GB
		l.red.High()
		l.green.Low()
		l.blue.Low()
	case ESCINIT: // R
		l.red.Low()
		l.green.High()
		l.blue.High()
	case ESCERROR: // RGB
		l.red.Low()
		l.green.Low()
		l.blue.Low()
	case IMUCONFIG: // B
		l.red.High()
		l.green.High()
		l.blue.Low()
	case IMUINIT: // R
		l.red.Low()
		l.green.High()
		l.blue.High()
	case IMUERROR: // RB
		l.red.Low()
		l.green.High()
		l.blue.Low()
	case CALIBRATE: // RGB
		l.red.Low()
		l.green.Low()
		l.blue.Low()
	case DISARMED: // G
		l.red.High()
		l.green.Low()
		l.blue.High()
	case ARMED: // B
		l.red.High()
		l.green.High()
		l.blue.Low()
	case FAILSAFED: // R
		l.red.Low()
		l.green.High()
		l.blue.High()
	}
}
