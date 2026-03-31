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
}

var ledController LEDController

var redLED = machine.LED_RED
var greenLED = machine.LED_GREEN
var blueLED = machine.LED_BLUE

func (l *LEDController) updateLED() {
	switch l.state {
	case LEDOFF:
		redLED.High()
		greenLED.High()
		blueLED.High()
	case PWMCONFIG: // R
		redLED.Low()
		greenLED.High()
		blueLED.High()
	case PWMERROR: // RG
		redLED.Low()
		greenLED.Low()
		blueLED.High()
	case SERVOINIT: // G
		redLED.High()
		greenLED.Low()
		blueLED.High()
	case SERVOERROR: // GB
		redLED.High()
		greenLED.Low()
		blueLED.Low()
	case ESCINIT: // R
		redLED.Low()
		greenLED.High()
		blueLED.High()
	case ESCERROR: // RGB
		redLED.Low()
		greenLED.Low()
		blueLED.Low()
	case IMUCONFIG: // B
		redLED.High()
		greenLED.High()
		blueLED.Low()
	case IMUINIT: // R
		redLED.Low()
		greenLED.High()
		blueLED.High()
	case IMUERROR: // RB
		redLED.Low()
		greenLED.High()
		blueLED.Low()
	case CALIBRATE: // RGB
		redLED.Low()
		greenLED.Low()
		blueLED.Low()
	case DISARMED: // G
		redLED.High()
		greenLED.Low()
		blueLED.High()
	case ARMED: // B
		redLED.High()
		greenLED.High()
		blueLED.Low()
	case FAILSAFED: // R
		redLED.Low()
		greenLED.High()
		blueLED.High()
	}
}
