package main

/*
initialize the LEDs first then slowly pulse the red led throughout the initialization,
upon completion of initialzaiton blink red,blue,then green with a twice as long duration on green,
then for calibration blink 3 times as a countdown then blink quickly while calibrating and then flash green for success
then we should have a alternating red then green light in the waiting state. flight mode should be solid green and failsafe should be rapid red flash
*/

import (
	"machine"
	"time"
)

// Define LED patterns
const (
	LED_OFF       = 0
	LED_ON        = 1
	LED_SLOWFLASH = 2
	LED_FASTFLASH = 3
	LED_FLASH     = 4
	LED_ALTERNATE = 5
	LED_BLINK3    = 6
)

// LED state struct
type ledState struct {
	pin         machine.Pin
	state       int
	lastToggle  time.Time
	onDuration  time.Duration
	offDuration time.Duration
	isOn        bool
	pulseValue  float64
	pulseDir    bool
}

// Function to initialize LED state
func newLEDState(pin machine.Pin) *ledState {
	pin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	return &ledState{
		pin:         pin,
		state:       LED_OFF,
		lastToggle:  time.Now(),
		onDuration:  0,
		offDuration: 0,
		isOn:        false,
		pulseValue:  0,
		pulseDir:    true,
	}
}

// Function to update LED state
func (ls *ledState) update() {
	now := time.Now()
	switch ls.state {
	case LED_OFF:
		ls.pin.Low()
		ls.isOn = false
	case LED_ON:
		ls.pin.High()
		ls.isOn = true
	case LED_SLOWFLASH:
		ls.onDuration = 250 * time.Millisecond
		ls.offDuration = 250 * time.Millisecond
		if now.Sub(ls.lastToggle) >= ls.onDuration {
			if ls.isOn {
				ls.pin.Low()
			} else {
				ls.pin.High()
			}
			ls.isOn = !ls.isOn
			ls.lastToggle = now
		}
	case LED_FASTFLASH:
		ls.onDuration = 50 * time.Millisecond
		ls.offDuration = 50 * time.Millisecond
		if now.Sub(ls.lastToggle) >= ls.onDuration {
			if ls.isOn {
				ls.pin.Low()
			} else {
				ls.pin.High()
			}
			ls.isOn = !ls.isOn
			ls.lastToggle = now
		}
	case LED_FLASH:
		ls.onDuration = 150 * time.Millisecond
		ls.offDuration = 150 * time.Millisecond
		if now.Sub(ls.lastToggle) >= ls.onDuration {
			if ls.isOn {
				ls.pin.Low()
			} else {
				ls.pin.High()
			}
			ls.isOn = !ls.isOn
			ls.lastToggle = now
		}
	case LED_ALTERNATE:
		ls.onDuration = 500 * time.Millisecond
		ls.offDuration = 500 * time.Millisecond
		if now.Sub(ls.lastToggle) >= ls.onDuration {
			if ls.isOn {
				ls.pin.Low()
			} else {
				ls.pin.High()
			}
			ls.isOn = !ls.isOn
			ls.lastToggle = now
		}
	case LED_BLINK3:
		// Implement 3 blinks logic
		// This will require a counter and some timing
	}
}

func (ls *ledState) setState(state int) {
	ls.state = state
}
