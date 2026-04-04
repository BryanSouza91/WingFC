// +build !tinygo
// Test helper for LED tests when not building with TinyGo

package main

import "fmt"

// LEDController is a test-safe version for testing
type LEDController struct {
	red   DigitalPin
	green DigitalPin
	blue  DigitalPin
	state LEDState
}

// NewLEDController creates a mock LED controller for testing
func NewLEDController(red, green, blue DigitalPin) *LEDController {
	if red == nil || green == nil || blue == nil {
		fmt.Println("Warning: nil pin passed to NewLEDController")
	}
	return &LEDController{
		red:   red,
		green: green,
		blue:  blue,
		state: LEDOFF,
	}
}

// SetState sets the LED state (for testing)
func (lc *LEDController) SetState(state LEDState) {
	lc.state = state
}

// Update updates the LED output (for testing)
func (lc *LEDController) Update() {
	// No-op for testing
}

// GetState returns the current state (for testing)
func (lc *LEDController) GetState() LEDState {
	return lc.state
}
