//go:build !tinygo
// +build !tinygo

package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestNewLEDController(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)

	assert.NotNil(t, controller)
	assert.Equal(t, LEDOFF, controller.state)
}

func TestLEDControllerSetState(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)

	states := []LEDState{
		PWMCONFIG,
		PWMERROR,
		SERVOINIT,
		SERVOERROR,
		ESCINIT,
		ESCERROR,
		IMUCONFIG,
		IMUINIT,
		IMUERROR,
		CALIBRATE,
		DISARMED,
		ARMED,
		FAILSAFED,
	}

	for _, state := range states {
		t.Run("SetState "+string(rune(state)), func(t *testing.T) {
			controller.SetState(state)
			assert.Equal(t, state, controller.state)
		})
	}
}

func TestLEDControllerLEDOFF(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(LEDOFF)
	controller.Update()

	// LEDOFF: all pins high (LEDs off)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerPWMCONFIG(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(PWMCONFIG)
	controller.Update()

	// PWMCONFIG: R only (red low, green high, blue high)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerPWMERROR(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(PWMERROR)
	controller.Update()

	// PWMERROR: RG (red low, green low, blue high)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerSERVOINIT(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(SERVOINIT)
	controller.Update()

	// SERVOINIT: G only (red high, green low, blue high)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerSERVOERROR(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(SERVOERROR)
	controller.Update()

	// SERVOERROR: GB (red high, green low, blue low)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerESCINIT(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(ESCINIT)
	controller.Update()

	// ESCINIT: R (red low, green high, blue high)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerESCERROR(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(ESCERROR)
	controller.Update()

	// ESCERROR: RGB (all low)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerIMUCONFIG(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(IMUCONFIG)
	controller.Update()

	// IMUCONFIG: B only (red high, green high, blue low)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerIMUINIT(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(IMUINIT)
	controller.Update()

	// IMUINIT: R (red low, green high, blue high)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerIMUERROR(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(IMUERROR)
	controller.Update()

	// IMUERROR: RB (red low, green high, blue low)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerCALIBRATE(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(CALIBRATE)
	controller.Update()

	// CALIBRATE: RGB (all low)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerDISARMED(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(DISARMED)
	controller.Update()

	// DISARMED: G (red high, green low, blue high)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(0), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerARMED(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(ARMED)
	controller.Update()

	// ARMED: B (red high, green high, blue low)
	assert.Equal(t, uint8(1), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(0), blue.GetState())
}

func TestLEDControllerFAILSAFED(t *testing.T) {
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(FAILSAFED)
	controller.Update()

	// FAILSAFED: R (red low, green high, blue high)
	assert.Equal(t, uint8(0), red.GetState())
	assert.Equal(t, uint8(1), green.GetState())
	assert.Equal(t, uint8(1), blue.GetState())
}

func TestLEDControllerStateTransition(t *testing.T) {
	// Test transitioning between states
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)

	// Start at LEDOFF
	assert.Equal(t, LEDOFF, controller.state)

	// Transition to PWMCONFIG
	controller.SetState(PWMCONFIG)
	controller.Update()
	assert.Equal(t, PWMCONFIG, controller.state)
	assert.Equal(t, uint8(0), red.GetState())

	// Transition to ARMED
	controller.SetState(ARMED)
	controller.Update()
	assert.Equal(t, ARMED, controller.state)
	assert.Equal(t, uint8(0), blue.GetState())

	// Back to LEDOFF
	controller.SetState(LEDOFF)
	controller.Update()
	assert.Equal(t, LEDOFF, controller.state)
	assert.Equal(t, uint8(1), red.GetState())
}

func TestLEDControllerMultipleUpdates(t *testing.T) {
	// Test that multiple updates of the same state produce consistent results
	red := NewMockPin()
	green := NewMockPin()
	blue := NewMockPin()

	controller := NewLEDController(red, green, blue)
	controller.SetState(ARMED)

	for i := 0; i < 5; i++ {
		controller.Update()
		assert.Equal(t, uint8(1), red.GetState())
		assert.Equal(t, uint8(1), green.GetState())
		assert.Equal(t, uint8(0), blue.GetState())
	}
}
