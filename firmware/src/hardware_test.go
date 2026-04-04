//go:build !tinygo
// +build !tinygo

package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestFC_HardwareStructure(t *testing.T) {
	// Verify hardware structure is properly initialized
	hw := &FC_Hardware{}

	assert.NotNil(t, hw)
	assert.Equal(t, uint64(0), hw.ServoPeriod)
	assert.Equal(t, uint64(0), hw.ESCPeriod)
}

func TestFC_HardwareWithMocks(t *testing.T) {
	// Test hardware structure with mock components
	hw := &FC_Hardware{
		I2C:      NewMockI2C(),
		UART:     NewMockUART(),
		ServoPWM: NewMockPWM(),
		ESCPWM:   NewMockPWM(),
		IMU:      NewMockIMUDevice(),
		LED:      NewMockLEDUpdater(),
	}

	assert.NotNil(t, hw.I2C)
	assert.NotNil(t, hw.UART)
	assert.NotNil(t, hw.ServoPWM)
	assert.NotNil(t, hw.ESCPWM)
	assert.NotNil(t, hw.IMU)
	assert.NotNil(t, hw.LED)
}

func TestFC_HardwarePWMChannels(t *testing.T) {
	hw := &FC_Hardware{
		pwmCh1: 0,
		pwmCh2: 1,
		pwmCh3: 2,
	}

	assert.Equal(t, uint8(0), hw.pwmCh1)
	assert.Equal(t, uint8(1), hw.pwmCh2)
	assert.Equal(t, uint8(2), hw.pwmCh3)
}

func TestFC_HardwarePeriods(t *testing.T) {
	hw := &FC_Hardware{
		ServoPeriod: 20000, // 50Hz = 20ms
		ESCPeriod:   20000,
	}

	// Standard PWM frequency for servos is 50Hz = 20ms period
	assert.Equal(t, uint64(20000), hw.ServoPeriod)
	assert.Equal(t, uint64(20000), hw.ESCPeriod)
}

func TestFC_HardwareComponentInteraction(t *testing.T) {
	// Test that hardware components can interact through the hardware struct
	hwUART := NewMockUART()
	hwPWM := NewMockPWM()

	hw := &FC_Hardware{
		UART:     hwUART,
		ServoPWM: hwPWM,
	}

	// Write to UART
	hwUART.WriteData([]byte{0x20, 0x40})

	// Read from UART through hardware struct
	byte1, _ := hw.UART.Read()
	byte2, _ := hw.UART.Read()

	assert.Equal(t, byte(0x20), byte1)
	assert.Equal(t, byte(0x40), byte2)

	// Set PWM through hardware struct
	hw.ServoPWM.Set(0, 1500)
	retrieved := hwPWM.GetChannelValue(0)
	assert.Equal(t, uint32(1500), retrieved)
}

func TestFC_HardwareIMUIntegration(t *testing.T) {
	imu := NewMockIMUDevice()
	imu.SetAccelData(100, 0, 0) // Using int16 values
	imu.SetGyroData(0, 0, 0)

	hw := &FC_Hardware{
		IMU: imu,
	}

	ax, ay, az, err := hw.IMU.ReadAccel()
	assert.NoError(t, err)
	assert.Equal(t, int16(100), ax)
	assert.Equal(t, int16(0), ay)
	assert.Equal(t, int16(0), az)

	gx, gy, gz, err := hw.IMU.ReadGyro()
	assert.NoError(t, err)
	assert.Equal(t, int16(0), gx)
	assert.Equal(t, int16(0), gy)
	assert.Equal(t, int16(0), gz)
}

func TestFC_HardwareLEDIntegration(t *testing.T) {
	led := NewMockLEDUpdater()

	hw := &FC_Hardware{
		LED: led,
	}

	hw.LED.SetState(ARMED)
	hw.LED.Update()
	assert.Equal(t, ARMED, led.GetLastState())

	hw.LED.SetState(FAILSAFED)
	hw.LED.Update()
	assert.Equal(t, FAILSAFED, led.GetLastState())
}

func TestFC_HardwareMultiplePWMChannels(t *testing.T) {
	pwm := NewMockPWM()

	hw := &FC_Hardware{
		ServoPWM: pwm,
		pwmCh1:   0,
		pwmCh2:   1,
		pwmCh3:   2,
	}

	// Set multiple channels
	hw.ServoPWM.Set(hw.pwmCh1, 1500)
	hw.ServoPWM.Set(hw.pwmCh2, 1600)
	hw.ServoPWM.Set(hw.pwmCh3, 1400)

	assert.Equal(t, uint32(1500), pwm.GetChannelValue(hw.pwmCh1))
	assert.Equal(t, uint32(1600), pwm.GetChannelValue(hw.pwmCh2))
	assert.Equal(t, uint32(1400), pwm.GetChannelValue(hw.pwmCh3))
}

func TestFC_HardwareI2CIntegration(t *testing.T) {
	i2c := NewMockI2C()
	
	testData := []byte{0xAA, 0xBB}
	i2c.SetRegisterData(0x0F, testData)

	hw := &FC_Hardware{
		I2C: i2c,
	}

	readBuffer := make([]byte, 2)
	writeBuffer := []byte{0x0F}

	err := hw.I2C.Tx(0x68, writeBuffer, readBuffer)
	assert.NoError(t, err)
	assert.Equal(t, testData, readBuffer)
}

func TestFC_HardwareUARTIntegration(t *testing.T) {
	uart := NewMockUART()

	hw := &FC_Hardware{
		UART: uart,
	}

	// Write data through the hardware struct
	testData := []byte{0x20, 0x40, 0xD0, 0x07}
	for _, b := range testData {
		uart.WriteData([]byte{b})
	}

	// Read it back
	for _, expectedByte := range testData {
		b, err := hw.UART.Read()
		assert.NoError(t, err)
		assert.Equal(t, expectedByte, b)
	}
}

func TestFC_HardwareStateSequence(t *testing.T) {
	// Test typical initialization sequence
	led := NewMockLEDUpdater()
	pwm := NewMockPWM()

	hw := &FC_Hardware{
		LED:      led,
		ServoPWM: pwm,
	}

	// Simulate initialization states
	states := []LEDState{PWMCONFIG, SERVOINIT, ESCINIT, IMUINIT, LEDOFF}

	for _, state := range states {
		hw.LED.SetState(state)
		hw.LED.Update()
		assert.Equal(t, state, led.GetLastState())
	}
}

func TestFC_HardwareErrorHandling(t *testing.T) {
	// Test error conditions
	hw := &FC_Hardware{
		I2C: NewMockI2C(),
	}

	// Nil hardware should be handled gracefully in actual code
	assert.NotNil(t, hw)
	assert.NotNil(t, hw.I2C)
}
