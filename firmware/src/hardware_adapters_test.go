//go:build tinygo
// +build tinygo

package main

import (
	"machine"
	"testing"

	"github.com/stretchr/testify/assert"
)

// ===== MachinePin Adapter Tests =====

func TestNewMachinePin(t *testing.T) {
	adapter := NewMachinePin(machine.Pin(0))

	assert.NotNil(t, adapter)
}

func TestMachinePinConfigure(t *testing.T) {
	adapter := NewMachinePin(machine.Pin(0))

	cfg := machine.PinConfig{Mode: machine.PinOutput}
	// Should not panic
	adapter.Configure(cfg)
}

// ===== MachineUART Adapter Tests =====

func TestMachineUARTAdapter(t *testing.T) {
	// We can't easily test with real machine.UART without hardware
	// But we can verify the adapter structure is correct

	// This would normally use machine.UART
	// For testing, we just verify the interface is implemented
	var _ UART = (*MachineUART)(nil)
}

// ===== MachinePWM0 Adapter Tests =====

func TestMachinePWM0Adapter(t *testing.T) {
	// Verify PWM adapter implements the PWM interface
	var _ PWM = (*MachinePWM0)(nil)
}

// ===== Mock Adapter Tests =====

func TestMockPWMAdapter(t *testing.T) {
	pwm := NewMockPWM()

	assert.NotNil(t, pwm)

	// Configure should succeed
	cfg := machine.PWMConfig{}
	err := pwm.Configure(cfg)
	assert.NoError(t, err)
}

func TestMockPWMSet(t *testing.T) {
	tests := []struct {
		name    string
		channel uint8
		value   uint16
	}{
		{
			name:    "Set channel 0",
			channel: 0,
			value:   1500,
		},
		{
			name:    "Set channel 1",
			channel: 1,
			value:   2000,
		},
		{
			name:    "Set channel 255",
			channel: 255,
			value:   1000,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pwm := NewMockPWM()
			pwm.Set(test.channel, uint32(test.value))

			retrieved := pwm.GetChannelValue(test.channel)
			assert.Equal(t, uint32(test.value), retrieved)
		})
	}
}

func TestMockPWMMultipleChannels(t *testing.T) {
	pwm := NewMockPWM()

	values := map[uint8]uint16{
		0: 1500,
		1: 1800,
		2: 1200,
		3: 1000,
	}

	for ch, val := range values {
		pwm.Set(ch, uint32(val))
	}

	for ch, expected := range values {
		retrieved := pwm.GetChannelValue(ch)
		assert.Equal(t, uint32(expected), retrieved)
	}
}

// ===== Mock I2C Tests =====

func TestMockI2C(t *testing.T) {
	i2c := NewMockI2C()

	assert.NotNil(t, i2c)

	// Configure should succeed
	cfg := machine.I2CConfig{}
	err := i2c.Configure(cfg)
	assert.NoError(t, err)
}

func TestMockI2CSetRegisterData(t *testing.T) {
	i2c := NewMockI2C()

	testData := []byte{0x01, 0x02, 0x03}
	i2c.SetRegisterData(0x00, testData)

	// Read it back
	readBuffer := make([]byte, 3)
	writeBuffer := []byte{0x00}

	err := i2c.Tx(0x68, writeBuffer, readBuffer)
	assert.NoError(t, err)

	assert.Equal(t, testData, readBuffer)
}

// ===== Mock Digital Pin Tests =====

func TestMockPinHighLow(t *testing.T) {
	pin := NewMockPin()

	assert.Equal(t, uint8(1), pin.GetState()) // Default high

	pin.Low()
	assert.Equal(t, uint8(0), pin.GetState())

	pin.High()
	assert.Equal(t, uint8(1), pin.GetState())
}

func TestMockPinConfigure(t *testing.T) {
	pin := NewMockPin()

	cfg := machine.PinConfig{Mode: machine.PinOutput}
	pin.Configure(cfg)

	// Configure should not change state
	assert.Equal(t, uint8(1), pin.GetState())
}

func TestMockPinMultipleOperations(t *testing.T) {
	pin := NewMockPin()

	pin.Low()
	assert.Equal(t, uint8(0), pin.GetState())

	pin.High()
	assert.Equal(t, uint8(1), pin.GetState())

	pin.Low()
	assert.Equal(t, uint8(0), pin.GetState())

	pin.Low() // Double low
	assert.Equal(t, uint8(0), pin.GetState())

	pin.High()
	assert.Equal(t, uint8(1), pin.GetState())
}

// ===== Mock LED Updater Tests =====

func TestMockLEDUpdater(t *testing.T) {
	led := NewMockLEDUpdater()

	assert.NotNil(t, led)
	assert.Equal(t, LEDState(0), led.GetLastState())
}

func TestMockLEDUpdaterSetState(t *testing.T) {
	led := NewMockLEDUpdater()

	states := []LEDState{LEDOFF, PWMCONFIG, ARMED, FAILSAFED}

	for _, state := range states {
		led.SetState(state)
		led.Update()
		assert.Equal(t, state, led.GetLastState())
	}
}

// ===== Mock IMU Device Tests =====

func TestMockIMUDevice(t *testing.T) {
	imu := NewMockIMUDevice()

	assert.NotNil(t, imu)

	gx, gy, gz, err := imu.ReadGyro()
	assert.NoError(t, err)
	assert.Equal(t, int16(0), gx)
	assert.Equal(t, int16(0), gy)
	assert.Equal(t, int16(0), gz)

	ax, ay, az, err := imu.ReadAccel()
	assert.NoError(t, err)
	assert.Equal(t, int16(0), ax)
	assert.Equal(t, int16(0), ay)
	assert.Equal(t, int16(0), az)
}

func TestMockIMUDeviceSetData(t *testing.T) {
	imu := NewMockIMUDevice()

	imu.SetGyroData(100, 200, 300)
	gx, gy, gz, _ := imu.ReadGyro()
	assert.Equal(t, int16(100), gx)
	assert.Equal(t, int16(200), gy)
	assert.Equal(t, int16(300), gz)

	imu.SetAccelData(400, 500, 600)
	ax, ay, az, _ := imu.ReadAccel()
	assert.Equal(t, int16(400), ax)
	assert.Equal(t, int16(500), ay)
	assert.Equal(t, int16(600), az)
}

func TestMockIMUDeviceMultipleReads(t *testing.T) {
	imu := NewMockIMUDevice()

	imu.SetGyroData(50, 60, 70)

	// Multiple reads should return same value
	for i := 0; i < 5; i++ {
		gx, gy, gz, _ := imu.ReadGyro()
		assert.Equal(t, int16(50), gx)
		assert.Equal(t, int16(60), gy)
		assert.Equal(t, int16(70), gz)
	}
}

func TestMockIMUDeviceUpdateData(t *testing.T) {
	imu := NewMockIMUDevice()

	imu.SetAccelData(100, 200, 300)
	ax1, _, _, _ := imu.ReadAccel()
	assert.Equal(t, int16(100), ax1)

	// Update data
	imu.SetAccelData(1000, 2000, 3000)
	ax2, _, _, _ := imu.ReadAccel()
	assert.Equal(t, int16(1000), ax2)
}

// ===== Float Comparison Helper Tests =====

func TestFloatEqual(t *testing.T) {
	tests := []struct {
		name      string
		a         float64
		b         float64
		tolerance float64
		expected  bool
	}{
		{
			name:      "Identical values",
			a:         1.5,
			b:         1.5,
			tolerance: 0.01,
			expected:  true,
		},
		{
			name:      "Within tolerance",
			a:         1.5,
			b:         1.505,
			tolerance: 0.01,
			expected:  true,
		},
		{
			name:      "Outside tolerance",
			a:         1.5,
			b:         1.52,
			tolerance: 0.01,
			expected:  false,
		},
		{
			name:      "Negative difference",
			a:         1.5,
			b:         1.495,
			tolerance: 0.01,
			expected:  true,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := FloatEqual(test.a, test.b, test.tolerance)
			assert.Equal(t, test.expected, result)
		})
	}
}
