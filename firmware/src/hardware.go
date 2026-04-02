package main

import (
	"fmt"
	"machine"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// FC_Hardware holds all hardware interfaces and their state.
type FC_Hardware struct {
	I2C      I2C
	UART     UART
	ServoPWM PWM
	ESCPWM   PWM
	IMU      IMUDevice
	LED      LEDUpdater

	// PWM channel IDs
	pwmCh1 uint8
	pwmCh2 uint8
	pwmCh3 uint8

	// Tracked periods for PWM pulse width calculations
	ServoPeriod uint64
	ESCPeriod   uint64

	// Hardware pin mappings
	PWM_CH1_PIN machine.Pin // Aileron Servo
	PWM_CH2_PIN machine.Pin // Elevator Servo
	PWM_CH3_PIN machine.Pin // ESC (Electronic Speed Controller)
}

// InitHardware initializes all hardware components with dependency injection.
// Returns nil if successful or an error if any component fails to initialize.
func InitHardware(hw *FC_Hardware) error {
	if hw == nil {
		return fmt.Errorf("hardware cannot be nil")
	}

	// Initialize UART for receiver communication
	if err := initUART(hw); err != nil {
		hw.LED.SetState(LEDOFF)
		hw.LED.Update()
		return err
	}

	// Initialize servo PWM (pwm0)
	if err := initServoPWM(hw); err != nil {
		hw.LED.SetState(PWMERROR)
		hw.LED.Update()
		return err
	}

	// Initialize ESC PWM (pwm1)
	if err := initESCPWM(hw); err != nil {
		hw.LED.SetState(ESCERROR)
		hw.LED.Update()
		return err
	}

	// Initialize I2C for IMU
	if err := initI2C(hw); err != nil {
		hw.LED.SetState(IMUCONFIG)
		hw.LED.Update()
		return err
	}

	// Initialize IMU sensor
	if err := initIMU(hw); err != nil {
		hw.LED.SetState(IMUERROR)
		hw.LED.Update()
		return err
	}

	// All systems initialized successfully
	hw.LED.SetState(LEDOFF)
	hw.LED.Update()
	println("All hardware initialized successfully.")

	return nil
}

// initUART configures the UART receiver interface.
func initUART(hw *FC_Hardware) error {
	hw.LED.SetState(PWMCONFIG)
	hw.LED.Update()

	cfg := machine.UARTConfig{
		BaudRate: BAUD_RATE,
		TX:       machine.UART_TX_PIN,
		RX:       machine.UART_RX_PIN,
	}
	if err := hw.UART.Configure(cfg); err != nil {
		return fmt.Errorf("UART configure failed: %w", err)
	}
	println("UART configured for receiver.")
	return nil
}

// initServoPWM configures PWM0 for servo control with retry logic.
func initServoPWM(hw *FC_Hardware) error {
	hw.LED.SetState(PWMCONFIG)
	hw.LED.Update()

	servoPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
	}

	// Retry loop for PWM configuration
	for retries := 0; retries < 5; retries++ {
		if err := hw.ServoPWM.Configure(servoPWMConfig); err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("servo PWM configure failed after retries: %w", err)
		}
		break
	}

	hw.LED.SetState(SERVOINIT)
	hw.LED.Update()

	hw.ServoPeriod = servoPWMConfig.Period

	// Channel 1 configuration
	for retries := 0; retries < 5; retries++ {
		ch, err := hw.ServoPWM.Channel(hw.PWM_CH1_PIN)
		if err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("servo PWM channel 1 failed after retries: %w", err)
		}
		hw.pwmCh1 = ch
		break
	}

	// Channel 2 configuration
	for retries := 0; retries < 5; retries++ {
		ch, err := hw.ServoPWM.Channel(hw.PWM_CH2_PIN)
		if err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("servo PWM channel 2 failed after retries: %w", err)
		}
		hw.pwmCh2 = ch
		break
	}

	println("PWM configured for servos.")
	return nil
}

// initESCPWM configures PWM1 for ESC control with retry logic.
func initESCPWM(hw *FC_Hardware) error {
	hw.LED.SetState(ESCINIT)
	hw.LED.Update()

	escPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / ESC_PWM_FREQUENCY,
	}

	// Retry loop for PWM configuration
	for retries := 0; retries < 5; retries++ {
		if err := hw.ESCPWM.Configure(escPWMConfig); err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("ESC PWM configure failed after retries: %w", err)
		}
		break
	}

	hw.ESCPeriod = escPWMConfig.Period

	// Channel 3 configuration
	for retries := 0; retries < 5; retries++ {
		ch, err := hw.ESCPWM.Channel(hw.PWM_CH3_PIN)
		if err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("ESC PWM channel failed after retries: %w", err)
		}
		hw.pwmCh3 = ch
		break
	}

	println("PWM configured for ESC.")
	return nil
}

// initI2C configures the I2C bus for IMU communication.
func initI2C(hw *FC_Hardware) error {
	hw.LED.SetState(IMUCONFIG)
	hw.LED.Update()

	cfg := machine.I2CConfig{
		Frequency: 400 * machine.KHz,
	}
	if err := hw.I2C.Configure(cfg); err != nil {
		return fmt.Errorf("I2C configure failed: %w", err)
	}
	println("I2C configured for IMU.")
	return nil
}

// initIMU initializes the LSM6DS3TR IMU with retry logic and connectivity check.
func initIMU(hw *FC_Hardware) error {
	hw.LED.SetState(IMUINIT)
	hw.LED.Update()

	// Retry loop for IMU configuration
	for retries := 0; retries < 5; retries++ {
		cfg := lsm6ds3tr.Configuration{
			AccelRange:      lsm6ds3tr.ACCEL_8G,
			AccelSampleRate: lsm6ds3tr.ACCEL_SR_416,
			GyroRange:       lsm6ds3tr.GYRO_1000DPS,
			GyroSampleRate:  lsm6ds3tr.GYRO_SR_416,
		}
		if err := hw.IMU.Configure(cfg); err != nil {
			if retries < 4 {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			return fmt.Errorf("IMU configure failed after retries: %w", err)
		}
		break
	}

	// Connectivity check with retry logic
	for retries := 0; retries < 5; retries++ {
		if hw.IMU.Connected() {
			break
		}
		if retries < 4 {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		return fmt.Errorf("IMU not connected after retries")
	}

	println("LSM6DS3TR IMU configured and initialized.")
	return nil
}
