package main

import (
	"fmt"
	"machine"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// FC_Hardware holds all hardware interfaces and their state.
type FC_Hardware struct {
	I2C   I2C
	UART  UART
	SPI   SPI   // Underlying SPI bus for DShot
	DShot DShot // High-level DShot driver
	IMU   IMUDevice
	LED   LEDUpdater

	// Dedicated PWM Instances for Seeed XIAO nRF52840
	PWM0 PWM // Instance 0: ESC (Freq 1)
	PWM1 PWM // Instance 1: Servos 1,2,4,5 (Freq 2)
	PWM2 PWM // Instance 2: Servo 6 (Freq 2)

	// PWM channel IDs
	pwmCh1, pwmCh2, pwmCh3, pwmCh4, pwmCh5, pwmCh6 uint8

	// Tracked periods for PWM pulse width calculations
	PeriodPWM0 uint32
	PeriodPWM1 uint32
	PeriodPWM2 uint32

	// Hardware pin mappings
	PWM_CH1_PIN machine.Pin // Servo 1 (Aileron)
	PWM_CH2_PIN machine.Pin // Servo 2 (Elevator)
	PWM_CH3_PIN machine.Pin // ESC (Throttle)
	PWM_CH4_PIN machine.Pin // Servo 4 (Rudder/V-tail)
	PWM_CH5_PIN machine.Pin // Servo 5 (Aileron 2)
	PWM_CH6_PIN machine.Pin // Servo 6 (Future)
}

func InitHardware(hw *FC_Hardware) error {
	if hw == nil {
		return fmt.Errorf("hardware cannot be nil")
	}

	// Initialize the LED controller first for status indication during setup
	hw.LED = NewLEDController(
		NewMachinePin(machine.LED_RED),
		NewMachinePin(machine.LED_GREEN),
		NewMachinePin(machine.LED_BLUE),
	)

	if DSHOT {
		// Initialize DShot via SPI
		// On XIAO nRF52840, SPI0 MOSI is typically D10 (PWM_CH3_PIN)
		hw.SPI = NewMachineSPI(machine.SPI0)
		hw.DShot = NewDShotDriver(hw.SPI)
		if err := hw.DShot.Configure(); err != nil {
			hw.LED.SetState(ESCERROR)
			hw.LED.Update()
			return fmt.Errorf("DShot/SPI configuration failed: %w", err)
		}
		hw.DShot.SendThrottle(0, false) // Ensure ESC starts in a safe state
	}

	// Initialize PWM instances before I2C and IMU to ensure ESC is set to a safe state early in the process.
	// Allowing the ESC to initialize properly if it requires a valid signal at power-up and preventing unintended motor spin during setup.
	if err := initPWMs(hw); err != nil {
		hw.LED.SetState(PWMERROR)
		hw.LED.Update()
		return err
	}

	if err := initUART(hw); err != nil {
		hw.LED.SetState(PWMERROR)
		hw.LED.Update()
		return err
	}
	if err := initI2C(hw); err != nil {
		hw.LED.SetState(IMUERROR)
		hw.LED.Update()
		return err
	}
	if err := initIMU(hw); err != nil {
		hw.LED.SetState(IMUERROR)
		hw.LED.Update()
		return err
	}

	hw.LED.SetState(LEDOFF)
	hw.LED.Update()
	println("All hardware initialized successfully.")
	return nil
}

func initUART(hw *FC_Hardware) error {
	cfg := machine.UARTConfig{BaudRate: BAUD_RATE, TX: machine.UART_TX_PIN, RX: machine.UART_RX_PIN}
	return hw.UART.Configure(cfg)
}

// initPWMs sets up the 3 instances for multi-frequency operation.
func initPWMs(hw *FC_Hardware) error {
	hw.LED.SetState(PWMCONFIG)
	hw.LED.Update()

	var err error

	// --- Instance 0: ESC (400Hz or 50Hz) ---
	escCfg := machine.PWMConfig{Period: machine.GHz * 1 / ESC_PWM_FREQUENCY}
	if err := hw.PWM0.Configure(escCfg); err != nil {
		return fmt.Errorf("PWM0 config failed: %w", err)
	}
	hw.PeriodPWM0 = escCfg.Period

	if hw.pwmCh3, err = hw.PWM0.Channel(hw.PWM_CH3_PIN); err != nil {
		return err
	}

	// Set ESC to minimum throttle before initializing I2C and IMU to prevent unintended motor spin during setup
	// and allow the ESC to initialize properly if it requires a valid signal at power-up.
	hw.PWM0.Set(hw.pwmCh3, 0) // Start with 0% duty cycle for ESC

	// --- Instance 1: Servos 1, 2, 4, 5 (50Hz) ---
	servoCfg := machine.PWMConfig{Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY}
	if err := hw.PWM1.Configure(servoCfg); err != nil {
		return fmt.Errorf("PWM1 config failed: %w", err)
	}
	hw.PeriodPWM1 = servoCfg.Period

	if hw.pwmCh1, err = hw.PWM1.Channel(hw.PWM_CH1_PIN); err != nil {
		return err
	}
	if hw.pwmCh2, err = hw.PWM1.Channel(hw.PWM_CH2_PIN); err != nil {
		return err
	}
	if hw.pwmCh4, err = hw.PWM1.Channel(hw.PWM_CH4_PIN); err != nil {
		return err
	}
	if hw.pwmCh5, err = hw.PWM1.Channel(hw.PWM_CH5_PIN); err != nil {
		return err
	}

	// --- Instance 2: Servo 6 (50Hz) ---
	if err := hw.PWM2.Configure(servoCfg); err != nil {
		return fmt.Errorf("PWM2 config failed: %w", err)
	}
	hw.PeriodPWM2 = servoCfg.Period

	if hw.pwmCh6, err = hw.PWM2.Channel(hw.PWM_CH6_PIN); err != nil {
		return err
	}

	println("Multi-frequency PWM instances configured.")
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
			hw.LED.SetState(IMUERROR)
			hw.LED.Update()
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
		hw.LED.SetState(IMUERROR)
		hw.LED.Update()
		return fmt.Errorf("IMU not connected after retries")
	}

	println("LSM6DS3TR IMU configured and initialized.")
	return nil
}
