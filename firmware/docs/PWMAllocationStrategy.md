# **nRF52840 PWM Allocation (XIAO BLE Sense & Sense Plus)**

The nRF52840 features 4 independent PWM instances (PWM0-3). Each instance has 4 channels that **must share the same frequency.** This is a critical constraint for flight controllers using a mix of 50Hz servos and 400Hz+ ESCs.

## **1\. Optimal Allocation for WingFC (Standard & Plus)**

To ensure stable flight, keep servos and ESCs on separate PWM instances.

| Component | Pin | Instance | Frequency | Notes |
| :---- | :---- | :---- | :---- | :---- |
| **Aileron Servo** | D0 | machine.PWM0 | 50 Hz | Standard & Plus |
| **Elevator Servo** | D1 | machine.PWM0 | 50 Hz | Standard & Plus |
| **ESC (Throttle)** | D2 | machine.PWM1 | 400 Hz+ | Supports OneShot/DShot |

## **2\. Expansion Scenarios for "Sense Plus"**

The **XIAO Sense Plus** breaks out more pins, allowing for complex airframes without sacrificing I2C or UART with the addition of UART2.

### **The "Full House" Airplane (Servos \+ ESC)**

Cluster all servos on **PWM0** to save other instances for high-speed tasks.

* **D0:** Aileron (PWM0)  
* **D1:** Elevator (PWM0)  
* **D3:** Rudder (PWM0)  
* **D7:** Flaps (PWM0) — *Note: On Plus, D7 is available as GPIO however will be used for UART2*  
* **D2:** ESC (PWM1) — *Independent high-speed throttle*


## **3\. Plus-Specific Considerations**

The XIAO Sense Plus introduces features that impact how we handle the control loop:

* **Battery Management:** The Plus version includes a built-in battery charging chip. It can monitor battery voltage via the internal P0.31 (VBAT) pin using ADC to implement a low-voltage battery failsafe.  
* **Additional GPIO:** While the standard Sense has 11 GPIOs, the Plus provides more routing options, allowing for the RC Receiver (UART) to be moved to the extra pins if D6/D7 are needed for PWM.  
* **Flash Memory:** The Plus often features more QSPI Flash, which is ideal for "Blackbox" flight logging. Need to write the driver for TinyGo before we go down this path.

## **4\. TinyGo Configuration**

Ensure the periods are set explicitly during hardware initialization to prevent frequency clashing:  
`// 50Hz for traditional analog/digital servos`  
`hw.ServoPWM.Configure(machine.PWMConfig{Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY})` 

`// 400Hz for standard ESCs (for smoother throttle response)`  
`hw.ESCPWM.Configure(machine.PWMConfig{Period: machine.GHz * 1 / ESC_PWM_FREQUENCY})`   

## **5\. Hardware Guardrails**

* **Avoid D6/D7 (Standard):** These are UART pins used for your RC Receiver (SBUS/IBUS). On the Plus, check your specific pin map if you use GPS which will use the secondary UART.  
* **Reserve D4/D5:** Keep these free for I2C. We will be adding Magnetometer and will likely want to add a Barometer (BMP280/SPL06) for altitude hold.  
* **Internal IMU:** The LSM6DS3TR-C is on an internal I2C bus. Do not assign PWM to the internal SDA/SCL pins or the IMU will disconnect.