# WingFC YAML Configuration & Airframe Profiles

WingFC supports YAML-based aircraft profiles. Configuration parameters (such as airframe mixing, channel maps, servo trims, rates, filters, and PID gains) are defined in human-readable `.yaml` files and compiled down to [firmware/src/config.go](file:///home/bryansouza/Repos/WingFC/firmware/src/config.go) at build time.

This architecture ensures:
* **Zero Runtime Overhead**: No YAML parser or dynamic memory allocation overhead on the microcontroller.
* **100% TinyGo Compatibility**: Full compatibility with bare-metal targets without reflection bottlenecks.
* **Compile-Time Optimization**: Fast reciprocal multiplication (`1.0 / MAX_PITCH_ANGLE`) and fixed array sizing remain zero-cost compile constants.
* **Version-Controlled Aircraft Profiles**: Maintain separate profiles for each of your airframes.

---

## Quick Start

### 1. List Available Profiles
From the repository root:
```bash
go run ./tools/configgen -list
```
Output:
```
Available WingFC Aircraft Profiles:
  • default          [AIRFRAME_ELEVON]
  • elevon_wing      [AIRFRAME_ELEVON]
  • t_tail           [AIRFRAME_DUAL_AILERON_T_TAIL]
  • v_tail           [AIRFRAME_DUAL_AILERON_V_TAIL]
```

### 2. Apply a Profile
To select a profile and update `config.go`:
```bash
# Apply a preconfigured profile
go run ./tools/configgen -profile elevon_wing

# Or specify a custom YAML file
go run ./tools/configgen -config configs/my_plane.yaml
```

### 3. Re-generate Default
If you edit [configs/default.yaml](file:///home/bryansouza/Repos/WingFC/configs/default.yaml), regenerate with standard Go tooling:
```bash
# From firmware/src:
go generate ./...

# Or from repository root:
go generate ./firmware/src/...
```

### 4. Build Firmware
Compile and flash as usual:
```bash
cd firmware/src
tinygo flash -target=xiao-ble -tags=crsf .
```

---

## Pre-Packaged Profiles

| Profile | File | Airframe Type | Description |
|---|---|---|---|
| **default** | `configs/default.yaml` | `AIRFRAME_ELEVON` | Default flight controller settings |
| **elevon_wing** | `configs/elevon_wing.yaml` | `AIRFRAME_ELEVON` | Flying wings (e.g. ZOHD Dart 250G, Flik) |
| **t_tail** | `configs/t_tail.yaml` | `AIRFRAME_DUAL_AILERON_T_TAIL` | Conventional T-tail trainer / sport plane |
| **v_tail** | `configs/v_tail.yaml` | `AIRFRAME_DUAL_AILERON_V_TAIL` | V-Tail sport plane / glider |

---

## YAML Configuration Schema

```yaml
version: "1.0"

# Protocol settings
protocol:
  num_channels: 18         # Total number of RC channels (e.g., 16 or 18)

# Board physical orientation
# Values: 0 (DEFAULT), 1 (CW90), 2 (CW180), 3 (CW270), 4 (FLIP), 5 (FLIP_CW90), 6 (FLIP_CW180), 7 (FLIP_CW270)
board:
  orientation: 0

# Airframe geometry & mixer
# Values:
#   AIRFRAME_ELEVON
#   AIRFRAME_SINGLE_AILERON_T_TAIL
#   AIRFRAME_DUAL_AILERON_T_TAIL
#   AIRFRAME_SINGLE_AILERON_V_TAIL
#   AIRFRAME_DUAL_AILERON_V_TAIL
airframe:
  type: AIRFRAME_ELEVON

# Radio receiver channel mappings (0-indexed) & pulse parameters
receiver:
  channels:
    aileron: 0            # CH1 (Roll)
    elevator: 1           # CH2 (Pitch)
    throttle: 2           # CH3 (Throttle)
    yaw: 3                # CH4 (Yaw)
    arm: 4                # CH5 (Arming switch)
    manual_mode: 7        # CH8 (Stabilized / Manual mode switch)
    tuning_a: 5           # CH6 (Potentiometer for tuning parameter A)
    tuning_b: 6           # CH7 (Potentiometer for tuning parameter B)
    tuning_c: 8           # CH9
    tuning_d: 9           # CH10
  deadband: 10            # Deadband in microseconds around center
  high_rx_value: 1800     # Threshold for switch triggers
  min_rx_value: 988       # Min RC pulse in microseconds
  max_rx_value: 2012      # Max RC pulse in microseconds
  neutral_rx_value: 1500  # Center RC pulse in microseconds

# Motor ESC and PWM Configuration
pwm:
  dshot: false            # Enable DShot for ESC (true/false)
  dshot_mode: 600         # 150, 300, or 600
  servo_frequency_hz: 100 # 50Hz for analog servos, 100-333Hz for digital servos
  esc_frequency_hz: 400   # 400Hz for standard high-speed ESC PWM

# Servo calibration: pulse limits, subtrim, and reversal
servos:
  servo1: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo2: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo4: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo5: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo6: { min: 1100, max: 1900, subtrim: 0, reverse: false }

# Flight controller stabilization dynamics and PID loop
flight_control:
  max_rates_deg:
    pitch: 200            # Max angular rate in deg/s (manual/rate mode)
    roll: 600
    yaw: 100
  max_angles_deg:
    pitch: 45.0           # Max stabilized pitch tilt in degrees
    roll: 60.0            # Max stabilized roll tilt in degrees
  pid_weight: 0.5         # Blending between gyro rate feedback and attitude error
  filters:
    accel_level: 2        # 0 = none, 1 = mild (α≈0.25), 2 = stronger (α≈0.125)
    gyro_level: 0         # Gyro filtering level
  pid:
    pitch: { p: 1.00, i: 0.050, d: 0.00 }
    roll:  { p: 1.00, i: 0.050, d: 0.00 }
    yaw:   { p: 0.50, i: 0.020, d: 0.00 }

# In-flight transmitter PID tuning
# Parameter IDs: 0 (Disabled), 1 (Pitch P), 2 (Roll P), 3 (Pitch I), 4 (Roll I), 5 (Pitch D), 6 (Roll D)
tuning:
  param_a: { parameter: 0, min: 0.1000, max: 2.0000 }
  param_b: { parameter: 0, min: 0.0100, max: 0.5000 }
  param_c: { parameter: 0, min: 0.0010, max: 0.1000 }
  param_d: { parameter: 0, min: 0.0010, max: 0.1000 }
```

---

## Generator CLI Options

```bash
go run ./tools/configgen [flags]

Flags:
  -config string
        Path to YAML configuration file (e.g. configs/my_custom.yaml)
  -profile string
        Profile name in configs/ directory (e.g. elevon_wing, t_tail, v_tail)
  -out string
        Path to output generated Go file (default: firmware/src/config.go)
  -list
        List available profiles in configs/ directory
  -validate-only
        Validate the YAML configuration without generating or overwriting any file
```
