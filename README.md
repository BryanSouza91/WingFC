# WingFC

### Latest Version 0.4.0

WingFC is a specialized open-source embedded flight controller designed specifically for the rapidly growing sub-250g flying wing FPV (First-Person-View) market. The project's core mission is to provide a reliable, user-friendly, and highly customizable solution for hobbyists and enthusiasts building ultra-lightweight Unmanned Aerial Vehicles (UAVs).

The emphasis on the sub-250g weight class is a direct response to a patchwork of international regulations. Many aviation authorities worldwide, including the Federal Aviation Administration (FAA) in the U.S. and the European Union Aviation Safety Agency (EASA), have established a 250-gram weight threshold as a key regulatory distinction. UAVs below this weight are often considered a lower risk and are exempt from stricter rules, such as mandatory registration, Remote ID requirements, or more complex pilot licensing. By focusing on this market segment, WingFC enables hobbyists to build and fly FPV aircraft that can be legally operated in more places and with fewer bureaucratic hurdles, significantly lowering the barrier to entry for beginners and experienced pilots alike.


Powered by TinyGo, a Go compiler for microcontrollers, WingFC offers a robust and stable software platform. The project provides essential flight control features for elevon-equipped fixed-wing aircraft, including:

- **Stabilization Modes**: Offers flight stabilization to assist pilots, particularly for smooth, cinematic FPV footage and relaxed cruising.
- **Airframe Mixing**: Handles the mixing of aileron and elevator inputs for elevon wings, standard T-tails, and V-tails.
- **YAML Configuration Profiles**: Clean, human-readable aircraft profiles for tuning PID gains, channel mappings, servo endpoints, and filters.
- **Safety Features**: Incorporates failsafe mechanisms to protect the aircraft and others in the event of signal loss.
- **Receiver Protocol Support**: Ensures broad compatibility with popular FPV communication standards, including FlySky's iBus, TBS's CRSF, and the open-source ELRS protocol.

The flight controller is engineered for seamless integration into FPV flying wing airframes like the ZOHD Dart 250G or Alight Wing Aeronautics Flik. The hardware choice of the Seeed Studio Xiao nRF52840 Sense with an onboard IMU aligns with the project's goal of creating a compact, lightweight, and high-performance control system for these specific FPV aircraft.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

```
- Go 1.24+ (TinyGo)
- Supported microcontroller (Seeed Studio Xiao nRF52840 Sense)
```

### Aircraft Configuration (YAML Profiles)

WingFC profiles are stored as YAML in `configs/`. You can list and apply profiles before flashing:

```bash
# List available aircraft profiles
go run ./tools/configgen -list

# Select a profile (e.g. elevon_wing, t_tail, v_tail)
go run ./tools/configgen -profile elevon_wing
```
See [configs/README.md](configs/README.md) for full configuration options and parameter reference.

### Installing and Flashing

Navigate to the firmware source directory:
```bash
cd firmware/src
```

Plug in WingFC board via USB-C and quickly double-press the reset button to enter bootloader mode.

Flash the firmware for your receiver protocol:

```bash
# For CRSF / ELRS protocol:
tinygo flash -target=xiao-ble -tags=crsf .

# For iBus protocol:
tinygo flash -target=xiao-ble -tags=ibus .
```

## Deployment

- Flash the firmware to your supported board
- Connect servos and sensors as described
- Power up and calibrate according to instructions

## Built With

### Software
• [TinyGo](https://tinygo.org/) - Go compiler for microcontrollers  
• [Go](https://golang.org/) - Language
### Hardware
• [Seeed Studio Xiao nrf52840 Sense](https://wiki.seeedstudio.com/XIAO_BLE/) - Xiao nrf52840 Sense microcontroller with onboard IMU


## Supported Receiver Protocols

WingFC supports multiple RC receiver protocols for maximum compatibility:

- **iBus** (FlySky): Supports up to 18 channels (FS-A8S, FS-iA6B)
- **CRSF** (TBS): Supports up to 16 channels
- **ELRS** (open-source): Supports up to 16 channels (uses CRSF build)

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/BryanSouza91/WingFC/tags).

## Authors

• Bryan Souza - Initial work - [BryanSouza91](https://github.com/BryanSouza91)  
• truglodite - CRSF/ELRS finalization, feature additions - [truglodite](https://github.com/truglodite)

See also the list of [contributors](https://github.com/BryanSouza91/WingFC/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

• Hat tip to anyone whose code was used  
• Inspiration from open-source flight controllers  
• TinyGo and Go communities
