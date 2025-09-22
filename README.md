# WingFC

WingFC is a specialized open-source embedded flight controller designed specifically for the rapidly growing sub-250g flying wing FPV (First-Person-View) market. The project's core mission is to provide a reliable, user-friendly, and highly customizable solution for hobbyists and enthusiasts building ultra-lightweight Unmanned Aerial Vehicles (UAVs).

The emphasis on the sub-250g weight class is a direct response to a patchwork of international regulations. Many aviation authorities worldwide, including the Federal Aviation Administration (FAA) in the U.S. and the European Union Aviation Safety Agency (EASA), have established a 250-gram weight threshold as a key regulatory distinction. UAVs below this weight are often considered a lower risk and are exempt from stricter rules, such as mandatory registration, Remote ID requirements, or more complex pilot licensing. By focusing on this market segment, WingFC enables hobbyists to build and fly FPV aircraft that can be legally operated in more places and with fewer bureaucratic hurdles, significantly lowering the barrier to entry for beginners and experienced pilots alike.


Powered by TinyGo, a Go compiler for microcontrollers, WingFC offers a robust and stable software platform. The project provides essential flight control features for elevon-equipped fixed-wing aircraft, including:

- **Stabilization Modes**: Offers flight stabilization to assist pilots, particularly for smooth, cinematic FPV footage and relaxed cruising.

- **Mixing**: Handles the complex mixing of aileron and elevator inputs for a fixed-wing flying wing platform, simplifying the aircraft's setup.

- **Safety Features**: Incorporates failsafe mechanisms to protect the aircraft and others in the event of signal loss.

- **Receiver Protocol Support**: Ensures broad compatibility with popular FPV communication standards, including FlySky's iBus, TBS's CRSF, and the open-source ELRS protocol, which is highly valued for its long-range capabilities and low latency in the FPV community.

The flight controller is engineered for a seamless integration into FPV flying wing airframes like the ZOHD Dart 250G or Alight Wing Aeronautics Flik, a prominent model in this market. The hardware's choice of the Seeed Studio Xiao nRF52840 Sense with an onboard IMU aligns with the project's goal of creating a compact, lightweight, and high-performance control system for these specific FPV drones.

### Latest Version 0.2.1

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
- Go (TinyGo)
- Supported microcontroller (see docs)
```

### Installing

[Download source code](https://github.com/BryanSouza91/WingFC/releases/tag/v0.2.1)
Extract source code then navigate to the top directory of the source.
```
cd firmware/src
```

Plug in WingFC board via USB-C 

When the filesystem shows up on your computer, flash firmware to your board

```
tinygo flash -target=xiao-ble -tags=<ibus,crsf> firmware/src
```

## Deployment

Add additional notes about how to deploy this on a live system

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

Please read [CONTRIBUTING.md](https://github.com/BryanSouza91/WingFC/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/BryanSouza91/WingFC/tags).

## Authors

• Bryan Souza - Initial work - [BryanSouza91](https://github.com/BryanSouza91)

See also the list of [contributors](https://github.com/BryanSouza91/WingFC/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/BryanSouza91/WingFC/blob/main/LICENSE) file for details

## Acknowledgments

• Hat tip to anyone whose code was used  
• Inspiration from open-source flight controllers  
• TinyGo and Go communities

## Footer

[GitHub Homepage](https://github.com/)
# WingFC

