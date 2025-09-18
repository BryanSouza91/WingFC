# WingFC

WingFC is an open-source embedded flight controller for fixed-wing aircraft, designed for TinyGo. It provides stabilization, mixing, and safety features for elevon-equipped models, with a focus on reliability and ease of use.

### Latest Version 0.1.1

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
- Go (TinyGo)
- Supported microcontroller (see docs)
```

### Installing

[Download source code](https://github.com/BryanSouza91/WingFC/releases/tag/v0.1.1)
Extract source code then navigate to the top directory of the source.
```
cd firmware/src
```

Plug in WingFC board via USB-C 

When the filesystem shows up on your computer, flash firmware to your board

```
tinygo flash -target=xiao-ble 
```

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
go test ./...
```

### And coding style tests

Explain what these tests test and why

```
go vet
golint
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

