# WingFC

WingFC is an open-source embedded flight controller for fixed-wing aircraft, designed for TinyGo. It provides stabilization, mixing, and safety features for elevon-equipped models, with a focus on reliability and ease of use.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
- Go (TinyGo)
- Supported microcontroller (see docs)
- Serial terminal for flashing
```

### Installing

A step by step series of examples that tell you how to get a development env running

```
git clone https://github.com/BryanSouza91/WingFC.git
cd WingFC
tinygo build -target <your-board> -o firmware.hex
```

And repeat

```
Flash stabServo.hex to your board using your preferred tool
Connect hardware as described in the documentation
```

End with an example of getting some data out of the system or using it for a little demo

```
Open serial terminal to view debug output and verify sensor readings
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


## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/BryanSouza91/WingFC/tags).

## Authors

• Bryan Souza - Initial work - [BryanSouza91](https://github.com/BryanSouza91)

See also the list of [contributors](https://github.com/BryanSouza91/WingFC/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/BryanSouza91/WingFC/blob/main/LICENSE.md) file for details

## Acknowledgments

• Hat tip to anyone whose code was used  
• Inspiration from open-source flight controllers  
• TinyGo and Go communities

## Footer

[GitHub Homepage](https://github.com/)
# WingFC

