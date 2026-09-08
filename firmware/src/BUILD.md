# **WingFC Firmware Build Instructions**

This document outlines the steps required to build the WingFC firmware for different Radio Control (RC) protocols using TinyGo.

## **Prerequisites**

Before you begin, ensure you have the following tools installed and configured on your system:

* **Go:** The Go programming language, version 1.24 or later.  
* **TinyGo:** The TinyGo compiler, installed and set up for your target hardware.

## **Configuration with YAML Profiles**

WingFC aircraft profiles are defined in YAML format under the `configs/` directory. You can choose a pre-existing profile or create your own custom aircraft profile.

### **List Available Profiles**
```bash
go run ./tools/configgen -list
```

### **Select an Airframe Profile**
Apply an airframe profile to generate `config.go`:
```bash
# Flying wing (e.g. ZOHD Dart 250G, Flik)
go run ./tools/configgen -profile elevon_wing

# Conventional T-tail airframe
go run ./tools/configgen -profile t_tail

# V-tail airframe
go run ./tools/configgen -profile v_tail

# Custom YAML configuration file
go run ./tools/configgen -config configs/my_plane.yaml
```

To re-generate from the default configuration:
```bash
go generate ./...
```

For full details on the YAML schema and parameters, see [configs/README.md](file:///home/bryansouza/Repos/WingFC/configs/README.md).

## **Multi-Protocol Support**

The WingFC firmware supports multiple RC receiver protocols, specifically iBus and CRSF. Each protocol is implemented in a separate Go file (`ibus.go` and `crsf.go`) and uses a [Go build tag](https://pkg.go.dev/cmd/go#hdr-Build_tags) to enable or disable its inclusion during compilation.

TinyGo allows for building and flashing through a single `flash` command.
To build the firmware for a specific protocol, you must include the corresponding build tag in the tinygo `flash` command using the `-tags` flag.

## **Flash Commands**

Once your Xiao nrf52840 Sense is plugged in, quickly press the reset button twice to connect the bootloader. 
The Xiao should show up as a USB device and is now ready to flash.
Use one of the following commands to build and flash the firmware for your desired protocol.

### **Flash for iBus Protocol**

This command compiles the firmware with iBus protocol support enabled:
```bash
tinygo flash -target=xiao-ble -tags=ibus .
```

### **Flash for CRSF Protocol**

This command compiles the firmware with CRSF protocol support enabled:
```bash
tinygo flash -target=xiao-ble -tags=crsf .
```

## **Build Commands**

To build the firmware for a specific protocol, you must include the corresponding build tag in the tinygo build command using the `-tags` flag.
Use one of the following commands to build the firmware for your desired protocol.

### **Build for iBus Protocol**

This command compiles the firmware with iBus protocol support enabled:
```bash
tinygo build -o wingfc-ibus.hex -target=xiao-ble -tags=ibus .
```

### **Build for CRSF Protocol**

This command compiles the firmware with CRSF protocol support enabled:
```bash
tinygo build -o wingfc-crsf.hex -target=xiao-ble -tags=crsf .
```

### **Explanation of Flags**

* `-o <filename>`: Specifies the output filename for the compiled firmware. We use a different name for each protocol for clarity.  
* `-target=<board>`: Specifies the target hardware board, the Xiao nrf52840 Sense.
* `-tags=<protocol>`: This is the critical flag for protocol selection. It tells the compiler which protocol file to include in the build.  
* `.`: The final dot indicates that the source code is in the current directory (`WingFC/firmware/src`).
