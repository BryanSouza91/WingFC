# **WingFC Firmware Build Instructions**

This document outlines the steps required to build the WingFC firmware for different Radio Control (RC) protocols using TinyGo.

## **Prerequisites**

Before you begin, ensure you have the following tools installed and configured on your system:

* **Go:** The Go programming language, version 1.24 or later.  
* **TinyGo:** The TinyGo compiler, installed and set up for your target hardware.

## **Multi-Protocol Support**

The WingFC firmware supports multiple RC receiver protocols, specifically iBus and CRSF. Each protocol is implemented in a separate Go file (`ibus.go` and `crsf.go`) and uses a [Go build tag](https://www.google.com/search?q=https://pkg.go.dev/cmd/go%23hdr-Build_tags) to enable or disable its inclusion during compilation.

To build the firmware for a specific protocol, you must include the corresponding build tag in the tinygo build command using the \-tags flag.

## **Build Commands**

Use one of the following commands to build the firmware for your desired protocol.

### **Build for iBus Protocol**

This command compiles the firmware with iBus protocol support enabled.

`tinygo build -o wingfc-ibus.hex -target=xiao-ble -tags=ibus .`

### **Build for CRSF Protocol**

This command compiles the firmware with CRSF protocol support enabled.

`tinygo build -o wingfc-crsf.hex -target=xiao-ble -tags=crsf .`

### **Explanation of Flags**

* `-o <filename>`: Specifies the output filename for the compiled firmware. We use a different name for each protocol for clarity.  
* `-target=<board>`: Specifies the target hardware board, the Xiao nrf52840 Sense.
* `-tags=<protocol>`: This is the critical flag for protocol selection. It tells the compiler which protocol file to include in the build.  
* `.`: The final dot indicates that the source code is in the current directory.