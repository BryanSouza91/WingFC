# WingFC Flight Controller Hardware Redesign Walkthrough

## Summary of Revisions

Based on feedback, the WingFC flight controller PCB has been comprehensively updated in [`hardware/kicad/`](file:///home/bryansouza/Repos/WingFC/hardware/kicad):

1. **Edge-Mounted North-Facing USB-C Port**:
   - Re-oriented the **Seeed Studio XIAO nRF52840 Sense Plus** (`U1`) by 180° so the USB-C receptacle faces North towards the front edge of the aircraft.
   - Added a dedicated **$12.0\,\text{mm} \times 2.0\,\text{mm}$ recessed access notch** on the front edge of the carrier board.
   - The USB-C connector receptacle now overhangs into free air by $\sim 3.5\,\text{mm}$, ensuring any standard USB-C cable plugs in without mechanical collision against the PCB.

2. **10 Actuator Channels (2 ESCs + 8 Servos)**:
   - Expanded the schematic ([`WingFC.kicad_sch`](file:///home/bryansouza/Repos/WingFC/hardware/kicad/WingFC.kicad_sch)) and netlist ([`WingFC.net`](file:///home/bryansouza/Repos/WingFC/hardware/kicad/WingFC.net)) from 7 channels to **10 channels total**:
     - **`ESC1`** (Motor 1): `D3` (`P0.29`, `PWM0` Ch 0)
     - **`ESC2`** (Motor 2 / Differential Thrust): `D8` (`P1.13`, `PWM0` Ch 1)
     - **`S1`** (Servo 1 - Left Elevon / Aileron): `D0` (`P0.02`, `PWM1` Ch 0)
     - **`S2`** (Servo 2 - Right Elevon / Elevator): `D1` (`P0.03`, `PWM1` Ch 1)
     - **`S3`** (Servo 3 - Rudder / V-Tail): `D2` (`P0.28`, `PWM1` Ch 2)
     - **`S4`** (Servo 4 - Flap / Aux): `D9` (`P1.14`, `PWM1` Ch 3)
     - **`S5`** (Servo 5 - Aux / Pan): `D10` (`P1.15`, `PWM2` Ch 0)
     - **`S6`** (Servo 6 - Flap / Tilt): `D11` (`P0.15`, `PWM2` Ch 1)
     - **`S7`** (Servo 7 - Aux / Arm): `D12` (`P0.19`, `PWM2` Ch 2)
     - **`S8`** (Servo 8 - Payload / Drop): `D13` (`P1.01`, `PWM2` Ch 3)
   - Arranged in a continuous $3 \times 10$ rear block (Signal / +5V / GND) with $2.54\,\text{mm}$ pitch.
   - Silkscreen labels rotated 90° to eliminate text overlap, with clear `S`, `+`, `-` row legends on both flanks.

3. **Outward-Facing JST-SH Connectors**:
   - Rotated both horizontal JST-SH connectors 180°:
     - **`J_RC`** (4-pin JST-SH) on the West (left) edge now has its plug insertion opening facing outward to the left.
     - **`J_GPS`** (6-pin JST-SH) on the East (right) edge now has its plug insertion opening facing outward to the right.
   - Solder pads sit securely on the board copper, while mating cables insert horizontally from outside the board.

4. **Optimized Compact Dimensions**:
   - Target mounting pattern: **$25.5 \times 25.5\,\text{mm}$ M2 pattern** (`H1`..`H4` at $\pm 12.75\,\text{mm}$ from center $(150, 100)$).
   - Board dimensions: **$34.0\,\text{mm}$ wide $\times 40.0\,\text{mm}$ long**.
   - Shortened board length from $41.5\,\text{mm}$ down to $40.0\,\text{mm}$ while fitting all 10 actuator channels.

---

## 3D Board Visualizations

````carousel
![WingFC Top 3D View](/home/bryansouza/.gemini/antigravity/brain/f0d6860b-f0ef-4c42-afec-4011cef9209b/WingFC_3D_top.png)
<!-- slide -->
![WingFC Isometric 3D View](/home/bryansouza/.gemini/antigravity/brain/f0d6860b-f0ef-4c42-afec-4011cef9209b/WingFC_3D_iso.png)
<!-- slide -->
![WingFC Bottom 3D View](/home/bryansouza/.gemini/antigravity/brain/f0d6860b-f0ef-4c42-afec-4011cef9209b/WingFC_3D_bottom.png)
````

---

## Hardware Specifications Table

| Subsystem | Component | Footprint / Package | Location / Layer | Details |
| :--- | :--- | :--- | :--- | :--- |
| **MCU & IMU** | Seeed XIAO nRF52840 Sense Plus | `KiCAD:XIAO-nRF52840-Plus-SMD` | `F.Cu` (Top Center) | 64MHz Cortex-M4F, BLE 5.0, 6-DoF IMU, PDM mic |
| **Actuators** | 10x 1x03 Pin Headers | `Connector_PinHeader_2.54mm` | `F.Cu` (Rear Shelf) | 2x ESCs (`ESC1`, `ESC2`) + 8x Servos (`S1`..`S8`) |
| **RC Receiver** | 4-Pin JST-SH Horizontal | `Connector_JST:JST_SH_SM04B...` | `F.Cu` (West Edge) | 5V, GND, UARTE0 TX/RX (CRSF / iBUS) |
| **GPS & Compass** | 6-Pin JST-SH Horizontal | `Connector_JST:JST_SH_SM06B...` | `F.Cu` (East Edge) | 5V, GND, UARTE1 TX/RX, TWISPI1 SDA/SCL |
| **BEC Power** | Monolithic Power MP2315 | `Package_TO_SOT_SMD:TSOT-23-8` | `B.Cu` (Bottom) | 5V / 2A synchronous step-down (up to 24V in) |
| **Inductor** | Bourns SRN4018 4.7µH | `Inductor_SMD:L_Bourns_SRN4018` | `B.Cu` (Bottom) | Shielded power inductor |
| **Barometer** | Infineon DPS310 | `Package_LGA:Infineon_PG-VLGA-8-1` | `B.Cu` (Bottom) | High-precision pressure sensor on TWISPI1 |
| **Protection** | Littlefuse SMAJ5.0A TVS | `Diode_SMD:D_SMA` | `B.Cu` (Bottom) | 400W transient voltage suppressor on +5V rail |
| **MCU Isolation** | Diodes Inc B0520W Schottky | `Diode_SMD:D_SOD-123` | `B.Cu` (Bottom) | Reverse current protection to +5V_MCU with 10µF cap |
| **Battery Sensing**| 100k / 10k Divider + 100nF | `0603 Metric` | `B.Cu` (Bottom Left) | 11:1 attenuation to SAADC AIN7 (`D16` / `P0.31`) |
| **Mounting** | 4x M2 Holes | `MountingHole_2.2mm_M2` | All Layers | Exact $25.5 \times 25.5\,\text{mm}$ square whoop pitch |
