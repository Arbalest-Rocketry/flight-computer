# The Arbalest Flight Computer Software by `Leroy Musa`

This repository contains the code and documentation for the flight computer used for our 2 Stage High Power Rocket set to launch at Launch Canada 2024 

<p align="center">
    <img src="images/Aerostructure.png" alt="Image showing me holding our COTS Flight Computer" width="75%" style="display: inline-block; margin-right: 10px;">
</p>

## Components
- Microcontroller: [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
- Sensors:
  - Altimeter: [Adafruit BMP280](https://www.adafruit.com/product/2651)
  - IMU (Inertial Measurement Unit): [Adafruit BNO055](https://www.adafruit.com/product/4646)
  - GPS Module: [SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/products/18037)
  - SD Breakout: [Adafruit SPI Flash SD Card - XTSD 512 MB](https://www.adafruit.com/product/4899)
- Communication: [Adafruit LoRa RFM9X](https://www.adafruit.com/product/3072)
- Our COTS (Commercial Off-The-Shelf) Flight Computer: [Featherweight Blue Raven](https://www.featherweightaltimeters.com/blue-raven-altimeter.html)
(left)
- Our COTS GPS: [Featherweight GPS Tracker](https://www.featherweightaltimeters.com/featherweight-gps-tracker.html) (right)
<div style="text-align: center;">
  <img src="images/cots.jpg" alt="Image showing me holding our COTS Flight Computer" width="20%" style="display: inline-block; margin-right: 10px;">
  <img src="images/Featherweight GS v2 opened.jpg" alt="Electronics Mount CAD Design" width="35%" style="display: inline-block;">
  <img src="images/Featherweight v2 with tracker.jpg" alt="Electronics Mount CAD Design" width="20%" style="display: inline-block;">
  <img src="images/featherweight_alt_gps.png" alt="Electronics Mount CAD Design" width="80%" style="display: inline-block;">
</div>

## Electronics Mount CAD Design by `Caelan Babenko`
The electronics mount CAD design showcases the placement of PCBs within the rocket's fuselage. This design ensures the flight computer's and associated components' proper integration and protection.

<div style="text-align: center;">
  <img src="images/electronics_mount_cad.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block; margin-right: 10px;">
  <img src="images/electronics_mount_cad_2.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block;">
  <img src="images/electronics_mount_cad_3.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block;">
</div>

## Testing Setup
Here's an image showing my testing setup for the flight computer(left) and ground station(right):

<div style="text-align: center;">
  <img src="images/test_setup_1.jpg" alt="Testing Setup" width="45%" style="display: inline-block;">
  <img src="images/test_setup_2.jpg" alt="Testing Setup" width="45%" style="display: inline-block;">
</div>

## PCBv2.0 Schematics by `Baseer Yousufzai`
Pictures showing our current **Arbalest Flight Computer** schematics layout
<div style="text-align: center;">
  <img src="images/FlightComputer_sch.png" alt="Arbalest Flight Computer Demo" width="100%" style="display: inline-block;">
</div>
<div style="text-align: center;">
  <img src="images/PCBv2.0_Gerber.png" alt="PCB Gerber" width="32.%" style="display: inline-block;">
  <img src="images/PCB_front.png" alt="PCB 3D Front" width="32.%" style="display: inline-block;">
  <img src="images/PCB_back.png" alt="PCB 3D Rear" width="32.%" style="display: inline-block;">
</div>

## IRLv2.0
<div style="text-align: center;"></div>

## PCBv1.0 Schematics by `Baseer Yousufzai`
Pictures showing v1.0 of our **Arbalest Flight Computer** schematics layout
<div style="text-align: center;">
  <img src="images/flightcomputer_sch.png" alt="Arbalest Flight Computer Demo" width="100%" style="display: inline-block;">
</div>
<div style="text-align: center;">
  <img src="images/PCB_Gerber.png" alt="PCB Gerber" width="32%" style="display: inline-block;">
  <img src="images/PCB_Layout_Frontal.png" alt="PCB 3D Front" width="32%" style="display: inline-block;">
  <img src="images/PCB_Layout_Rear.png" alt="PCB 3D Rear" width="32%" style="display: inline-block;">
</div>

## IRLv1.0
<div style="text-align: center;">
  <img src="images/gems/Leroy_holds_pcb_1.jpg" alt="PCB IRL Front" width="47%" style="display: inline-block;">
  <img src="images/gems/Leroy_holds_pcb_2.png" alt="PCB IRL Front" width="47%" style="display: inline-block;">
</div>

## How it works 
Simple Illustration 
<div style="text-align: center;">
  <img src="images/fc_illustration.png" alt="Flight Computer Illustration" width="100%" style="display: inline-block;">
</div>

## Arbalest Flight Computer Integration Guide

### Overview

This document provides a detailed description of the Arbalest Flight Computer setup as depicted in the schematic. The system integrates four 3.7V 650mAh LiPo batteries, a screw switch, and three Runcam Split 4 cameras connected to the Arbalest Flight Computer. The flight computer controls four pyro channels specifically designed for parachute deployment.

### Components

1. **Arbalest Flight Computer**
2. **3.7V 650mAh LiPo Batteries (4x)**
3. **Screw Switch**
4. **Runcam Split 4 Cameras (3x)**
5. **Pyro Channels (4x) for Parachute Deployment**

### Wiring Diagram

The wiring diagram consists of the following connections:
- Power supply wiring from the batteries
- Activation of switch wiring
- Camera connections
- Pyro channel connections for parachute deployment

### Power Supply

The power supply section is crucial for powering the flight computer and the connected components.

1. **LiPo Batteries:**
   - Four 3.7V 650mAh LiPo batteries are wired in series parallel to form a 7.4V 1950mAh power pack.
   - **Series Connection:** Two pairs of two batteries connected in series, resulting in two 7.4V packs.
   - **Parallel Connection:** These two 7.4V packs are connected in parallel to increase the capacity to 1950mAh.

2. **Wiring:**
   - The positive terminal of the first battery is connected to the positive terminal of the second battery.
   - The negative terminal of the second battery is connected to the positive terminal of the third battery.
   - The negative terminal of the third battery is connected to the positive terminal of the fourth battery.
   - The negative terminal of the fourth battery is connected back to the system ground.

3. **Screw Switch:**
   - The main power from the batteries is routed through a screw switch to control the power supply to the entire system.
   - The positive wire from the battery pack connects to one terminal of the screw switch.
   - The other terminal of the screw switch connects to the power input of the Arbalest Flight Computer.


### Pyro Channels for Parachute Deployment

The flight computer controls four pyro channels for parachute deployment.

1. **Connections:**
   - Each pyro channel (1 to 4) has two wires: one connected to the flight computer and the other to the pyro device.
   - The flight computer sends a signal to activate the pyrotechnic devices through these wires.

## Conclusion

The integration of the Arbalest Flight Computer with the LiPo batteries, screw switch, Runcam Split 4 cameras, and pyro channels provides a robust system for managing various flight operations, including parachute deployment. 

# Connecting Runcam Split 4 Cameras to the Testing Setup

To simulate real-flight conditions and verify the functionality of the Runcam Split 4 cameras, a direct connection to the testing setup is essential. This section outlines the straightforward process of integrating Runcam Split 4 cameras into your testing environment for video recording and analysis.

<div style="text-align: center;">
  <img src="images/RunCamSplit4_compared_to_palm.jpg" alt="RCS4" width="25%" style="display: inline-block;">
  <img src="images/RunCamSplit4_isolated.jpg" alt="RCS4" width="33.3%" style="display: inline-block;">
  <img src="images/runcamsplit4_setup.jpg" alt="RCS4" width="25%" style="display: inline-block;">
</div>

### Step 1: Gather Necessary Components

Ensure you have the following components ready:

- Runcam Split 4 camera module
- Teensy 4.1 microcontroller board
- Jumper wires
- Soldering equipment

### Step 2: Physical Connection

1. **Connect Power**: Use jumper wires to connect the power source to the Teensy 4.1 to provide power to the entire setup.
2. **Connect Camera**: Solder jumper wires onto the RX (receive) and TX (transmit) pins of the Runcam Split 4 camera. Then, you can connect the RX wire to the TX pin and the TX wire to the RX pin on the Teensy 4.1 for video data transmission.
3. **Ground Connection**: Connect another jumper wire from the ground (GND) pin of the Runcam Split 4 to any ground pin on Teensy 4.1 to complete the circuit.

# Transceiver Test w/ RC plane
<div style="text-align: center;">
  <img src="images/IMG_4160.jpg" alt="RC Plane w/ Flight Computer on board" width="30%" style="display: inline-block;">
  <img src="images/IMG_4161.jpg" alt="RC Plane w/ Flight Computer on board" width="30%" style="display: inline-block;">
  <img src="images/IMG_4162.jpg" alt="RC Plane w/ Flight Computer on board" width="30%" style="display: inline-block;">
</div>


## Low Noise Amplifiers

The low noise amplifiers (LNAs) are crucial components in the Arbalest flight computer's communication system. They are used to amplify weak signals received from the rocket, ensuring they are strong enough to be processed and analyzed. The following images show the setup of the LNAs connected to the Teensy 4.1 microcontroller board:

<div style="text-align: center;">
  <img src="images/LoRaLNA.jpg" alt="LoRaLNA" width="31%" style="display: inline-block;">
  <img src="images/LNA1.jpg" alt="LNA1" width="31%" style="display: inline-block;">
  <img src="images/LNA2.jpg" alt="LNA2" width="31%" style="display: inline-block;">
</div>

### Function of the Low Noise Amplifiers

Low noise amplifiers are designed to amplify weak signals received by the antenna with minimal addition of noise. This is essential in communication systems where signal strength can be significantly reduced due to distance, atmospheric conditions, and other interference factors.

### How the Amplifier and Filter Work

The LNA circuit shown in the images is connected to the LoRa (Long Range) communication module to improve the signal-to-noise ratio. Here’s a detailed explanation of the process:

1. **Signal Reception:**
   - The weak signal received by the antenna is fed into the LNA. This signal is typically very weak and can be easily lost in the noise.

2. **Amplification:**
   - The LNA amplifies the weak signal. The primary goal is to increase the signal strength without adding significant noise. This is achieved through the careful design of the amplifier circuit, using high-quality components that contribute minimal noise.

3. **Filtering:**
   - After amplification, the signal may pass through a filter to remove unwanted frequencies or noise that might have been introduced during amplification. This ensures that only the desired signal is sent to the next stage of the communication system.

4. **Enhanced Signal:**
   - The enhanced signal, now much stronger and clearer, is fed into the LoRa module. The LoRa module can now process this signal more effectively, providing reliable communication over long distances.

### Connecting to LoRa for Increased Gain

The LoRa communication module is known for its ability to transmit data over long distances with low power consumption. By connecting the output of the LNA to the LoRa module, we achieve the following benefits:

- **Increased Range:**
  - The amplified signal can travel greater distances without degradation, improving the overall range of the communication system.

- **Improved Signal Quality:**
  - The higher signal-to-noise ratio achieved through amplification and filtering results in better signal quality, reducing the likelihood of errors in data transmission.

- **Enhanced Reliability:**
  - With stronger and clearer signals, the communication link becomes more reliable, which is critical for mission success in rocketry.

---
# To Infinity and Beyond
<p align="center">
  <img src="images/LC_Patch_2024.png" alt="LC 2024 logo" width="33.33%">
</p>


---
# Authors
- [Leroy Musa](mailto:musaleroyohio@gmail.com) - Author & Maintainer
