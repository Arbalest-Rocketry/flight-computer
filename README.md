# The Arbalest Flight Computer for our 2 Stage High Power Rocket set to launch at Launch Canada 2024 

This repository contains the code and documentation for the flight computer used in our 2 stage rocket project.

# Components
- Microcontroller: [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
  <div style="text-align: center;">
  <img src="images/Teensy4.1_layout.png" alt="Image showing me holding our COTS Flight Computer" width="100%" style="display: inline-block; margin-right: 10px;">
</div>

- Sensors:
  - Altimeter: [Adafruit BMP280](https://www.adafruit.com/product/2651)
  - IMU (Inertial Measurement Unit): [Adafruit BNO055](https://www.adafruit.com/product/4646)
  - GPS Module: [SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/products/18037)
- Communication: [Adafruit LoRa RFM9X](https://www.adafruit.com/product/3072)
- Our COTS (Commercial Off-The-Shelf) Flight Computer: [Featherweight Blue Raven](https://www.featherweightaltimeters.com/blue-raven-altimeter.html)
(left)
- Our COTS GPS: [Featherweight GPS Tracker](https://www.featherweightaltimeters.com/featherweight-gps-tracker.html) (right)
<div style="text-align: center;">
  <img src="images/cots.jpg" alt="Image showing me holding our COTS Flight Computer" width="30%" style="display: inline-block; margin-right: 10px;">
  <img src="images/featherweight_alt_gps.png" alt="Electronics Mount CAD Design" width="60%" style="display: inline-block;">
</div>

# Electronics Mount CAD Design
The electronics mount CAD design showcases the placement of PCBs within the rocket's fuselage. This design ensures proper integration and protection of the flight computer and associated components.

<div style="text-align: center;">
  <img src="images/electronics_mount_cad.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block; margin-right: 10px;">
  <img src="images/electronics_mount_cad_2.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block;">
  <img src="images/electronics_mount_cad_3.png" alt="Electronics Mount CAD Design" width="100%" style="display: inline-block;">
</div>

Credits: Electronics Mount CAD design by Caelan Babenko

# Nosecone (in construction) & Fuselage Housing for the Electronics Mount
<div style="text-align: center;">
  <img src="images/Nosecone_Tube.jpg" alt="Nosecone image" width="100%" style="display: inline-block; margin-right: 10px;">
</div>

Credits: Nosecone rolled by Sean Hwang, Caelan Babenko, Jordan Birley |  Fuselage Housing CAD design by Caelan Babenko

# Testing Setup
Here's an image showing my testing setup for the flight computer(left) and ground station(right):

<div style="text-align: center;">
  <img src="images/test_setup_1.jpg" alt="Testing Setup" width="45%" style="display: inline-block;">
  <img src="images/test_setup_2.jpg" alt="Testing Setup" width="45%" style="display: inline-block;">
</div>

## On one board 
<div style="text-align: center;">
  <img src="images/onebreadboard.jpg" alt="Testing Setup" width="100%" style="display: inline-block;">
</div>

# PCB Schematics
Pictures showing our current **Arbalest Flight Computer** schematics layout
<div style="text-align: center;">
  <img src="images/flightcomputer_sch.png" alt="Arbalest Flight Computer Demo" width="100%" style="display: inline-block;">
</div>
<div style="text-align: center;">
  <img src="images/PCB_Gerber.png" alt="PCB Gerber" width="32%" style="display: inline-block;">
  <img src="images/PCB_Layout_Frontal.png" alt="PCB 3D Front" width="32%" style="display: inline-block;">
  <img src="images/PCB_Layout_Rear.png" alt="PCB 3D Rear" width="32%" style="display: inline-block;">
</div>

# How it works 
Simple Illustration 
<div style="text-align: center;">
  <img src="images/fc_illustration.png" alt="Flight Computer Illustration" width="100%" style="display: inline-block;">
</div>

## Arbalest Flight Computer Integration Guide

### Overview

This document provides a detailed description of the Arbalest Flight Computer setup as depicted in the provided schematic. The system integrates four 3.7V 650mAh LiPo batteries, a screw switch, and three Runcam Split 4 cameras connected to the Arbalest Flight Computer. The flight computer controls four pyro channels specifically designed for parachute deployment.

### Components

1. **Arbalest Flight Computer**
2. **3.7V 650mAh LiPo Batteries (4x)**
3. **Screw Switch**
4. **Runcam Split 4 Cameras (3x)**
5. **Pyro Channels (4x) for Parachute Deployment**

### Wiring Diagram

The wiring diagram consists of the following connections:
- Power supply wiring from the batteries
- Activation switch wiring
- Camera connections
- Pyro channel connections for parachute deployment

## Power Supply

The power supply section is crucial for powering the flight computer and the connected components.

1. **LiPo Batteries:**
   - Four 3.7V 650mAh LiPo batteries are wired in series-parallel to form a 7.4V 1950mAh power pack.
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

## Camera Connections

Three Runcam Split 4 cameras are connected to the flight computer for capturing video during flight.

1. **Power Supply:**
   - The positive and ground wires from the battery pack are connected to the power inputs of each Runcam Split 4 PCB.
   - This ensures that all cameras receive a stable 7.4V power supply.

2. **Video Signal:**
   - Each Runcam Split 4 camera is connected to its respective video input on the Arbalest Flight Computer.
   - This allows the flight computer to manage and record video from multiple angles.

## Pyro Channels for Parachute Deployment

The flight computer controls four pyro channels for parachute deployment.

1. **Connections:**
   - Each pyro channel (1 to 4) has two wires: one connected to the flight computer and the other to the pyro device.
   - The flight computer sends a signal through these wires to activate the pyrotechnic devices.

## Detailed Steps for Assembly

1. **Battery Pack Assembly:**
   - Connect the four LiPo batteries as per the series-parallel configuration.
   - Secure the connections with soldering and insulation to prevent short circuits.

2. **Switch Installation:**
   - Mount the screw switch in a convenient location on the flight system.
   - Connect the positive wire from the battery pack to one terminal of the screw switch.
   - Connect another wire from the screw switch to the power input of the flight computer.

3. **Camera Setup:**
   - Install the Runcam Split 4 cameras in their designated positions.
   - Connect the power wires from the battery pack to each camera's PCB.
   - Connect the video output from each camera to the flight computer.

4. **Pyro Channel Wiring:**
   - Connect the wires from each pyro channel on the flight computer to the corresponding pyrotechnic devices for parachute deployment.
   - Ensure that the connections are secure and insulated.

5. **Final Checks:**
   - Verify all connections for proper polarity and secure attachment.
   - Check the power supply voltage and ensure that the system is receiving the correct voltage.
   - Test the screw switch functionality to ensure it correctly controls the power to the flight computer.

## Conclusion

The integration of the Arbalest Flight Computer with the LiPo batteries, screw switch, Runcam Split 4 cameras, and pyro channels provides a robust system for managing various flight operations, including parachute deployment. 


# Transceiver Test w/ RC plane
<div style="text-align: center;">
  <img src="images/IMG_4160.jpg" alt="RC Plane w/ Flight Computer on board" width="40%" style="display: inline-block;">
  <img src="images/IMG_4163.jpg" alt="RC Plane w/ Flight Computer on board" width="50%" style="display: inline-block;">
  <img src="images/IMG_4161.jpg" alt="RC Plane w/ Flight Computer on board" width="40%" style="display: inline-block;">
  <img src="images/IMG_4162.jpg" alt="RC Plane w/ Flight Computer on board" width="40%" style="display: inline-block;">
</div>

# To Infinity and Beyond
<div style="text-align: center;">
  <img src="images/LC2022.png" alt="LC 2022 logo" width="42%" style="display: inline-block;">
  <img src="images/LC2023.png" alt="LC 2023 logo" width="40%" style="display: inline-block;">
</div>
