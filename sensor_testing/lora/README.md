# LoRa RFM9X

This project demonstrates how to use Adafruit's RFM9X LoRa packet radio breakouts with Arduino and CircuitPython to transmit and receive data over long distances.

## Overview

The RFM9X LoRa module enables long-range wireless communication using LoRa modulation. This project includes examples for both Arduino and CircuitPython to help you get started.

## Pinouts

### Power Pins

- **Vin**: Power in (3.3-6VDC)
- **GND**: Ground
- **EN**: Enable pin (pull low to cut power to the radio)

### SPI Logic Pins

- **SCK**: SPI Clock
- **MISO**: Microcontroller In Serial Out
- **MOSI**: Microcontroller Out Serial In
- **CS**: Chip Select
- **RST**: Reset
- **G0**: GPIO 0 (interrupt request notification)

### Radio GPIO

- Five additional GPIO pins for various functions

### Antenna Connection

- Options include wire antenna, uFL connector, or SMA edge-mount connector

## Assembly

1. **Prepare the Header Strip**: Cut the strip to length if necessary and insert it into a breadboard with long pins down.
2. **Add the Breakout Board**: Place the breakout board over the pins so that the short pins poke through the breakout pads.
3. **Solder**: Solder all pins for reliable electrical contact.
4. **Antenna Options**: Choose from a wire antenna, uFL connector, or SMA edge-mount connector, and connect it appropriately.

## Arduino Wiring

Wire the radio in SPI mode using the hardware SPI port of your Arduino. Connect the power, SPI logic, and control pins accordingly.

## Using the RFM9x LoRa Radio

### Arduino Libraries

Use the RadioHead library for basic RX and TX examples.

### Basic RX & TX Example

- **Transmitter**: Sends a packet of data once a second.
- **Receiver**: Receives the packet and sends a reply.

## CircuitPython for RFM9x LoRa

Follow the instructions for wiring and using the RFM9x LoRa module with CircuitPython, including initialization, basic send/receive functions, and advanced usage with reliable datagram.