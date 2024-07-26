# Sensors Integration with CircuitPython

This project demonstrates the integration of BNO055 and BMP280 sensors with a Teensy 4.1 board using CircuitPython. The sensors are connected via I2C, and the data is read and printed to the serial output. Additionally, data is sent via an RFM9x LoRa radio module and logged to an SD card.

## Setup Instructions

Follow these steps to configure CircuitPython on your Teensy 4.1 and get the sensors working:

### 1. Install CircuitPython on Teensy 4.1

1. Download the latest CircuitPython firmware for the Teensy 4.1 from [here](https://circuitpython.org/board/teensy41/). You can choose between `.hex` and `.uf2` file formats. This guide will use the `.hex` file format, but the `.uf2` format can also be used.

2. Connect your Teensy 4.1 to your computer via USB.

3. Put the Teensy into bootloader mode by pressing the reset button on the Teensy. The LED on the Teensy should turn on.

4. Open a terminal and navigate to the directory where the firmware was downloaded.

5. Copy the firmware to the Teensy 4.1:

   ```sh
   sudo cp ~/Downloads/adafruit-circuitpython-teensy41-en_US-9.0.5.hex /media/$USER/TEENSY
   ```

6. If you see an error like "No such file or directory", use the following command to find the correct mount point:

   ```sh
   sudo dmesg | tail
   ```

7. Once the Teensy reboots, it should mount as a `CIRCUITPY` drive.

### 2. Download Required Libraries

1. Download the latest CircuitPython library bundle from [here](https://circuitpython.org/libraries).

2. Extract the library bundle:

   ```sh
   unzip ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723.zip -d ~/Downloads/
   ```

3. Copy the necessary libraries to the `CIRCUITPY` drive:

   ```sh
   sudo cp ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_bno055.mpy /media/$USER/CIRCUITPY/lib/
   sudo cp ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_bmp280.mpy /media/$USER/CIRCUITPY/lib/
   sudo cp ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_rfm9x.mpy /media/$USER/CIRCUITPY/lib/
   sudo cp ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_sdcard.mpy /media/$USER/CIRCUITPY/lib/
   sudo cp -r ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_bus_device /media/$USER/CIRCUITPY/lib/
   sudo cp -r ~/Downloads/adafruit-circuitpython-bundle-9.x-mpy-20240723/lib/adafruit_register /media/$USER/CIRCUITPY/lib/
   ```

### 3. Configure Teensy Loader

To upload the CircuitPython firmware using the Teensy Loader:

1. Download the Teensy Loader for Linux from [here](https://www.pjrc.com/teensy/loader_linux.html):

   ```sh
   sudo wget https://www.pjrc.com/teensy/teensy_linux64.tar.gz -O teensy_loader_linux64.tar.gz
   ```

2. Extract the Teensy Loader:

   ```sh
   sudo mkdir /media/$USER/teensy_loader
   sudo tar -xzf teensy_loader_linux64.tar.gz -C /media/$USER/teensy_loader
   ```

3. Run the Teensy Loader with the following command:

   ```sh
   sudo /media/$USER/teensy_loader/teensy
   ```

### 4. Copy `pcb.py` File

1. Copy the `pcb.py` file from the same directory as this `README.md` to the `CIRCUITPY` drive:

   ```sh
   sudo cp ~/path-to-your-repo/pcb.py /media/$USER/CIRCUITPY/
   ```

2. Eject the `CIRCUITPY` drive:

   ```sh
   sudo eject /media/$USER/CIRCUITPY
   ```

### 5. Running the Code

1. Open a terminal and monitor the serial output from the Teensy using `screen`:

   ```sh
   screen /dev/ttyACM0 115200  # Adjust the device name as necessary
   ```

2. The output should display the sensor readings, send data via the RFM9x, and log data to the SD card.

### Additional Resources

- [Adafruit BNO055 Library Documentation](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor)
- [Adafruit BMP280 Library Documentation](https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout)
- [Adafruit RFM9x LoRa Documentation](https://learn.adafruit.com/adafruit-rfm9x-lora-bonnet-and-featherwing)
- [Adafruit SMT SD Card Breakout](https://www.adafruit.com/product/4899)