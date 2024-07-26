import time
import board
import busio
import digitalio
from adafruit_bno055 import BNO055_I2C
from adafruit_bmp280 import Adafruit_BMP280_I2C
from adafruit_rfm9x import RFM9x
import adafruit_sdcard
import storage

# Initialize I2C with a lower frequency
i2c = busio.I2C(board.SCL, board.SDA, frequency=50000)  # 50kHz

# Initialize BNO055
bno055 = BNO055_I2C(i2c)

# Leroy Remember! This is important: use the correct I2C address for BMP280
bmp280 = Adafruit_BMP280_I2C(i2c, address=0x76)

# Initialize SPI for RFM9x and SD card
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize RFM9x
cs = digitalio.DigitalInOut(board.D1)
reset = digitalio.DigitalInOut(board.D34)
rfm9x = RFM9x(spi, cs, reset, 915.0)

# Initialize SD card
sd_cs = digitalio.DigitalInOut(board.D10)
sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

while True:
    try:
        # Print BNO055 data
        print("BNO055 Temperature: {} degrees C".format(bno055.temperature))
        print("BNO055 Accelerometer (m/s^2): {}".format(bno055.acceleration))
        print("BNO055 Magnetometer (microteslas): {}".format(bno055.magnetic))
        print("BNO055 Gyroscope (rad/sec): {}".format(bno055.gyro))
        print("BNO055 Euler angle: {}".format(bno055.euler))
        print("BNO055 Quaternion: {}".format(bno055.quaternion))
        print("BNO055 Linear acceleration (m/s^2): {}".format(bno055.linear_acceleration))
        print("BNO055 Gravity (m/s^2): {}".format(bno055.gravity))
    except OSError as e:
        print("Error accessing BNO055 sensor:", e)

    try:
        # Print BMP280 data
        print("BMP280 Temperature: {} degrees C".format(bmp280.temperature))
        print("BMP280 Pressure: {} hPa".format(bmp280.pressure))
    except OSError as e:
        print("Error accessing BMP280 sensor:", e)

    # Send a packet with RFM9x
    try:
        rfm9x.send(bytes("Hello from RFM9x!\r\n", "utf-8"))
        print("Sent: Hello from RFM9x!")
    except Exception as e:
        print("Error sending RFM9x packet:", e)
    
    # Write to SD card
    try:
        with open("/sd/data.txt", "a") as file:
            file.write("BNO055 Temperature: {} degrees C\n".format(bno055.temperature))
            file.write("BMP280 Temperature: {} degrees C\n".format(bmp280.temperature))
            file.write("BMP280 Pressure: {} hPa\n".format(bmp280.pressure))
    except Exception as e:
        print("Error writing to SD card:", e)
    
    time.sleep(1)