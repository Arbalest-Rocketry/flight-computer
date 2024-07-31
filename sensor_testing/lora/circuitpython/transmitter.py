import time
import board
import busio
import digitalio
import adafruit_rfm9x

# Define radio parameters
RADIO_FREQ_MHZ = 915.0  # Set this to 433.0 if you're using a 433MHz module

# Define pins connected to the chip
CS = digitalio.DigitalInOut(board.D1)  # CS pin is 1
RESET = digitalio.DigitalInOut(board.D34)  # RST pin is 34
# INT pin is 8, but typically it's used for IRQ (not directly used in this script)

# Initialize SPI bus
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialize RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# Set transmit power (default is 13dB, can go up to 23dB)
rfm9x.tx_power = 23

# Enable CRC checking
rfm9x.enable_crc = True

# Send packets periodically
while True:
    rfm9x.send(bytes("Hello from sender!\r\n", "utf-8"))
    print("Sent: Hello from sender!")
    time.sleep(1)  # Send a packet every second
