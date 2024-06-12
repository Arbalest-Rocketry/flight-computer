import time
import board
import busio

# Initialize UART for RunCam communication on TX1 and RX0
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.1)

# CRC-8 DVB-S2 calculation function
def crc8_dvb_s2(crc, a):
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc <<= 1
    return crc & 0xFF

# Function to send command to RunCam
def send_command(uart, action_id):
    command_packet = bytearray(5)
    command_packet[0] = 0xCC  # Header
    command_packet[1] = 0x01  # RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL
    command_packet[2] = action_id  # Action ID
    command_packet[3] = crc8_dvb_s2(0, command_packet[1])  # Calculate CRC for Command ID
    command_packet[4] = crc8_dvb_s2(command_packet[3], command_packet[2])  # Calculate CRC for Action ID

    uart.write(command_packet)
    time.sleep(0.1)  # Give the camera time to process the command
    response = uart.read(32)  # Read up to 32 bytes from the camera
    if response:
        print(f"Response from camera: {response}")
    else:
        print("No response from camera")

# Send commands to RunCam
def send_turn_on_command():
    send_command(uart, 0x01)  # RCDEVICE_PROTOCOL_CAMERA_TURN_ON

def send_start_recording_command():
    send_command(uart, 0x03)  # RCDEVICE_PROTOCOL_CAMERA_START_RECORDING

def send_stop_recording_command():
    send_command(uart, 0x04)  # RCDEVICE_PROTOCOL_CAMERA_STOP_RECORDING

def send_turn_off_command():
    send_command(uart, 0x02)  # RCDEVICE_PROTOCOL_CAMERA_TURN_OFF

# Main loop to send commands and check responses
while True:
    print("Sending turn on command to camera")
    send_turn_on_command()
    time.sleep(2)
    
    print("Sending start recording command to camera")
    send_start_recording_command()
    time.sleep(2)
    
    print("Sending stop recording command to camera")
    send_stop_recording_command()
    time.sleep(2)
    
    print("Sending turn off command to camera")
    send_turn_off_command()
    time.sleep(2)