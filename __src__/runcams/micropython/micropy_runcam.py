from machine import UART, Pin
import time

# Initialize UART for RunCam communication
uart1 = UART(1, baudrate=9600, tx=Pin(1), rx=Pin(0))  # Adjust the pins as needed
uart2 = UART(2, baudrate=9600, tx=Pin(8), rx=Pin(7))  # Adjust the pins as needed
uart3 = UART(3, baudrate=9600, tx=Pin(14), rx=Pin(15))  # Adjust the pins as needed

# CRC-16-CCITT calculation function
def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc & 0xFFFF

# CRC-8 DVB-S2 calculation function
def crc8_dvb_s2(crc, a):
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc <<= 1
    return crc & 0xFF

# Send command function
def send_command(uart, action_id):
    command_packet = bytearray(5)
    command_packet[0] = 0xCC  # Header
    command_packet[1] = 0x01  # RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL
    command_packet[2] = action_id  # Action ID
    command_packet[3] = crc8_dvb_s2(0, command_packet[1])  # Calculate CRC for Command ID
    command_packet[4] = crc8_dvb_s2(command_packet[3], command_packet[2])  # Calculate CRC for Action ID

    # Send the command packet over UART
    uart.write(command_packet)

# Example commands
def send_turn_on_command():
    send_command(uart1, 0x01)  # RCDEVICE_PROTOCOL_CAMERA_TURN_ON
    #send_command(uart2, 0x01)  # RCDEVICE_PROTOCOL_CAMERA_TURN_ON
    #send_command(uart3, 0x01)  # RCDEVICE_PROTOCOL_CAMERA_TURN_ON

def send_start_recording_command():
    send_command(uart1, 0x03)  # RCDEVICE_PROTOCOL_CAMERA_START_RECORDING
    #send_command(uart2, 0x03)  # RCDEVICE_PROTOCOL_CAMERA_START_RECORDING
    #send_command(uart3, 0x03)  # RCDEVICE_PROTOCOL_CAMERA_START_RECORDING

def send_stop_recording_command():
    send_command(uart1, 0x04)  # RCDEVICE_PROTOCOL_CAMERA_STOP_RECORDING
    #send_command(uart2, 0x04)  # RCDEVICE_PROTOCOL_CAMERA_STOP_RECORDING
    #send_command(uart3, 0x04)  # RCDEVICE_PROTOCOL_CAMERA_STOP_RECORDING

def send_turn_off_command():
    send_command(uart1, 0x02)  # RCDEVICE_PROTOCOL_CAMERA_TURN_OFF
    #send_command(uart2, 0x02)  # RCDEVICE_PROTOCOL_CAMERA_TURN_OFF
    #send_command(uart3, 0x02)  # RCDEVICE_PROTOCOL_CAMERA_TURN_OFF

# Example usage
send_turn_on_command()
time.sleep(2)
send_start_recording_command()
time.sleep(2)
send_stop_recording_command()
time.sleep(2)
send_turn_off_command()