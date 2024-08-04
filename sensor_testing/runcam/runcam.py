import serial
import time

# CRC8 calculation function
def crc8_dvb_s2(crc, a):
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc = crc << 1
    return crc & 0xFF

# Initialize serial connection to Teensy
ser = serial.Serial(
    port='/dev/ttyACM0',  # Replace with the correct serial port
    baudrate=115200,
    timeout=1
)

def send_command(command):
    checksum = 0
    for byte in command:
        checksum = crc8_dvb_s2(checksum, byte)
    command.append(checksum)
    print(f'Sending command: {command}')
    ser.write(bytearray(command))
    time.sleep(0.1)
    response = ser.read_all()
    print(f'Response: {response}')
    return response

def get_camera_info():
    # Command to get camera information (3 bytes)
    command = [0xCC, 0x00]
    response = send_command(command)
    print(f'Camera info response: {response}')

def start_video_recording():
    # Command to start video recording (4 bytes)
    command = [0xCC, 0x01, 0x03]
    response = send_command(command)
    print(f'Start video recording response: {response}')

def stop_video_recording():
    # Command to stop video recording (4 bytes)
    command = [0xCC, 0x01, 0x04]
    response = send_command(command)
    print(f'Stop video recording response: {response}')

def simulate_power_button():
    # Command to simulate power button press (4 bytes)
    command = [0xCC, 0x01, 0x01]
    response = send_command(command)
    print(f'Simulate power button response: {response}')

def simulate_wifi_button():
    # Command to simulate Wi-Fi button press (4 bytes)
    command = [0xCC, 0x01, 0x00]
    response = send_command(command)
    print(f'Simulate Wi-Fi button response: {response}')

if __name__ == "__main__":
    try:
        # Example usage
        get_camera_info()
        simulate_power_button()
        start_video_recording()
        time.sleep(5)  # Record for 5 seconds
        stop_video_recording()
        simulate_wifi_button()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()