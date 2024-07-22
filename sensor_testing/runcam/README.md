
## Purpose
To provide a method for debugging and remotely controlling the RunCam Split 4 via a Teensy 4.1.

## Process

Refer to the main README for hardware setup instructions.

### Teensy Sketch

The Teensy sketch initializes serial communication, receives commands from the Python script, and forwards them to the RunCam. It also reads responses from the RunCam and sends them back to the Python script.

```cpp
void setup() {
    Serial.begin(115200);  // USB serial
    Serial1.begin(115200); // Serial1 for RunCam communication
    while (!Serial) {
        ; // Wait for Serial to be ready
    }
    Serial.println("Teensy Ready");
}
```

### Python Script

The Python script sends formatted commands to the Teensy via a serial connection. It uses a CRC8 checksum for data integrity.

```python
import serial

ser = serial.Serial(
    port='/dev/ttyACM0',  # Replace with the correct serial port
    baudrate=115200,
    timeout=1
)

def send_command(command):
    ser.write(bytearray(command))
    response = ser.read_all()
    print(f'Response: {response}')
    return response
```

## Example Serial Monitor Output

```
15:21:43.540 -> Teensy Ready
15:21:55.408 -> Received from Python: CC 0 0
15:21:55.408 -> Sent to RunCam: CC 0 0 96
15:22:01.516 -> Received from RunCam: CC 1 0 1 2B
15:22:10.516 -> Received from Python: CC 1 3
15:22:10.516 -> Sent to RunCam: CC 1 3 67
15:22:20.516 -> Received from RunCam: CC 0 1 3A
```

## Setup Instructions
1. **Install Python dependencies**:

   ```bash
   pip install pyserial
   ```

2. **Upload the Teensy sketch**:
   - Open the Arduino IDE.
   - Load the `runcam_control.ino` sketch.
   - Select the Teensy 4.1 board and the correct port.
   - Upload the sketch.

3. **Run the Python script**:

   ```bash
   python3 runcam_control.py
   ```