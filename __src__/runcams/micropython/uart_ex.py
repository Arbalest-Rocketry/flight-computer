import machine

# Initialize UART1 for communication on pins TX1 (pin 1) and RX1 (pin 0)
try:
    uart1 = UART(1, baudrate=9600, tx=1, rx=0)
    print("UART1 initialized successfully")
except ValueError as e:
    print("Error initializing UART1:", e)

# Test sending a simple message
try:
    uart1.write(b'Hello, World!\n')
    print("Message sent successfully")
except Exception as e:
    print("Error sending message:", e)
