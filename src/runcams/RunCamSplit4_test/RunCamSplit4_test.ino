#define BUFF_SIZE 20
#define HWSERIAL Serial1

uint8_t txBuf[BUFF_SIZE], crc;
int recState = 0;

void setup(void) 
{
    Serial.begin(9600);  
    HWSERIAL.begin(115200);
    //give the runcam time to send whatever it does out of reset
    delay(3000);

    //serial command to toggle recording
    //more info on the serial packet structure --> https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
    txBuf[0] = 0xCC;
    txBuf[1] = 0x01;
    txBuf[2] = 0x01;  
    txBuf[3] = calcCrc(txBuf, 3);  //compute the CRC
    Serial.println(txBuf[3]);

    // Start recording after 10 seconds
    delay(10000);
    startRecording();
}

void loop() 
{
    // Nothing to do in the loop for this example
}

void startRecording() 
{
    if(recState == 0) 
    {
        Serial.println("Starting Recording");
        recState = 1;
        HWSERIAL.write(txBuf, 4);
    }
}

void stopRecording() 
{
    if(recState == 1)
    {
        Serial.println("Stopping Recording");
        recState = 0;
        HWSERIAL.write(txBuf, 4);
    }
}

uint8_t calcCrc(uint8_t *buf, uint8_t numBytes)
{
    uint8_t crc = 0;
    for(uint8_t i = 0; i < numBytes; i++)
        crc = crc8_calc(crc, *(buf + i), 0xd5);
        
    return crc;
}

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly)
{
    crc ^= a;
    for(int ii = 0; ii < 8; ++ii) 
    {
        if(crc & 0x80)
            crc = (crc << 1) ^ poly;
        else
            crc = crc << 1;
    }
    
    return crc;
}