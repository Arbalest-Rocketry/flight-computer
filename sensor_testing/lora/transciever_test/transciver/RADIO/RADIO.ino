#include "radio_class.h"
#include <SD.h>
#include <SPI.h>

Radio radio;

union floatunion_t {
    float f;
    char a[sizeof (float) ];
} float_test;

void setup(){
  Serial.begin(115200);
  Serial.println("Setup");
  radio.begin(); 
}

//UNCOMMENT ONE OF THE LOOPS BELOW FOR TESTING



void loop(){
  radio.receivedPacket();
  delay(10);
}


//Sending
/*
void loop()
{
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // re-open the file for reading:
  char testName[17]="flightLog000.txt";
  char fileName[17]="flightLog000.txt";
  for (uint8_t i = 0; i < 100; i++) {
    testName[9] = i/100 + '0';
    testName[10] = i/10 + '0';
    testName[11] = i%10 + '0';
    if (!SD.exists(testName)){
      break;
    }
    for(int j=0; j<17; j++)
    {
      fileName[j]=testName[j];
    }
  }
  Serial.println(fileName);
  File myFile = SD.open(fileName);
  int counter=0;
  int dataPointCount=27;
  char buf[dataPointCount*4];
  myFile.readBytesUntil('\n',buf,dataPointCount*4);
  if (myFile) {
    Serial.println("File Open");
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      myFile.readBytes(buf,dataPointCount*4);
      counter++;
//      Serial.print("Line ");
      Serial.print(counter);
//      Serial.print(": ");
      for(int j=0; j<dataPointCount; j++)
      {
        for(int i=0; i<4; i++)
        {
          char letter=buf[4*j+i];
          float_test.a[i]=letter;
        }
//        Serial.print(float_test.f);
//        Serial.print(",");
      }
      radio.sendRadio(buf);
      myFile.readBytes(buf,2);
      Serial.println();
//      delay(50);
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  while(1){}
  // testing commits
}
*/ 
