//-----------------------------------------------------------------------------
//  Copyright (c) 2016 Pressure Profile Systems
//
//  Licensed under the MIT license. This file may not be copied, modified, or
//  distributed except according to those terms.
//    
//  Author: Barnali Das
//   Aug 2016
//-----------------------------------------------------------------------------

#include <Wire.h> //For I2C/SMBus
#include <Timer1.h> //For timestamp

#define CMD_READ        0x01
#define CMD_WRITE       0x02
#define CMD_TOGGLE_GPIO 0x03
#define CMD_WRITE_CAL   0x04
#define TOGGLE_ALL      0x08
#define TOGGLE_NONE     0x00

//Constants
//General I2C packet structure
const byte I2C_HEADER1 = 0;
const byte I2C_HEADER2 = 1;
const byte I2C_HEADER3 = 2;
const byte I2C_HEADER4 = 3;
const byte I2C_ADDRESS_BYTE = 4;
const byte I2C_TIMOUT_BYTE = 5;
const byte I2C_ID_BYTE = 6;
const byte I2C_NUM_FOOTER_BYTES = 4;
const byte I2C_TO_SENSOR_BUFFER_LENGTH = 32;
const byte I2C_TIMEOUT = 100; //100ms

//From PC
const byte I2C_FROMPC_CMD = 7;
const byte I2C_FROMPC_READWRITE_LOCATION = 8;
const byte I2C_FROMPC_NUM_BYTES_TO_READWRITE = 9;
const byte SERIAL_FROMPC_BUFFER_LENGTH = 43;
const byte MINIMUM_FROMPC_PACKETLENGTH = 13; //I2C_FROMPC_NUM_BYTES_TO_READWRITE + 4 footer bytes

//To PC
const byte I2C_TOPC_TIMESTAMP = 7;
const byte I2C_TOPC_NBYTES = 11;
const byte SERIAL_TOPC_BUFFER_LENGTH = 77;

// Pin 13 has an LED connected.
int led = 13;

// Pin 2, 3, 4, 5, 6, 7 are reserved for Power Line
int singleTact1 = 2;
int singleTact2 = 3;
int singleTact3 = 4;
int singleTact4 = 5;
int singleTact5 = 6;
int singleTact6 = 7;

//From Arduino to sensor
byte outgoingI2CBuffer[I2C_TO_SENSOR_BUFFER_LENGTH];

// Arduino internal read
byte readFromSensorI2C[6];

//From Arduino to PC host
byte serialToPCBuffer[SERIAL_TOPC_BUFFER_LENGTH];
byte serialToPCBufferIndex_ = 0;

//From PC host to Arduino
byte serialIncomingBuffer[SERIAL_FROMPC_BUFFER_LENGTH];
byte serialIncomingBufferIndex_ = 0;

unsigned long timeStamp_;

//Zero a buffer
void BlankBuffer(byte* buffer, byte length)
{
  for(int i = 0; i < length; i++)
  {
    buffer[i] = 0;
  }
}

void ResetSerialBuffer()
{
  for(int i = 0; i < SERIAL_FROMPC_BUFFER_LENGTH; i++)
  {
    serialIncomingBuffer[i] = 0;
  }

  serialIncomingBufferIndex_ = 0;
}

void setup()
{
  int i;

  Wire.begin(); // join i2c bus (address optional for master)

  pinMode(led, OUTPUT);
  Serial.begin(115200);  // start serial for output
  Serial.flush();

  pinMode(singleTact1, OUTPUT);
  pinMode(singleTact2, OUTPUT);
  pinMode(singleTact3, OUTPUT);
  pinMode(singleTact4, OUTPUT);
  pinMode(singleTact5, OUTPUT);
  pinMode(singleTact6, OUTPUT);

  BlankBuffer(outgoingI2CBuffer, I2C_TO_SENSOR_BUFFER_LENGTH);

  //Never changes, so just set now
  serialToPCBuffer[I2C_HEADER1] = 0xFF;
  serialToPCBuffer[I2C_HEADER2] = 0xFF;
  serialToPCBuffer[I2C_HEADER3] = 0xFF;
  serialToPCBuffer[I2C_HEADER4] = 0xFF;

  digitalWrite(led, HIGH);

  timeStamp_ = 0;
  startTimer1(100); //Timer for timestamp

  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("PPS UK: SingleTact sensor value in PSI. \n(resembles PC executable display)");
  Serial.println("Refer manual for any other calculation.");
  Serial.println("----------------------------------------");
}


//Check the full footer
boolean Checkfooter(int endOfPacket)
{
  for(int i = 0; i < 4; i++)
  {
    if(serialIncomingBuffer[endOfPacket - i] != 0xFE)
    {
      return false;  //Footer corrupt
    }
  }

  return true; //Footer all good
}


//Check available header bytes (called as each one comes in, building upto 4)
boolean Checkheader(int checkMaxIndex)
{
  for(int j = 0; j <= checkMaxIndex && j < 4; j ++)
  {
    if(serialIncomingBuffer[j] != 0xFF)
    {
      return false; //Header corrupt
    }
  }
  return true; //Header all good
}

//Returns true if we have a new serial data packet, otherwise returns false
//Corrupt data is removed from buffer
boolean ProcessIncomingSerialData()
{
  byte i2cPacketLength; //Working variable

  //----------  This is important as Arduino does not receive any command from PC --- //
  for (int i = 0; i < 4; i++) serialIncomingBuffer[i] = 0xFF;
  serialIncomingBuffer[4] = 0x04; // i2cAddress
  serialIncomingBuffer[5] = 100; // Time out
  serialIncomingBuffer[6] = 1;  // Command iteration. No use now. 

  serialIncomingBuffer[7] = 0x01; 
  serialIncomingBuffer[8] = 128; // Location to read (sensor data starts at 128)
  serialIncomingBuffer[9] = 6; // Number of bytes to read (max = 32)
  serialIncomingBuffer[10] = 0xFF; 
  for (int i = 0; i < 4; i++) serialIncomingBuffer[11 + i] = 0xFE;
  //----------------------------------------------------------------------------------//


  if(false == Checkheader(serialIncomingBufferIndex_))
  {
    ResetSerialBuffer();  //Header not correct, reset buffer
    return false;
  }

  //Do we have enough data to process
  if(serialIncomingBufferIndex_ >MINIMUM_FROMPC_PACKETLENGTH)
  {
    if(CMD_READ == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      i2cPacketLength = 0;
    }
    else
    {
      i2cPacketLength = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];
    }


    //Do we have a full packet
    if(serialIncomingBufferIndex_ >= (i2cPacketLength + MINIMUM_FROMPC_PACKETLENGTH + 1))
    {
      if(Checkfooter(serialIncomingBufferIndex_))
      {
        //We have a good packet
        return true;
      }
      else
      {
        //Corrupt packet, reset
        ResetSerialBuffer();
        return false;
      }
    }
  }

  //We have run out of buffer space - something has gone wrong, so just reset the buffer
  if(serialIncomingBufferIndex_ >= (SERIAL_FROMPC_BUFFER_LENGTH -1))
  {
    ResetSerialBuffer();
  }

  serialIncomingBufferIndex_++; //Move the index on
  return false;  //Not at the end of the packet yet
}

// Define the function which will handle the notifications
ISR(timer1Event)
{
  resetTimer1();
  timeStamp_++;
}

void loop()
{
  byte inputBuffer[43];
  byte finishedPacket = false;
  byte timeout = 5;
  byte i = 0;
  byte lengthReceived = 0;

  if(ProcessIncomingSerialData())
  {
    if(CMD_READ == serialIncomingBuffer[I2C_FROMPC_CMD])
    {
      byte i2cPacketLength = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];

      //Perform I2C Read
      outgoingI2CBuffer[0] = CMD_READ;
      outgoingI2CBuffer[1] = serialIncomingBuffer[I2C_FROMPC_READWRITE_LOCATION];
      outgoingI2CBuffer[2] = serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE];

      //Send I2C packet
      Wire.beginTransmission(serialIncomingBuffer[I2C_ADDRESS_BYTE]); // transmit to device
      Wire.write(outgoingI2CBuffer, 3); //Only requires 3 bytes
      Wire.endTransmission();    // stop transmitting

      Wire.requestFrom(serialIncomingBuffer[I2C_ADDRESS_BYTE], serialIncomingBuffer[I2C_FROMPC_NUM_BYTES_TO_READWRITE]);

      //Parse request
      int i = 0;
      int i2cTimeout = I2C_TIMEOUT; //1s

      while( i < i2cPacketLength && i2cTimeout > 0)    // slave may send less than requested
      {
        if(Wire.available())
        {
          readFromSensorI2C[i] = Wire.read(); // receive a byte as character
          i++;
        }
        else
        {
          delay(1); //Wait 1ms
          i2cTimeout--;
        }
      }

      unsigned int sensor_val = ((readFromSensorI2C[4] << 8) + readFromSensorI2C[5]) & 0x000003FF; 
      Serial.println(sensor_val);
    }

    ResetSerialBuffer();  //Reset and wait for next command
  }
  delay(50); // Change the delay to control serial monitor output
}
