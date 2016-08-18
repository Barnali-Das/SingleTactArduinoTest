/*
 SingleTact Analog Input Demo
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 display value on comm port obtained by analogRead().

 The circuit:
 * SingleTact analog output (Refer the manual) to A0 of Arduino.
 * Connect GND & VCC between SingleTact and Arduino
 * 
 * To compile: Sketch --> Verify/Compile
 * To upload: Sketch --> Upload
 * 
 * * For comm port monitoring: Click on Tools --> Serial Monitor
 * Remember to set the baud rate at 57600.
 * 
 * Created by Barnali Das Aug 2016
 */
int sensorPin = A0;    // select the input pin for the potentiometer
float sensorValue = 0;  // variable to store the value coming from the sensor
#define SCALE_PARAMETER 220 //Change this value based on you sensor. I'm using 1N sensor.  

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("SingleTact sensor value (analogue) in volt:");
}

void loop() {
  sensorValue = analogRead(sensorPin);
  sensorValue /= SCALE_PARAMETER;
  delay(1000); // 1 sec delay between two consecutive reading
  Serial.println(sensorValue);
}
