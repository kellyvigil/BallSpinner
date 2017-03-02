#include <Arduino.h>
#include "DRV8825.h"
#include <Ethernet2.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCBoards.h>




// ----- OSC Initializing Info -----
//the Arduino's IP
IPAddress ip(10, 0, 1, 200);
//destination IP
IPAddress outIp(10, 0, 1, 10);
//in and out ports
const unsigned int outPort = 8000;
const unsigned int inPort = 9000;
//mac address
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // you can find this written on the board of some Arduino Ethernets or shields

EthernetUDP Udp;

//----- Stepper Motor Driver Initializing Info -----
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200

// All the wires needed for full functionality
#define DIR 8
#define STEP 9
#define ENBL 13
// microstep control for DRV8825 supports 32 microsteps
#define MODE0 10
#define MODE1 11
#define MODE2 12

DRV8825 stepper(MOTOR_STEPS, DIR, STEP, MODE0, MODE1, MODE2);

/*
* Microstepping mode: 1,2,4,8,16 or 32(DRV8834 only)
* Mode 1 is full speed.
* Mode 32 is 32 microsteps per step.
* The motor should rotate just as fast (set RPM),
* but movement precision is increased.
*/

//MicroStep speed devider
int msSpeed = 8;
//Guess
int RPM = 3;

// incoming serial byte
int inByte = 0;

//current rotation state
int rState;


//converts the pin to an osc address
char * numToOSCAddress( int pin){
    static char s[10];
    int i = 9;

    s[i--]= '\0';
	do
    {
		s[i] = "0123456789"[pin % 10];
                --i;
                pin /= 10;
    }
    while(pin && i);
    s[i] = '/';
    return &s[i];
}

void incomingMessage(OSCMessage &msg, int addrOffset ){
  //iterate through all the analog pins
  /*for(byte pin = 0; pin < NUM_DIGITAL_PINS; pin++){
    //match against the pin number strings
    int pinMatched = msg.match(numToOSCAddress(pin), addrOffset);
    if(pinMatched){
      unsigned int frequency = 0;
      //if it has an int, then it's an integers frequency in Hz
      if (msg.isInt(0)){
        frequency = msg.getInt(0);
      } //otherwise it's a floating point frequency in Hz
      else if(msg.isFloat(0)){
        frequency = msg.getFloat(0);
      }
      else
        noTone(pin);
      if(frequency>0)
      {
         if(msg.isInt(1))
           tone(pin, frequency, msg.getInt(1));
         else
           tone(pin, frequency);
      }
    }
  } */
}




void setup() {
  //Set Ethernet info
  Ethernet.begin(mac,ip);
  Udp.begin(8888);

  //Serial baud rate
  //Serial.begin(9600);

  // set global RPM for stepper motor
  stepper.setRPM(RPM);

  //init the rotation
  rState = 0;
}


void loop() {

  //Set the microstep speed for fine stepping ;)
  stepper.setMicrostep(msSpeed);

  //Add 1 rotation to the count
  //@TODO figure our how to make sure that the update of rState
  // is happening at the same rate that the motor is updating
  // the pule for a rotation
  // @TODO Look into the step degrees to increase by that amount (1.8º)
  rState = rState + 1;

  //Keep rState looping between 0-359
  // @TODO Update for above incramenting

  if (rState == 360) {
    rState = 0;
  }


  // ------  OSC Message Process ------

  //reads and dispatches the incoming message
  OSCBundle bundleIN;
   int size;

   if( (size = Udp.parsePacket())>0)
   {
     while(size--)
       bundleIN.fill(Udp.read());

      if(!bundleIN.hasError())
        bundleIN.route("/toArduino", incomingMessage);
   }

  //Set the OSC Path and message
  OSCMessage msg("/rState");
  msg.add(rState);

  //Set the IP path and send packet
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);

  //End the packet and free up space occupied by packet
  Udp.endPacket();
  msg.empty();

  //Move the stepper motor by 1 total movement based on
  // msSpeed steps multiplier
  stepper.move(1 * msSpeed);

}


/*
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}
*/
