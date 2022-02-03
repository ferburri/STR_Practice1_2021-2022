// --------------------------------------
// Include files
// --------------------------------------
#include <string.h>
#include <stdio.h>
#include <Wire.h>
#include <time.h>
#include <float.h>

// --------------------------------------
// Global Constants
// --------------------------------------
#define SLAVE_ADDR 0x8
#define MESSAGE_SIZE 8
#define LED_ACC 13
#define LED_BRK 12
#define LED_MIX 11
#define LED_SPEED 10
#define S1 9
#define S3 8
#define ACC_DOWN 0.25
#define ACC_UP -0.25
#define ACC_FLAT 0
#define ACC 0.5
#define BRAKE -0.5
#define MAX_UNSIGNED_LONG 4294967295

// --------------------------------------
// Global Variables
// --------------------------------------
double speed = 55.5;
bool request_received = false;
bool answer_requested = false;
char request[MESSAGE_SIZE+1];
char answer[MESSAGE_SIZE+1];
double acc_slope = 0.0;
double acc = 0.0;
unsigned long timeLast = micros();

// --------------------------------------
// Handler function: receiveEvent
// --------------------------------------
void receiveEvent(int num)
{
   char aux_str[MESSAGE_SIZE+1];
   int i=0;

   // read message char by char
   for (int j=0; j<num; j++) {
      char c = Wire.read();
      if (i < MESSAGE_SIZE) {
         aux_str[i] = c;
         i++;
      }
   }
   aux_str[i]='\0';

   // if message is correct, load it
   if ((num == MESSAGE_SIZE) && (!request_received)) {
      memcpy(request, aux_str, MESSAGE_SIZE+1);
      request_received = true;
   }
}

// --------------------------------------
// Handler function: requestEvent
// --------------------------------------
void requestEvent()
{
   // if there is an answer send it, else error
   if (answer_requested) {
      Wire.write(answer,MESSAGE_SIZE);
      memset(answer,'\0', MESSAGE_SIZE+1);
   } else {
      Wire.write("MSG: ERR",MESSAGE_SIZE);
   }

   // set answer empty
   request_received = false;
   answer_requested = false;
   memset(request,'\0', MESSAGE_SIZE+1);
   memset(answer,'0', MESSAGE_SIZE);
}

// --------------------------------------
// Function: speed_req
// --------------------------------------
double totalElapsedTime = 0;
int speed_req()
{
   // Compute The times to generate a speed
   unsigned long newtime = micros();
   unsigned long timeDiff = diffULong(timeLast, newtime);
   timeLast = newtime;
   double elapsedTime = (double)timeDiff / 1000000;
   totalElapsedTime += elapsedTime;
   speed = speed + (acc+acc_slope) * elapsedTime;
   Serial.println("SPD: "+String(speed));
   speed_cmp();
   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("SPD: REQ",request)) ) {

      // send the answer for speed request
      char num_str[5];
      dtostrf(speed,4,1,num_str);
      sprintf(answer,"SPD:%s",num_str);


      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }
   return 0;
}

// --------------------------------------
// Function: slope_req
// --------------------------------------
int slope_req()
{
  // Read the input of Switches
  int up = digitalRead(S1);
  int down = digitalRead(S3);

  // UP is triggered
  if(up && !down){
      // Change the Acceleration
      acc_slope = ACC_UP;
  }
  else if(!up && down){
      // Change the Acceleration
      acc_slope = ACC_DOWN;
  }
	else if(!down && !up){
		  // Change the Acceleration
      acc_slope = ACC_FLAT;
	}

  // while there is enough data for a request
  if ( (request_received) &&
      (0 == strcmp("SLP: REQ",request)) ) {
	  // Check the state

    // UP is triggered
  if(up && !down){
      // Set the response MSG
      sprintf(answer,"SLP:  UP");
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
    }
    // DOWN is triggered
  else if(!up && down){
      // Set the response MSG
      sprintf(answer,"SLP:DOWN");
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
    }
    // Is in Flat
  else if(!down && !up){
      // Set the response MSG
      sprintf(answer,"SLP:FLAT");
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
    }
  }
  return 0;
}

// --------------------------------------
// Function: Activate / Deactivate Accelerator
// --------------------------------------
int acc_req()
{
   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("GAS: SET",request)) ) {
      // Put the Led on.
      digitalWrite(LED_ACC, HIGH);

      // send the answer for speed request
      sprintf(answer,"GAS:  OK");
      acc = ACC;
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }
   // Set the Light Off
   else if((request_received) &&
        (0 == strcmp("GAS: CLR",request)) ) {
      // Put the Led on.
      digitalWrite(LED_ACC, LOW);
      acc = 0;
      // send the answer for speed request
      sprintf(answer,"GAS:  OK");

      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }

   return 0;
}

// --------------------------------------
// Function: Activate / Deactivate Brake
// --------------------------------------
int brk_req()
{
   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("BRK: SET",request)) ) {
      // Put the Led on.
      digitalWrite(LED_BRK, HIGH);
      acc = BRAKE;
      // send the answer for speed request
      sprintf(answer,"BRK:  OK");

      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }
   else if((request_received) &&
        (0 == strcmp("BRK: CLR",request)) ) {
      // Put the Led on.
      digitalWrite(LED_BRK, LOW);

      // send the answer for speed request
      sprintf(answer,"BRK:  OK");
      acc=0.0;
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }

   return 0;
}

// --------------------------------------
// Function: Activate / Deactivate Brake
// --------------------------------------
int mix_req()
{
   // Check the state of the Mixer
   if ( (request_received) &&
        (0 == strcmp("MIX: SET",request)) ) {

      // Put the Led on. Aqui hay que comprobar el estado del led con la funcion que runea periodicamente
      digitalWrite(LED_MIX, HIGH);

      // send the answer for speed request
      sprintf(answer,"MIX:  OK");

      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }
   // Deactivate the Mixer Led
   else if((request_received) &&
        (0 == strcmp("MIX: CLR",request)) ) {

      // Put the Led off.
      digitalWrite(LED_MIX, LOW);

      // send the answer for speed request
      sprintf(answer,"MIX:  OK");

      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }

   return 0;
}

// --------------------------------------
// Function: Compute the Speed
// --------------------------------------
int speed_cmp()
{

   // Set the brightness of speed
   if(speed >= 40 && speed <= 70){
      // Invoke the fucntion TransformRange to convert into the range of [0, 255]
      double brightness = transformRange(speed);
      // Set the Light
      analogWrite(LED_SPEED, brightness);
   }
   // Set to zero if the speed is not in the range
   else{
     analogWrite(LED_SPEED, 0);
   }

}

// --------------------------------------
// Function: Transform the range of the speed to the range of PWD.
// --------------------------------------
double transformRange(double value){
  return (value - 40) * (double)255/30;
}

// --------------------------------------
// Function: Make the operation to obtain the time to compute the speed
// --------------------------------------
unsigned long diffULong(unsigned long start, unsigned long end){
	if (start < end){
		return end-start;
	}else{
		return (MAX_UNSIGNED_LONG-start+end);
	}
}

// --------------------------------------
// Function: setup
// --------------------------------------
void setup()
{
  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);

  // Put the Leds as output
  pinMode(LED_ACC, OUTPUT);
  pinMode(LED_BRK, OUTPUT);
  pinMode(LED_MIX, OUTPUT);
  pinMode(LED_SPEED, OUTPUT);

  // Put the Switches as input
  pinMode(S1, INPUT);
  pinMode(S3, INPUT);

  Serial.begin(9600);
}

// --------------------------------------
// Function: loop
// --------------------------------------
void loop()
{
  // Declare the Start Time of the Loop
  unsigned long start_time = micros();
  unsigned long end_time = 0;
  unsigned long lapso = 0;
  // Infinite Loop
  while(true){

    speed_req();
    slope_req();
    acc_req();
    brk_req();
    mix_req();

    // Apply the Sleep Times.
    end_time = micros();
    lapso = diffULong(start_time,end_time);
    start_time = start_time + 200000;
    lapso = lapso/1000;
    delay(200 - lapso);

  }
}
