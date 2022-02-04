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
#define LED_LAMP 7
#define P1 2
#define P2 3
#define P3 4
#define P4 5
#define ACC_DOWN 0.25
#define ACC_UP -0.25
#define ACC_FLAT 0
#define ACC 0.5
#define BRAKE -0.5
#define MAX_UNSIGNED_LONG 4294967295


// --------------------------------------
// Global Variables
// --------------------------------------
const int ldrPin = A0;
double speed = 55.5;
bool request_received = false;
bool answer_requested = false;
char request[MESSAGE_SIZE+1];
char answer[MESSAGE_SIZE+1];
double elapsedTime = 0.0;
double acc_slope = 0.0;
double acc = 0.0;
unsigned long timeLast = micros();
int lamps = 0;
char dis_value[5];
double selected_distance = 0.0;
double act_distance = 0.0;
int sensorValue = 0;
int buttonState = 0;
int lastButtonState = 0;
int buttonStateStop = 0;
int lastButtonStateStop = 0;
int CURRENT_MODE = 0;

static const struct number {
    uint8_t d;
    uint8_t c;
    uint8_t b;
    uint8_t a;
  } numbers[] = {
    { LOW,  LOW,  LOW,  LOW}, /* 0 */
    { LOW,  LOW,  LOW, HIGH}, /* 1 */
    { LOW,  LOW, HIGH,  LOW}, /* 2 */
    { LOW,  LOW, HIGH, HIGH}, /* 3 */
    { LOW, HIGH,  LOW,  LOW}, /* 4 */
    { LOW, HIGH,  LOW, HIGH}, /* 5 */
    { LOW, HIGH, HIGH,  LOW}, /* 6 */
    { LOW, HIGH, HIGH, HIGH}, /* 7 */
    {HIGH,  LOW,  LOW,  LOW}, /* 8 */
    {HIGH,  LOW,  LOW, HIGH}, /* 9 */
  };

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
      Serial.println(request);
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
      Serial.println(answer);
      memset(answer,'\0', MESSAGE_SIZE+1);

   } else {
      Serial.println("RESPONDED ERROR\n");
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
int speed_req()
{
   // if its in Emergency mode, when the speed is null, we brake the entire system
   if(CURRENT_MODE == 3 && speed <=0.0){
    speed = 0.0;
   }
   else if(CURRENT_MODE != 2){
     unsigned long newtime = micros();
     unsigned long timeDiff = diffULong(timeLast, newtime);
     timeLast = newtime;
     elapsedTime = (double)timeDiff / 1000000;
     speed = speed + (acc+acc_slope) * elapsedTime;
   }
    else {
    speed = 0.0;
   }

   speed_cmp();
   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("SPD: REQ",request)) ) {

      Serial.println("ACC."+String(acc)+" SLP."+String(acc_slope));
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
  int up = digitalRead(9);
  int down = digitalRead(8);

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
      Serial.println("SLOPE FLAT");
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
// Function: Deactivate Accelerator (Emergency mode)
// --------------------------------------
int acc_emg_req(){
  // Turn off the Gas Light
  digitalWrite(LED_ACC, LOW);
  acc = 0;
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
// Function: Activate Brake (Emergency mode)
// --------------------------------------
int brk_emg_req(){
  // Turn off the Gas Light
  digitalWrite(LED_BRK, HIGH);
  acc = BRAKE;
  return 0;
}


// --------------------------------------
// Function: Activate / Deactivate Brake
// --------------------------------------
int mix_req()
{
   // while there is enough data for a request
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
// Function: lamps_req
// --------------------------------------
int lamps_req()
{
   // Read the Value of the LDR sensor
   int ldrStatus = analogRead(A0);
   //Serial.println("StatusLDR:"+String(ldrStatus));
   // Transform the value of ldrStatus into a range between 0 and 99 %
   int lamps = transformRangeLamps(ldrStatus);
   //Serial.println("Lamps:"+String(lamps));

   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("LIT: REQ",request)) ) {

      // send the answer for speed request
      char num_str[5];
      dtostrf(lamps,3,0,num_str);

      sprintf(answer,"LIT: %s%",num_str);

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
int lamp_led()
{
   // while there is enough data for a request
   if ( (request_received) &&
        (0 == strcmp("LAM: SET",request)) ) {
      // Put the Led on.
      digitalWrite(LED_LAMP, HIGH);
      acc = BRAKE;
      // send the answer for speed request
      sprintf(answer,"LAM:  OK");

      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }
   else if((request_received) &&
        (0 == strcmp("LAM: CLR",request)) ) {
      // Put the Led on.
      digitalWrite(LED_LAMP, LOW);

      // send the answer for speed request
      sprintf(answer,"LAM:  OK");
      acc=0.0;
      // set buffers and flags
      memset(request,'\0', MESSAGE_SIZE+1);
      request_received = false;
      answer_requested = true;
   }

   return 0;
}

// --------------------------------------
// Function: Activate Lamps
// --------------------------------------
int lamp_emg_led(){
  // Put the Led on.
  digitalWrite(LED_LAMP, HIGH);
  return 0;
}

// --------------------------------------
// Function: distance_req
// --------------------------------------
int distance_req()
{
  sensorValue = analogRead(A1);

  // Transform the Distance between 10000 and 90000
  selected_distance = transformRangeDistance(sensorValue);
  // Store the value of distance
  dtostrf(selected_distance,4,0,dis_value);

  return 0;
}

// --------------------------------------
// Function: distance_dsp
// --------------------------------------
int distance_dsp()
{
  // convert potentiometer_distance to a 1-9 digit
  int valueToDisplay = dis_value[0] - '0';

  // write on the display the value of the sensor (stored in potentiometer_distance)
  digitalWrite(P1, numbers[valueToDisplay].a);
  digitalWrite(P2, numbers[valueToDisplay].b);
  digitalWrite(P3, numbers[valueToDisplay].c);
  digitalWrite(P4, numbers[valueToDisplay].d);

  return 0;
}

// --------------------------------------
// Function: distance_val
// --------------------------------------
int distance_val()
{
  // check when the button has been activated (pushed and released)
  buttonState = digitalRead(6);
  //Serial.println("Button"+String(buttonState));
  //if pushed and released then the potentiometer_distance is selected as the actual distance
  if (buttonState != lastButtonState) {
    CURRENT_MODE = buttonState; // change to approach mode
    act_distance = selected_distance; // Make the selected distance as real distance
    // delay a little bit to avoid debouncing
    delay(5); // Wait for 5 millisecond(s)
  }
  // save the current state as the last state, for
  // the next time through the loop
  lastButtonState = buttonState;

  return 0;
}

// --------------------------------------
// Function: actual_distance
// --------------------------------------
int actual_distance()
{
  // Compute the Actual Distance depending on the Actual Speed.
  double cmp_distance = (speed * elapsedTime) + 0.5*((acc+acc_slope) * (elapsedTime*elapsedTime));
  // Update distance
  act_distance -= cmp_distance;

  // Check the value of the distance and speed
  if (act_distance <= 0 && speed <= 10 ) {
    // Change to Stope mode
    CURRENT_MODE = 2;
    act_distance = 0;
  }
  if (act_distance <= 0 && speed >= 10 ) {
    // Change to Selection mode
    CURRENT_MODE = 0;
  }
  Serial.println("ACT DISTANCE: "+String(act_distance));
  // Store the value of distance
  dtostrf(act_distance,4,0,dis_value);
  Serial.println("Value DISTANCE: "+String(dis_value));
  distance_dsp();

  // while there is enough data for a request
  if ( (request_received) &&
       (0 == strcmp("DS:  REQ",request)) ) {
         // Append the Value to the fucntion.
         sprintf(answer,"DS:%s", dis_value);
         // set buffers and flags
         memset(request,'\0', MESSAGE_SIZE+1);
         request_received = false;
         answer_requested = true;
   }

}


// --------------------------------------
// Function: stop_end
// --------------------------------------
int stop_end()
{
  // check when the button has been activated (pushed and released)
  buttonStateStop = digitalRead(6);

  //if pushed and released then the potentiometer_distance is selected as the actual distance
  if (buttonStateStop != lastButtonStateStop) {
    CURRENT_MODE = 0; // change to distance selection
    timeLast = micros();
    speed = 0.0;
    //act_distance = selected_distance; // Make the selected distance as real distance
    // delay a little bit to avoid debouncing
    delay(5); // Wait for 5 millisecond(s)
  }
  // save the current state as the last state, for
  // the next time through the loop
  lastButtonStateStop = buttonStateStop;

    if ( (request_received) &&
         (0 == strcmp("STP: REQ",request)) ) {

           if (CURRENT_MODE!=2){
             sprintf(answer,"STP:  GO");

             // set buffers and flags
             memset(request,'\0', MESSAGE_SIZE+1);
             request_received = false;
             answer_requested = true;
           } else {
             sprintf(answer,"STP:STOP");

             // set buffers and flags
             memset(request,'\0', MESSAGE_SIZE+1);
             request_received = false;
             answer_requested = true;
           }
    }

    return 0;

}

// --------------------------------------
// Function: enable_emg_mode
// --------------------------------------
int enable_emg_mode(){
  if ( (request_received) &&
       (0 == strcmp("ERR: SET",request)) ) {
         CURRENT_MODE = 3;
         sprintf(answer,"ERR:  OK");

         // set buffers and flags
         memset(request,'\0', MESSAGE_SIZE+1);
         request_received = false;
         answer_requested = true;
       }
  return 0;
{

// --------------------------------------
// Function: Compute the Speed
// --------------------------------------
int speed_cmp()
{

   // Set the brightness of speed
   if(speed >= 40 && speed <= 70){
      // Invoke the fucntion TransformRange to convert into the range of [0, 255]
      double brightness = transformRangeSpeed(speed);
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
double transformRangeSpeed(double value){
  return (value - 40) * (double)255/30;
}

// --------------------------------------
// Function: Transform the range of the lamps to the range of PWD.
// --------------------------------------
int transformRangeLamps(int value){
  return (value - 54) * (float)100/923;
}

// --------------------------------------
// Function: Transform the range of the Potenciometer to the range of PWD.
// --------------------------------------
double transformRangeDistance(int value){
  double scale = (double)80000/512;
  double val1 = (value - 511) * scale  + 10000;
  return val1;
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

  // Put the Display Pins as Output
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);
  pinMode(P3, OUTPUT);
  pinMode(P4, OUTPUT);

  // Led Pin Outputs
  pinMode(LED_ACC, OUTPUT);
  pinMode(LED_BRK, OUTPUT);
  pinMode(LED_MIX, OUTPUT);
  pinMode(LED_SPEED, OUTPUT);
  pinMode(LED_LAMP, OUTPUT);

  // Switch Inputs
  pinMode(9, INPUT);
  pinMode(8, INPUT);

  // Put the LDR sensor as Input
  pinMode(ldrPin, INPUT);

  pinMode(A1, INPUT); // Potenciometer
  pinMode(A0, INPUT); // LDR sensor as Input
  pinMode(6, INPUT); // Button Input

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
  while(true){

    switch(CURRENT_MODE){
      case 0: // Distance selection mode
        speed_req();
        slope_req();
        acc_req();
        brk_req();
        mix_req();
        lamps_req();
        lamp_led();
        distance_req();
        distance_dsp();
        distance_val();
        enable_emg_mode();
        break;

      case 1: // Approaching mode
        speed_req();
        slope_req();
        acc_req();
        brk_req();
        mix_req();
        lamps_req();
        lamp_led();
        actual_distance();
        enable_emg_mode();
        break;

      case 2: // Stop mode
        speed_req();
        slope_req();
        acc_req();
        brk_req();
        mix_req();
        lamps_req();
        lamp_led();
        stop_end();
        enable_emg_mode();
        break;
      case 3: // Emergency mode
        acc_emg_req();
        brk_emg_req();
        mix_req();
        speed_req();
        slope_req();
        lamp_emg_led();
        break;

    }
    // Apply the Sleep Times
    end_time = micros();
    lapso = diffULong(start_time,end_time);
    start_time = start_time + 200000;
    lapso = lapso/1000;
    delay(200 - lapso);

  }

}
