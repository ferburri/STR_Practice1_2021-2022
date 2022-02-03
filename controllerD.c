/**********************************************************
 *  INCLUDES
 *********************************************************/
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <float.h>
#include <fcntl.h>

#include <rtems.h>
#include <bsp.h>

//#define RASPBERRYPI
#ifdef RASPBERRYPI
#include <bsp/i2c.h>
#endif

#include "displayD.h"

/**********************************************************
 *  Constants
 **********************************************************/
#define MSG_LEN    8
#define SLAVE_ADDR 0x8
#define TIME_CYCLE_SEC 5
#define TIME_CYCLE_NSEC 5000000000
#define NS_PER_S  1000000000

#define NORMAL_MODE 0
#define BRAKING_MODE 1
#define STOP_MODE 2
#define EMERGENCY_MODE 3

/**********************************************************
 *  Global Variables
 *********************************************************/
float speed = 0.0;
struct timespec time_msg = {0,400000000};
int fd_i2c = -1;
int dark = 0;
int mixer_state = 0;
struct timespec time_last_change_mixer;
unsigned int current_distance;
int emg_mode = 0;
char error_string[10] = {'\0','\0','\0','\0','\0','\0','\0','\0','\n','\0'};

/**********************************************************
 *  Function: difftime
 *********************************************************/
void diffT(struct timespec end,
              struct timespec start,
              struct timespec *diff)
{
    if (end.tv_nsec < start.tv_nsec) {
        diff->tv_nsec = NS_PER_S - start.tv_nsec + end.tv_nsec;
        diff->tv_sec = end.tv_sec - (start.tv_sec+1);
    } else {
        diff->tv_nsec = end.tv_nsec - start.tv_nsec;
        diff->tv_sec = end.tv_sec - start.tv_sec;
    }
}

/**********************************************************
 *  Function: addtime
 *********************************************************/
void addT(struct timespec end,
             struct timespec start,
             struct timespec *add)
{
    unsigned long aux;
    aux = start.tv_nsec + end.tv_nsec;
    add->tv_sec = start.tv_sec + end.tv_sec +
    (aux / NS_PER_S);
    add->tv_nsec = aux % NS_PER_S;
}


/**********************************************************
 *  Function: task_speed
 *********************************************************/
int task_speed()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request, '\0', 10);
    memset(answer, '\0', 10);

    // request speed
    strcpy(request, "SPD: REQ\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif

    // display speed
    if (1 == sscanf (answer, "SPD:%f\n", &speed)){
        displaySpeed(speed);
    }
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    return 0;
}

/**********************************************************
 *  Function: task_speed_emg_mode
 *********************************************************/
int task_speed_emg_mode()
{
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request, '\0', 10);
    memset(answer, '\0', 10);

    // request speed
    strcpy(request, "SPD: REQ\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif

    // display speed
    if (1 == sscanf (answer, "SPD:%f\n", &speed)){
        displaySpeed(speed);
    }
    return 0;
}

//-------------------------------------
//-  Function: task_slope
//-------------------------------------
int task_slope()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // request slope
    strcpy(request, "SLP: REQ\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
  if (0 == strcmp(answer, "SLP:DOWN\n")) displaySlope(-1);
  else if (0 == strcmp(answer, "SLP:FLAT\n")) displaySlope(0);
  else if (0 == strcmp(answer, "SLP:  UP\n")) displaySlope(1);
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }

  return 0;
}

//-------------------------------------
//-  Function: task_slope_emg_mode
//-------------------------------------
int task_slope_emg_mode()
{
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // request slope
    strcpy(request, "SLP: REQ\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
  if (0 == strcmp(answer, "SLP:DOWN\n")) displaySlope(-1);
  else if (0 == strcmp(answer, "SLP:FLAT\n")) displaySlope(0);
  else if (0 == strcmp(answer, "SLP:  UP\n")) displaySlope(1);

  return 0;
}

//-------------------------------------
//-  Function: task_acc
//-------------------------------------
int task_acc()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to accelerate
    if(speed <= 55.0){
        strcpy(request, "GAS: SET\n");
        displayGas(1);
    }
    else{
        strcpy(request, "GAS: CLR\n");
        displayGas(0);
    }

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    return strcmp(answer, "GAS:  OK\n");
}

//-------------------------------------
//-  Function: task_acc_brake_mode
//-------------------------------------
int task_acc_brake_mode()
{

    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to accelerate in brake mode
    if(speed <= 2.5){
        strcpy(request, "GAS: SET\n");
        displayGas(1);
    }
    else{
        strcpy(request, "GAS: CLR\n");
        displayGas(0);
    }

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    return strcmp(answer, "GAS:  OK\n");
}

//-------------------------------------
//-  Function: task_acc_emg_mode
//-------------------------------------
int task_acc_emg_mode()
{
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to accelerate in emergency mode
    strcpy(request, "GAS: CLR\n");
    displayGas(0);

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    // Check The answer from arduino
    return strcmp(answer, "GAS:  OK\n");
}

//-------------------------------------
//-  Function: task_brake
//-------------------------------------
int task_brake()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to brake
    if(speed <= 55.0){
        strcpy(request, "BRK: CLR\n");
        displayBrake(0);
    }
    else{
        strcpy(request, "BRK: SET\n");
        displayBrake(1);
    }

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    return strcmp(answer, "BRK:  OK\n");
}

//-------------------------------------
//-  Function: task_brake_brake_mode
//-------------------------------------
int task_brake_brake_mode()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to brake in brake mode
    if(speed <= 2.5){
        strcpy(request, "BRK: CLR\n");
        displayBrake(0);
    }
    else{
        strcpy(request, "BRK: SET\n");
        displayBrake(1);
    }

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    return strcmp(answer, "BRK:  OK\n");
}

//-------------------------------------
//-  Function: task_brake_emg_mode
//-------------------------------------
int task_brake_emg_mode()
{
    char request[10];
    char answer[10];

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Request to brake in emergency mode
    strcpy(request, "BRK: SET\n");
    displayBrake(1);

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    // Check the answer from arduino
    return strcmp(answer, "BRK:  OK\n");
}


//-------------------------------------
//-  Function: task_mixer
//-------------------------------------
int task_mixer()
{
  if (emg_mode)
    return EMERGENCY_MODE;
  char request[10];
  char answer[10];

// Compute the time when the mixer needs to send the request to the arduino
  struct timespec current, lapse;
  clock_gettime(CLOCK_REALTIME, &current);
  diffT(current, time_last_change_mixer, &lapse);
  // Wait 30 seconds until changes the state
	if(lapse.tv_sec > 30) {
		if(mixer_state) {
			strcpy(request, "MIX: CLR\n");
			mixer_state = 0;
		} else {
			strcpy(request, "MIX: SET\n");
			mixer_state = 1;
		}
  }
#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
  // Check the Answer
  if(0 == strcmp(answer, "MIX:  OK\n")){
      displayMix(mixer_state);
      // Update the Mixer Time to change in the next 30 seconds
      time_last_change_mixer.tv_nsec = current.tv_nsec;
      time_last_change_mixer.tv_sec = current.tv_sec;
      return 0;
  }
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }
  return 0;
}

//-------------------------------------
//-  Function: task_mixer_emg_mode
//-------------------------------------
int task_mixer_emg_mode()
{

  char request[10];
  char answer[10];

// Compute the time when the mixer needs to send the request to the arduino
  struct timespec current, lapse;
  clock_gettime(CLOCK_REALTIME, &current);
  diffT(current, time_last_change_mixer, &lapse);
  // Wait 30 seconds until changes the state
	if(lapse.tv_sec > 30) {
		if(mixer_state) {
			strcpy(request, "MIX: CLR\n");
			mixer_state = 0;
		} else {
			strcpy(request, "MIX: SET\n");
			mixer_state = 1;
		}
  }
#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
  // Check the Answer
  if(0 == strcmp(answer, "MIX:  OK\n")){
      displayMix(mixer_state);
      // Update the Mixer Time to change in the next 30 seconds
      time_last_change_mixer.tv_nsec = current.tv_nsec;
      time_last_change_mixer.tv_sec = current.tv_sec;
      return 0;
  }
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }
  return 0;
}

//-------------------------------------
//-  Function: read_light_sensor
//-------------------------------------
int task_light_sensor()
{
    if (emg_mode)
      return EMERGENCY_MODE;
    char request[10];
    char answer[10];

	// Insert the request
	strcpy(request, "LIT: REQ\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    // Check
	int light = 0;
	if(sscanf(answer, "LIT:%d\n", &light) == 1) {

        // If the returned value is below of 50%, we request to switch on the lights.
		dark = light < 50 ? 1 : 0;
		displayLightSensor(dark);

	}
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }
	return light;
}

//-------------------------------------
//-  Function: lights_turn
//-------------------------------------
int task_lights_turn()
{
  if (emg_mode)
    return EMERGENCY_MODE;
  char request[10];
  char answer[10];

    // Check is variable is dark or not
	if(dark) {
		strcpy(request, "LAM: SET\n");
	} else {
		strcpy(request, "LAM: CLR\n");
	}
	displayLamps(dark);

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    if (strcmp(answer,"LAM:  OK\n")==0){
    	return 1;
    }
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
	return -1;
}

//-------------------------------------
//-  Function: lights_turn_brake_mode
//-------------------------------------
int task_lights_turn_brake_mode()
{
  if (emg_mode)
    return EMERGENCY_MODE;
	char request[10];
  char answer[10];

  // Turn on since it is in braking mode
	strcpy(request, "LAM: SET\n");
	displayLamps(1);

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }
	return strcmp(answer,"LAM:  OK\n");
}

//-------------------------------------
//-  Function: task_lights_emg_mode
//-------------------------------------
int task_lights_emg_mode()
{
	return task_lights_turn_brake_mode();
}


//-------------------------------------
//-  Function: task_read_movement
//-------------------------------------
int task_read_movement()
{
  if (emg_mode)
    return EMERGENCY_MODE;
  char request[10];
  char answer[10];

  //clear request and answer
  memset(request,'\0',10);
  memset(answer,'\0',10);

  // request movement
  strcpy(request, "STP: REQ\n");

#ifdef RASPBERRYPI
  // use Raspberry Pi I2C serial module
  write(fd_i2c, request, MSG_LEN);
  nanosleep(&time_msg, NULL);
  read(fd_i2c, answer, MSG_LEN);
  answer[8] = '\n';
#else
  //Use the simulator
  simulator(request, answer);
#endif
  if (strcmp(answer, error_string)==0){
    emg_mode = 1;
    return EMERGENCY_MODE;
  }
  if(strcmp(answer, "STP:  GO\n") == 0){
    displayStop(0);
    return NORMAL_MODE;

  }
  else if(strcmp(answer, "STP:STOP\n")==0){
    displayStop(1);
    return STOP_MODE;
  }
  else {
		// Error
	  displayStop(0);
	  return NORMAL_MODE;
	}

}

//-------------------------------------
//-  Function: task_distance()
//-------------------------------------
int task_distance()
{
  if (emg_mode)
    return EMERGENCY_MODE;

  char request[10];
  char answer[10];

  //clear request and answer
  memset(request,'\0',10);
  memset(answer,'\0',10);

  // request distance
  strcpy(request, "DS:  REQ\n");

  #ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
  #else
    //Use the simulator
    simulator(request, answer);
  #endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    if(sscanf(answer, "DS:%u\n", &current_distance) == 1){
      displayDistance(current_distance);

    	if(current_distance < 11000 && current_distance > 0) {
            return BRAKING_MODE;
    	}else{
    		return NORMAL_MODE;
    	}

    } // Error Reading
    else{
      return NORMAL_MODE;
    }

}

//-------------------------------------
//-  Function: task_distance_brake_mode()
//-------------------------------------
int task_distance_brake_mode()
{
  if (emg_mode)
    return EMERGENCY_MODE;

  char request[10];
  char answer[10];

  //clear request and answer
  memset(request,'\0',10);
  memset(answer,'\0',10);

  // request distance in brake mode
  strcpy(request, "DS:  REQ\n");

  #ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
  #else
    //Use the simulator
    simulator(request, answer);
  #endif
    if (strcmp(answer, error_string)==0){
      emg_mode = 1;
      return EMERGENCY_MODE;
    }
    if(sscanf(answer, "DS:%u\n", &current_distance) == 1){
      displayDistance(current_distance);

    	if(current_distance <= 0 && speed <= 10) {
            current_distance = 0;
            displayDistance(current_distance);
            return STOP_MODE;
    	} else {
    		return BRAKING_MODE;
    	}

    } // Error Reading
    else{
      return STOP_MODE;
    }

}

//-------------------------------------
//-  Function: enable_emg_mode
//-------------------------------------
int enable_emg_mode()
{
    char request[10];
    char answer[10];

    // Check if it's already in Emergency Mode
    if (emg_mode==1) return EMERGENCY_MODE;

    //clear request and answer
    memset(request,'\0',10);
    memset(answer,'\0',10);

    // Send the emg mode to the Arduino
    strcpy(request, "ERR: SET\n");

#ifdef RASPBERRYPI
    // use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    //Use the simulator
    simulator(request, answer);
#endif
    // Check The answer from arduino
    if(strcmp(answer, "ERR:  OK\n") == 0){
      emg_mode=1;
		  return EMERGENCY_MODE;
    }
    return EMERGENCY_MODE;
}

//-------------------------------------
//-  Function: Normal execution
//-------------------------------------
int normal_execution(){
  int mode = NORMAL_MODE;
  int secondary_cycle = 0;
  struct timespec start, end, diff, period;
  period.tv_sec = (time_t) TIME_CYCLE_SEC;
  period.tv_nsec = (long) 0;
  if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      printf("Error obtaining starting time\n");
  }

  while (mode == NORMAL_MODE){
    switch(secondary_cycle){
        case 0:
            task_slope();
            mode = task_distance();
            task_mixer();
            task_light_sensor();
            task_lights_turn();
            break;

        case 1:
            task_speed();
            task_acc();
            task_brake();
            task_light_sensor();
            task_lights_turn();
            break;
    }
    if(clock_gettime(CLOCK_REALTIME, &end)==-1){
    	printf("Error obtaining ending time\n");
    }
    secondary_cycle = (secondary_cycle+1) %2;
    diffT(end, start, &diff);
    diffT(period, diff, &diff);
    nanosleep(&diff, NULL);
    addT(start, period, &start);
  }

  return mode;
}

//-------------------------------------
//-  Function: Braking execution
//-------------------------------------
int braking_execution(){
  int mode = BRAKING_MODE;
  int secondary_cycle = 0;

  struct timespec start, end, diff, period;
  period.tv_sec = (time_t) TIME_CYCLE_SEC;
  period.tv_nsec = (long) 0;
  if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      printf("Error obtaining starting time\n");
  }

  while (mode == BRAKING_MODE){
    switch(secondary_cycle){
        case 0:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_slope();
            mode = task_distance_brake_mode();
            break;

        case 1:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_mixer();
            break;
        case 2:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_slope();
            mode = task_distance_brake_mode();
          break;
        case 3:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_mixer();
          break;
        case 4:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_slope();
            mode = task_distance_brake_mode();
          break;
        case 5:
            task_speed();
            task_acc_brake_mode();
            task_brake_brake_mode();
            task_lights_turn_brake_mode();
          break;

    }
    if(clock_gettime(CLOCK_REALTIME, &end)==-1){
    	printf("Error obtaining ending time\n");
    }
    secondary_cycle = (secondary_cycle+1) %6;
    diffT(end, start, &diff);
    diffT(period, diff, &diff);
    nanosleep(&diff, NULL);
    addT(start, period, &start);

  }
  return mode;
}

//-------------------------------------
//-  Function: Stop execution
//-------------------------------------
int stop_execution(){
  int mode = STOP_MODE;

  struct timespec start, end, diff, period;
  period.tv_sec = (time_t) TIME_CYCLE_SEC;
  period.tv_nsec = (long) 0;
  if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      printf("Error obtaining starting time\n");
  }
  while (mode == STOP_MODE){
    mode = task_read_movement();
    task_mixer();
    task_lights_turn_brake_mode();

    if(clock_gettime(CLOCK_REALTIME, &end)==-1){
    	printf("Error obtaining ending time\n");
    }
    diffT(end, start, &diff);
    diffT(period, diff, &diff);
    nanosleep(&diff, NULL);
    addT(start, period, &start);
  }
  return mode;
}

//-------------------------------------
//-  Function: Emergency execution
//-------------------------------------
int emg_execution(){
  int mode = EMERGENCY_MODE;
  int secondary_cycle = 0;
  struct timespec start, end, diff, period;
  period.tv_sec = (time_t) TIME_CYCLE_SEC;
  period.tv_nsec = (long) TIME_CYCLE_NSEC;
  if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      printf("Error obtaining starting time\n");
  }

  while (mode == EMERGENCY_MODE){
    switch(secondary_cycle){
        case 0:
            task_slope_emg_mode();
            task_mixer_emg_mode();
            enable_emg_mode();
            task_lights_emg_mode();
            break;

        case 1:
            task_speed_emg_mode();
            task_acc_emg_mode();
            task_brake_emg_mode();
            task_lights_emg_mode();
            break;
    }
    if(clock_gettime(CLOCK_REALTIME, &end)==-1){
    	printf("Error obtaining ending time\n");
    }
    secondary_cycle = (secondary_cycle+1) %2;
    diffT(end, start, &diff);
    diffT(period, diff, &diff);
    nanosleep(&diff, NULL);
    addT(start, period, &start);

  }
  return mode;
}

//-------------------------------------
//-  Function: controller
//-------------------------------------
void *controller(void *arg)
{
    int mode = 0;
    mixer_state = 0;
    clock_gettime( CLOCK_REALTIME, &time_last_change_mixer);

    // Endless loop
    while(1) {
      switch(mode){
        case NORMAL_MODE: //Normal mode
          mode = normal_execution();
          break;
        case BRAKING_MODE: //Braking mode
          mode = braking_execution();
          break;
        case STOP_MODE: //Stop mode
          mode = stop_execution();
          break;
        case EMERGENCY_MODE: //Emergency mode
          mode = emg_execution();
          break;
      }
    }
}

//-------------------------------------
//-  Function: Init
//-------------------------------------
rtems_task Init (rtems_task_argument ignored)
{
    pthread_t thread_ctrl;
    sigset_t alarm_sig;
    int i;

    /* Block all real time signals so they can be used for the timers.
     Note: this has to be done in main() before any threads are created
     so they all inherit the same mask. Doing it later is subject to
     race conditions */
    sigemptyset (&alarm_sig);
    for (i = SIGRTMIN; i <= SIGRTMAX; i++) {
        sigaddset (&alarm_sig, i);
    }
    sigprocmask (SIG_BLOCK, &alarm_sig, NULL);

    // init display
    displayInit(SIGRTMAX);

#ifdef RASPBERRYPI
    // Init the i2C driver
    rpi_i2c_init();

    // bus registering, this init the ports needed for the conexion
    // and register the device under /dev/i2c
    rpi_i2c_register_bus("/dev/i2c", 10000);

    // open device file
    fd_i2c = open("/dev/i2c", O_RDWR);

    // register the address of the slave to comunicate with
    ioctl(fd_i2c, I2C_SLAVE, SLAVE_ADDR);
#endif

    /* Create first thread */
    pthread_create(&thread_ctrl, NULL, controller, NULL);
    pthread_join (thread_ctrl, NULL);
    exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_MAXIMUM_TASKS 1
#define CONFIGURE_MAXIMUM_SEMAPHORES 10
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 30
#define CONFIGURE_MAXIMUM_DIRVER 10
#define CONFIGURE_MAXIMUM_POSIX_THREADS 2
#define CONFIGURE_MAXIMUM_POSIX_TIMERS 1

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
