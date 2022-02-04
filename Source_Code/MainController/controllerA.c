/**********************************************************
 *  INCLUDES
 *********************************************************/
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <float.h>
#include <time.h>

#include <rtems.h>
#include <bsp.h>

//#define RASPBERRYPI

#ifdef RASPBERRYPI
#include <bsp/i2c.h>
#endif

#include "displayA.h"

/**********************************************************
 *  Constants
 **********************************************************/
#define MSG_LEN    8
#define SLAVE_ADDR 0x8
#define TIME_CYCLE_SEC 10
#define TIME_CYCLE_NSEC 10000000000
#define NS_PER_S  1000000000

/**********************************************************
 *  Global Variables
 *********************************************************/
float speed = 0.0;
struct timespec time_msg = {0,400000000};
int fd_i2c = -1;

struct timespec time_last_change_mixer;
int mixer_state;

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
  // display slope
  if (0 == strcmp(answer, "SLP:DOWN\n")) displaySlope(-1);
  else if (0 == strcmp(answer, "SLP:FLAT\n")) displaySlope(0);
  else if (0 == strcmp(answer, "SLP:  UP\n")) displaySlope(1);
  else{
      return 2;
  }

  return 0;
}
//-------------------------------------
//-  Function: task_acc
//-------------------------------------
int task_acc()
{
    char request[10];
    char answer[10];

    // Clear request and answer
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
    // Use Raspberry Pi I2C serial module
    write(fd_i2c, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read(fd_i2c, answer, MSG_LEN);
    answer[8] = '\n';
#else
    // Use the simulator
    simulator(request, answer);
#endif

    return strcmp(answer, "GAS:  OK\n");
}

//-------------------------------------
//-  Function: task_brake
//-------------------------------------
int task_brake()
{
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

    return strcmp(answer, "BRK:  OK\n");
}

//-------------------------------------
//-  Function: task_mixer
//-------------------------------------
int task_mixer()
{
    char request[10];
    char answer[10];

  // Compute the time when the mixer needs to send the request to the arduino
    struct timespec current, lapse;
  	clock_gettime(CLOCK_REALTIME, &current);
	diffT(current, time_last_change_mixer, &lapse);
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
    // Error
    else{
        return 1;
    }
    return 0;
}


//-------------------------------------
//-  Function: controller
//-------------------------------------
void *controller(void *arg)
{
    mixer_state = 0;
    clock_gettime( CLOCK_REALTIME, &time_last_change_mixer);
    struct timespec start, end, diff, period;
    period.tv_sec = (time_t) TIME_CYCLE_SEC;
    period.tv_nsec = (long) 0;
    if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
        printf("Error obtaining Strating Time\n");
    }
    // Endless loop: In 1 secondaryCycle we execute all tasks:
    while(1) {

      // calling task of slope
     if(task_slope() != 0)
          printf("Error when reading slope\n");
      // calling task of speed
      if(task_speed() != 0)
          printf("Error when reading speed\n");
      // calling task of gas
      if(task_acc() != 0)
          printf("Error when reading gas\n");
      // calling task of brake
      if(task_brake() != 0)
          printf("Error when reading brake\n");
      // calling task of mixer
      if(task_mixer() != 0)
          printf("Error when reading mixer\n");

    //Get end of main cycle
    if( clock_gettime( CLOCK_REALTIME, &end) == -1 ) {
      printf("Error obtaining ending time\n");
    }

    // Sleep the neccesary time until makes the next iteration
    diffT(end, start, &diff);
    diffT(period, diff, &diff);

    nanosleep(&diff, NULL);
    addT(start, period, &start);

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
