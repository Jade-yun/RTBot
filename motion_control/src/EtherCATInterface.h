#ifndef CONFIG_HCFA_H
#define CONFIG_HCFA_H

#include <iostream>
#include <string.h>

#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sys/types.h>

#include <etherlab/ecrt.h>

typedef enum
{
  no_ready_to_switch_on = 0,
  /* Low level power(e.g. +/- 15V, 5V) has been applied to the drive.
   *  The drive is being initialized or is running self test.
   *  A brake, if present, has to be applied in this state.
   *  The drive function is disable.
   * */

  switch_on_disable,
  /* Drive initialization is complete.
   * The drive parameters have been set up.
   * Drive parameters may be changed.
   * High voltage may not be applied to the dirve.
   * The drive function is disabled.
   * */

  ready_to_switch_on,
  /* High voltage may be applied to the drive.
   * The drive parameters may be changed.
   * The drive function is disabled.
   * */

  switched_on,
  /* High voltage has been applied to the drive.
   * The power amplifier is ready.
   * The dirve parameters may be change.
   * The drive functuion is disable.
   * */
  operation_enable,
  /* No faults have been detected.
   * The dirve  function is enabled and power is apllied to the motor.
   * The dirve function is enable.
   * */

  quick_stop_active,
  /* The drive paramters may be changed.
   * The quick stop function is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault_reaction_active,
  /* The parameters may be changed.
   * A fault has occurred in the drive.
   * The quick stop functon is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault,
  /* The drive parameters may be changed.
   * A fault has occurred in the drive.
   * High voltage switch-on/off depends on the application.
   * The drive function is disabled.
   * */
  none
} cia402_state_t;

/*************************************************************************** */
//judge axis status
extern cia402_state_t get_axis_state(uint16_t status_word);


// #define PTHREAD_STACK_MIN 16384
// #define MY_STACK_SIZE 1024 * 1024

//slaves num vid pid
#define MASTER_ID 0
#define HCFA_X5_VID 0x000116c7
#define HCFA_X5_PID 0x005e0402

#define NUM_SLAVE 3
#define NUM_PDOS 8

//Application parameters
#define  FREQUENCY 125
#define  CLOCK_TO_USE CLOCK_REALTIME

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)


/*Offsets for PDO entries*/
static struct{
    unsigned int Status_Word[NUM_SLAVE];
    unsigned int Control_word[NUM_SLAVE];
    unsigned int Target_Position[NUM_SLAVE];
    unsigned int Position_Actual_Value[NUM_SLAVE];
    unsigned int Control_Mode_Display[NUM_SLAVE];
    unsigned int Control_Mode[NUM_SLAVE];
    unsigned int Target_velocity[NUM_SLAVE];
    unsigned int Velocity_actual_value[NUM_SLAVE];
}offset;

typedef struct {
    int PDOIndex;
    unsigned int *Offset;
} pdo_offset_t;


extern int init_ethercat();
extern void *simple_cyclic_task(void *data);
extern void init_pdo_entries(void);
extern void delay_nanoseconds(long nanoseconds);
extern void check_slave_config_states(ec_slave_config_t *sc, int i);
extern void check_master_state(void);
extern void check_domain_state(void);
extern void signal_handler(int signum);

#endif