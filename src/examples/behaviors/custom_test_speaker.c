/* Kilobot control software for the simple ALF experment : clustering
 * author: Mohamed Salaheddine Talamali (University of Sheffield) mstalamali1@sheffield.ac.uk
 */

#include "kilolib.h"
#include <math.h>
#include "distribution_functions.c"
// ****************************************
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if REAL
#define DEBUG
#include <stdarg.h>
#include "debug.h"
#include <avr/eeprom.h>
#else
#include <inttypes.h>
#endif

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


/* Message send to the other kilobots */
message_t messageA;

/* Flag for decision to send a word */
bool sending_msg = false;

/*Flag for the existence of information*/
bool new_information = false;

/*Max number of ticks*/
//const double max_time = 9288; //10^3
/* counters for motion, turning and random_walk */
const double std_motion_steps = 5*16;
uint32_t turning_ticks = 0; // keep count of ticks of turning
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
uint16_t straight_ticks = 0; // keep count of ticks of going straight


uint32_t last_motion_ticks = 0;
/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
// void set_motion(motion_t new_motion_type)
// {
//   if (current_motion_type != new_motion_type)
//   {
//     switch (new_motion_type)
//     {
//     case FORWARD:
//       spinup_motors();
//       set_motors(kilo_straight_left, kilo_straight_right);
//       break;
//     case TURN_LEFT:
//       spinup_motors();
//       set_motors(kilo_turn_left, 0);
//       break;
//     case TURN_RIGHT:
//       spinup_motors();
//       set_motors(0, kilo_turn_right);
//       break;
//     case STOP:
//     default:
//       set_motors(0, 0);
//     }
//     current_motion_type = new_motion_type;
//   }
// }

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{

  /* Initialise LED and motors */
  // set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  
  /* Initialise motion variables */
  // set_motion(FORWARD);

  /* Initialise KBots message */
  messageA.type = NORMAL;
  messageA.data[0] = kilo_uid;
  messageA.crc = message_crc(&messageA);
}


/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
  if (sending_msg)
  {
    /* this one is filled in the loop */
    return &messageA;
  }
  return 0;
}

/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void message_tx_success() {
  sending_msg = true;
}






/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
  // printf("CRW: %f \n", crw_exponent);
  // printf("LEVY: %f \n", levy_exponent);
//   random_walk();
//   broadcast();
   if(sending_msg)
   {
      sending_msg = false;
      set_color(RGB(1, 0, 1));
      delay(100);
      set_color(RGB(0, 0, 0));
   }
}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
    // register message transmission callback
    kilo_message_tx = message_tx;
    // register tranmsission success callback
    kilo_message_tx_success = message_tx_success;

     
    kilo_start(setup, loop);

    return 0;
}
