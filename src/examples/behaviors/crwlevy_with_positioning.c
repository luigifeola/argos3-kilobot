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
// # define NDEBUG 
# include <assert.h> 


/* Enum for different motion types */
typedef enum {
    FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
    WAIT_ANGLE = 4
} motion_t;

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;


/* Enum for the robot states */
typedef enum {
    OUTSIDE_TARGET = 0,
    DISCOVERED_TARGET = 1,
    COMMUNICATED_TARGET = 2,
    BIASING = 3
} state_t;

/* current motion type */
motion_t current_motion_type = STOP;

/* current state */
state_t current_state = OUTSIDE_TARGET;
state_t previous_state = OUTSIDE_TARGET;

/* Message send to the other kilobots */
message_t messageA;

/* Variables for Smart Arena messages */
int sa_type = 3;    //valore impossibile da assumere state_t in [0,2]
int sa_payload = 0;
bool new_sa_msg = false;

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

/*Parameters from ARK*/
double levy_exponent = -1; // 2 is brownian motion
double crw_exponent = -1; // go straight often

/* Bias parameters */
double bias_prob = -1;    // probability of facing home 
double bias_angle = -1;
motion_t bias_rotation = STOP;
// uint8_t previous_state_color = RGB(0,0,0);
// uint8_t previous_state = RGB(0,0,0);

uint32_t last_motion_ticks = 0;
/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;

void my_printf(const char *fmt, ...)
{
#if REAL
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);

  va_end(args);
#endif
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
  if (current_motion_type != new_motion_type)
  {
    switch (new_motion_type)
    {
    case FORWARD:
      spinup_motors();
      set_motors(kilo_straight_left, kilo_straight_right);
      break;
    case TURN_LEFT:
      spinup_motors();
      set_motors(kilo_turn_left, 0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      set_motors(0, kilo_turn_right);
      break;
    
    case WAIT_ANGLE:
    case STOP:
    default:
      set_motors(0, 0);
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{

  /* Initialise LED and motors */
  set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
  
  /* Initialise motion variables */
  set_motion(STOP);

  /* Initialise KBots message */
  messageA.type = 1;  //0 for ARK, 1 for KBots
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
  sending_msg = false;
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/* as in the crwlevy_ALF.cpp there are 3 kilobots messages per    */
/* message with 3 different kilobots ids.                            */
/*                                                                   */
/* type 0 for ark / type 1 for kbs interactive msgs                  */
/*                                                                   */
/* A message structure is 12 bytes in length and is composed of      */
/* three parts: the payload (9 bytes), the message type (1 byte),    */
/* and a CRC (2 bytes).                                              */
/*-------------------------------------------------------------------*/
void message_rx(message_t *msg, distance_measurement_t *d) {
  
  /* For testing communication */
  // if (msg->type == 253)
  // {
  //   /* Blinking behaviour */
  //   set_color(RGB(0, 3, 0));
  //   delay(50);
  //   set_color(RGB(3, 0, 0));
  //   delay(50);
  // }
  

  /* ----------------------------------*/
  /* smart arena message               */
  /* ----------------------------------*/

  /* Receiving parameters */
  if (msg->type == 255) 
  {
    // unpack message
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
    if (id1 == kilo_uid) {
        // unpack type
        sa_type = msg->data[1] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
        levy_exponent = (double) (sa_payload & 0x1F) /10;
        // crw_exponent = (double) ((sa_payload >> 5) & 0x1F) /10;
        crw_exponent = (double) ((sa_payload & 0xF8) >>5) /10;
        // bias_prob = (double) sa_type/10;
        
        // sa_type in [0,10], so bias_prob checked with rand_soft() in [0,255]
        bias_prob = sa_type * 255 / 10;
        // bias_prob = 6 * 255 / 10;  //60% probability
    }
    // printf("bias_prob = %f\n", bias_prob);
    
    if (id2 == kilo_uid) {
        // unpack type
        sa_type = msg->data[4] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
        // crw_exponent = (double) ((sa_payload >> 5) & 0x1F) /10;
        crw_exponent = (double) ((sa_payload & 0xF8) >>5) /10;
        levy_exponent = (double) (sa_payload & 0x1F) /10;
        bias_prob = sa_type * 255 / 10;
        // bias_prob = 6 * 255 / 10;  //60% probability
    }
    if (id3 == kilo_uid) {
        // unpack type
        sa_type = msg->data[7] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
        // crw_exponent = (double) ((sa_payload >> 5) & 0x1F) /10;
        crw_exponent = (double) ((sa_payload & 0xF8) >>5) /10;
        levy_exponent = (double) (sa_payload & 0x1F) /10;
        bias_prob = sa_type * 255 / 10;
        // bias_prob = 6 * 255 / 10;  //60% probability
    }
    // printf("bias_prob = %f\n", bias_prob);
  }

  /* Receiving kilobt state */
  if (msg->type == 0) 
  {
    // unpack message
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
    if (id1 == kilo_uid) {
        // unpack type
        sa_type = msg->data[1] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[1]&0b11) << 8) | (msg->data[2]);
        new_sa_msg = true;
    }
    if (id2 == kilo_uid) {
        // unpack type
        sa_type = msg->data[4] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
        new_sa_msg = true;
    }
    if (id3 == kilo_uid) {
        // unpack type
        sa_type = msg->data[7] >> 2 & 0x0F;
        // unpack payload
        sa_payload = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
        new_sa_msg = true;
    }

    if(new_sa_msg==true)
    {
      if((sa_type==1)&&(current_state==OUTSIDE_TARGET || current_state==COMMUNICATED_TARGET))
      {
          current_state=DISCOVERED_TARGET;
          new_information = true;
          set_color(RGB(3, 0, 0));
      }
      new_sa_msg = false;
    }
  }

  /* Receiving kilobt BIAS */
  if (msg->type == 254) 
  {
    // unpack message
    int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
    int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
    int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);
    if (id1 == kilo_uid) {
      // unpack payload
      bias_angle = (((msg->data[1]&0b11) << 8) | (msg->data[2])) * M_PI / 255;
      bias_rotation = msg->data[1] >> 2 & 0x0F;
    } 
    if (id2 == kilo_uid) {
      bias_angle = ((msg->data[4]&0b11)  << 8) | (msg->data[5]);
      bias_rotation = msg->data[4] >> 2 & 0x0F;
    }
    if (id3 == kilo_uid) {
      bias_angle = ((msg->data[7]&0b11)  << 8) | (msg->data[8]);
      bias_rotation = msg->data[7] >> 2 & 0x0F;
    }
    
    // printf("bias_angle = %f\n", bias_angle);
  }


  //The remaining possibility is for messages arriving from other kilobots
  //So if distance is too much, the message will be discarded
  uint8_t cur_distance = estimate_distance(d);
  if (cur_distance > 100) //100 mm  
  {
    return;
  }

  /* ----------------------------------*/
  /* KB interactive message            */
  /* ----------------------------------*/

  else if (msg->type==1 && msg->data[0]!=kilo_uid && msg->crc==message_crc(msg)) 
  {
    // printf("Receiving information about the target\n");
    new_information = true;
    if (current_state != DISCOVERED_TARGET)
    {
      current_state = COMMUNICATED_TARGET;
      set_color(RGB(0, 3, 0));
    }
  }
}

/*-------------------------------------------------------------------*/
/* Function to broadcast a message                                        */
/*-------------------------------------------------------------------*/
void broadcast()
{

  if (new_information && !sending_msg && kilo_ticks > last_broadcast_ticks + max_broadcast_ticks)
  {
    last_broadcast_ticks = kilo_ticks;
    sending_msg = true;
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the crwlevy random walk                     */
/*-------------------------------------------------------------------*/
void random_walk()
{
  switch (current_motion_type) 
  {
  case TURN_LEFT:
  case TURN_RIGHT:
    if (kilo_ticks > last_motion_ticks + turning_ticks) {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;
  case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks) {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      

      // rand_soft e' fra 0 e 255 mentre a te serve il 20%. 
      // Quindi o normalizzi rand_soft of usi il 20% di 255 che e' 51
      if(bias_prob-rand_soft() > 0) 
      {
        // printf("It's time to rotate \n");
        set_color(RGB(3,3,3));
        set_motion(WAIT_ANGLE);
        delay(1000);
        previous_state = current_state;
        current_state = BIASING;
      }
      else if (rand_soft() % 2)
      {
        set_motion(TURN_LEFT);
      }
      else
      {
        set_motion(TURN_RIGHT);
      }
      double angle = 0;
      if(crw_exponent == 0) 
      {
        angle = (uniform_distribution(0, (M_PI)));
        // my_printf("%" PRIu32 "\n", turning_ticks);
        // my_printf("%u" "\n", rand());
      }
      else
      {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
      // my_printf("%u" "\n", straight_ticks);
    }
    break;
  case WAIT_ANGLE:
    if (bias_angle != -1)
    {
      // update lat motion to current
      // turning ticks based on agle from ark
      current_state = previous_state;
      switch (current_state)
      {
        case OUTSIDE_TARGET:
          set_color(RGB(0,0,0));
          break;
        case DISCOVERED_TARGET:
          set_color(RGB(3,0,0));
          break;
        case COMMUNICATED_TARGET:
          set_color(RGB(0,3,0));
          break;
        
        default:
          break;
      }
      last_motion_ticks = kilo_ticks;
      turning_ticks = (uint32_t)((bias_angle / M_PI) * max_turning_ticks);
      set_motion(bias_rotation);  // bias_rotation is LEFT or RIGHT
    } 
    else
    {
      // do nothing
    }
    break;
  case STOP:
  default:
    set_motion(FORWARD);
  }
}




/*-------------------------------------------------------------------*/
/* Reset the experiment at start                                     */
/*-------------------------------------------------------------------*/
void check_reset()
{
  if (kilo_ticks == 0) // NOT THE RIGHT TEST, kilo_tick doesnt reinitiate?
  {
    setup();
  }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
  check_reset();


  // set_motion(TURN_RIGHT);
  if(crw_exponent!=-1 && levy_exponent!=-1)
  {
    random_walk();
    broadcast();  
  }
  

  
  
}

/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main()
{
    kilo_init();
    // register message reception callback
    kilo_message_rx = message_rx;
    // register message transmission callback
    kilo_message_tx = message_tx;
    // register tranmsission success callback
    kilo_message_tx_success = message_tx_success;

     
    kilo_start(setup, loop);

    return 0;
}
