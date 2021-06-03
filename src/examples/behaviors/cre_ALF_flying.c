#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define COLLISION_BITS 6
#define SECTORS_IN_COLLISION 2
#define ARGOS_SIMULATION
#define RESOURCES_SIZE 2

typedef enum
{ // Enum for different motion types
  TURN_LEFT = 1,
  TURN_RIGHT = 2,
  STOP = 3,
  FORWARD = 4,
} motion_t;

typedef enum
{ // Enum for boolean flags
  false = 0,
  true = 1,
} bool;

typedef enum
{ // Enum for the robot states
  UNCOMMITTED = 0,
  COMMITTED_N = 1,
  COMMITTED_S = 2,
} decision_t;

typedef enum
{ // Enum for the robot position wrt to areas
  LEFT = 1,
  RIGHT = 2,
} Free_space;

typedef enum
{ // Enum for the type of the resource
  RESOURCE_NORTH = 0,
  RESOURCE_SOUTH = 1,
} resource_t;

/*State*/
motion_t current_motion_type = STOP;             // Current motion type
decision_t current_decision_state = UNCOMMITTED; // Current state

/*Random walk*/
const float std_motion_steps = 20 * 16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2;          // 2 is brownian like motion (alpha)
const float crw_exponent = 0.4;         // higher more straight (rho)
uint32_t turning_ticks = 0;             // keep count of ticks of turning
const uint8_t max_turning_ticks = 120;  /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
uint32_t straight_ticks = 0;            // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;
/* ---------------------------------------------- */

/***********WALL AVOIDANCE***********/
// the kb is "equipped" with a proximity sensor

const uint8_t sector_base = (pow(2, COLLISION_BITS / 2) - 1);
uint8_t left_side = 0;
uint8_t right_side = 0;
uint8_t proximity_sensor = 0;
Free_space free_space = LEFT;
bool wall_avoidance_start = false;

/* ---------------------------------------------- */
/** Variables for Smart Arena messages */
uint8_t sa_payload_north = 0;
uint8_t sa_payload_south = 0;
bool new_sa_msg = false;
uint8_t sent_message = 1;
bool to_send_message = false;
message_t messageN;
message_t messageS;

/*Decision making*/
const float tau = 1;                     //accelerare o rallentare il sistema
const float h = 0.2;                     //0.1111111; self
const float k = 0.8;                     //0.8888889; recruitment
const uint16_t max_decision_ticks = 320; //10 secondi
uint32_t last_decision_ticks = 0;

uint8_t internal_error = 0;                      //computation error
const float ema_alpha = 0.1;                     //exponential moving average
uint8_t resources_pops[RESOURCES_SIZE] = {0, 0}; //keep local knowledge about resources
uint8_t recruiter_state = UNCOMMITTED;           //commitment communicated by another robot
uint8_t communicated_by = 100;                   //kID of communicated commitment

/*-------------------------------------------------------------------*/
/* count 1s after decimal to binary conversion                       */
/*-------------------------------------------------------------------*/
uint8_t countOnes(uint8_t n)
{
  uint8_t count = 0;
  // array to store binary number
  // uint8_t binaryNum[8];

  // counter for binary array
  int i = 0;
  while (n > 0)
  {

    // storing remainder in binary array
    // binaryNum[i] = n % 2;
    if ((n % 2) == 1)
      count++;
    n = n / 2;
    i++;
  }

  return count;
}

/*-------------------------------------------------------------------*/
/* Function implementing wall avoidance procedure                    */
/*-------------------------------------------------------------------*/
void wall_avoidance_procedure(uint8_t sensor_readings)
{
  right_side = sensor_readings & sector_base;
  left_side = (sensor_readings >> (COLLISION_BITS / 2)) & sector_base;

  uint8_t count_ones = countOnes(sensor_readings);
  if (count_ones > SECTORS_IN_COLLISION)
  {
    if (right_side < left_side)
    {
      // set_color(RGB(0,0,3));
      set_motion(TURN_RIGHT);
      free_space = RIGHT;
    }
    else if (right_side > left_side)
    {
      // set_color(RGB(3,0,0));
      set_motion(TURN_LEFT);
      free_space = LEFT;
    }

    else
    {
      set_motion(free_space);
    }
    if (kilo_ticks > last_motion_ticks + turning_ticks)
    {
      turning_ticks = (uint32_t)((M_PI / COLLISION_BITS) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
  }
  // else
  // {
  //   set_color(RGB(0,3,0));
  // }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void exponential_average(uint8_t resource_id, uint8_t resource_pop)
{
  // update by using exponential moving averagae to update estimated population
  resources_pops[resource_id] = (uint8_t)round(((float)resource_pop * (ema_alpha)) + ((float)resources_pops[resource_id] * (1.0 - ema_alpha)));
}

/*-------------------------------------------------------------------*/
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
  // index of first element in the 3 sub-blocks of data
  uint8_t shift = kb_index * 3;
  sa_payload_north = ((data[shift] << 4) | (data[shift + 1] >> 4)) & 0x3F;
  sa_payload_south = ((data[shift + 1] << 2) | (data[shift + 2] >> 6)) & 0x3F;

  if (sa_payload_north != 0)
  {
    exponential_average(RESOURCE_NORTH, sa_payload_north);
  }
  if (sa_payload_south != 0)
  {
    exponential_average(RESOURCE_SOUTH, sa_payload_south);
  }
  new_sa_msg = true;

  proximity_sensor = (data[shift + 2]) & 0x3F;
  if (proximity_sensor != 0)
  {
    wall_avoidance_start = true;
  }

  /** Print behaviour for the kilobot 0 */
  // if (kilo_uid == 0)
  // {
  //   printf("N:%d \tS:%d \t WA:%d \n", sa_payload_north, sa_payload_south, proximity_sensor);
  // }
}

void rx_message(message_t *msg, distance_measurement_t *d)
{
  /* Unpack the message - extract ID, type and payload */
  /*type 0 is for messages coming from ARK*/
  if (msg->type == 0)
  {
    int id1 = (msg->data[0] >> 2);
    int id2 = (msg->data[3] >> 2);
    int id3 = (msg->data[6] >> 2);
    if (id1 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 0);
    }
    else if (id2 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 1);
    }
    else if (id3 == kilo_uid)
    {
      parse_smart_arena_message(msg->data, 2);
    }
  }

  /*type 1 is for messages coming from other kilobots*/
  else if (msg->type == 1)
  {
    // printf("Receiving message\t");
    // printf("msg->data[0]= %d\n", msg->data[0]);
    /* get id (always firt byte when coming from another kb) */
    uint8_t id = msg->data[0];
    // check that is a valid crc and another kb
    if (id != kilo_uid)
    {
      // store the message for later parsing to avoid the rx to interfer with the loop
      recruiter_state = msg->data[1];
      communicated_by = id;
      // printf("recruiter_state: %d\n", recruiter_state);
    }
  }
}

/*-------------------------------------------------------------------*/
/* Message transmission                                              */
/*-------------------------------------------------------------------*/
message_t *message_tx()
{
  if (to_send_message == true)
  {
    /* this one is filled in the loop */
    to_send_message = false;

    if (current_decision_state == COMMITTED_N)
    {
      return &messageN;
    }
    else if (current_decision_state == COMMITTED_S)
    {
      return &messageS;
    }
    else
    {
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}
void message_tx_success()
{
  sent_message = 1;
}
/*-------------------------------------------------------------------*/
/* For Brodcast                                                      */
/*-------------------------------------------------------------------*/
/* Tell the kilobot to send its own state */
void send_own_state()
{
  if (current_decision_state != UNCOMMITTED)
  {

    // printf("kID: %d, SENDING current_decision_state: %d\n", kilo_uid, current_decision_state);
    to_send_message = true;
    // avoid rebroadcast to overwrite prev message
    sent_message = 0;
  }
}
/*-------------------------------------------------------------------*/
/* Decision Making Function                                          */
/* Sets ths current_decision var                                     */
/*-------------------------------------------------------------------*/
void take_decision()
{
  if (kilo_ticks >= max_decision_ticks + last_decision_ticks)
  {
    // temp variable used all along to account for quorum sensing
    uint8_t resource_index = 0;
    /* Start decision process */
    if (current_decision_state == UNCOMMITTED)
    {
      // printf("kID:%d, UNCOMMITTED\n", kilo_uid);
      uint8_t commitment = 0;
      /****************************************************/
      /* spontaneous commitment process through discovery */
      /****************************************************/
      uint8_t random_resource = rand_soft() % RESOURCES_SIZE;
      // printf("kID:%d, random resource:%d\n", kilo_uid, random_resource);
      // normalized between 0 and 63
      // printf("kID:%d, resources_pops[random_resource]:%d\n", kilo_uid, resources_pops[random_resource]);
      commitment = (uint8_t)floor((float)resources_pops[random_resource] * h * tau);
      // printf("kID:%d, commitment:%d\n", kilo_uid, commitment);
      /****************************************************/
      /* recruitment over a random agent            m_sData      */
      /****************************************************/
      uint8_t recruitment = 0;
      // if the recruiter is committed
      if (recruiter_state != UNCOMMITTED)
      {
        // printf("kID:%d, recruiter_state UNCOMMITTED\n", kilo_uid);
        /* get the correct index in case of quorum sensing mechanism */
        resource_index = recruiter_state - 1;
        // compute recruitment value for current agent
        recruitment = (uint8_t)floor((float)resources_pops[resource_index] * k * tau);
        // printf("kID:%d, recruitment:%d\n", kilo_uid, recruitment);
      }
      /****************************************************/
      /* extraction                                       */
      /****************************************************/
      /* check if the sum of all processes is below 1 (here 63 since normalize to 6 bit) */
      if ((uint16_t)commitment + (uint16_t)recruitment > 63)
      {
        internal_error = true;
        printf("kID:%d, Internal error true\n", kilo_uid);
        return;
      }
      // a random number to extract next decision
      uint8_t extraction = (uint8_t)(rand_soft() % 64);
      // printf("kID:%d, extraction:%d\n", kilo_uid, extraction);
      // if the extracted number is less than commitment, then commit
      if (extraction < commitment)
      {
        current_decision_state = (decision_t)(random_resource + 1); //RICONTROLLA
        // printf("kID:%d, extraction < commitment, current_decision_state:%d\n", kilo_uid, current_decision_state);
      }
      else if (extraction < recruitment + commitment)
      {
        current_decision_state = (decision_t)recruiter_state;
        // printf("kID:%d, extraction < recruitment, current_decision_state:%d\n", kilo_uid, current_decision_state);
      }
    }

    else
    {
      /****************************************************/
      /* cross inhibtion over a random agent              */
      /****************************************************/
      uint8_t cross_inhibition = 0;
      // if the inhibitor is committed or in quorum but not same as us
      if (recruiter_state != UNCOMMITTED && current_decision_state != recruiter_state)
      {
        /* get the correct index in case of quorum sensing mechanism */
        resource_index = recruiter_state - 1;
        // compute recruitment value for current agent
        cross_inhibition = (uint8_t)floor((float)resources_pops[resource_index] * k * tau);
      }
      /****************************************************/
      /* extraction                                       */
      /****************************************************/
      /* check if the sum of all processes is below 1 (here 63 since normalized with 6 bits) */
      if ((uint16_t)cross_inhibition > 63)
      {
        internal_error = true;
        printf("kID:%d, Internal error cross-inhibition true\n", kilo_uid);
        return;
      }
      // a random number to extract next decision
      uint8_t extraction = (uint8_t)(rand_soft() % 64); // rand() / (RAND_MAX / N + 1)
      // printf("kID:%d, extraction:%d\n", kilo_uid, extraction);
      // subtract cross-inhibition
      if (extraction < cross_inhibition)
      {
        // printf("kID:%d, cross-inhibition --> UNCOMMITTED\n", kilo_uid);
        current_decision_state = UNCOMMITTED;
        set_color(RGB(0, 0, 0));
      }
    }
    /* erase memory of neighbour commitment*/
    recruiter_state = UNCOMMITTED;
    last_decision_ticks = kilo_ticks;
    //update led
    switch (current_decision_state)
    {
    case (COMMITTED_N):
    {
      set_color(RGB(3, 0, 0));
      break;
    }
    case (COMMITTED_S):
    {
      set_color(RGB(0, 0, 3));
      break;
    }
    case (UNCOMMITTED):
    {
      set_color(RGB(0, 0, 0));
      break;
    }
    }
    // printf("recruiter ID:%d, state:%d\n", communicated_by, recruiter_state);
    // printf("kID:%d deciding... %d\n", kilo_uid, current_decision_state);
    // printf("resources_pops[0]: %d\n", resources_pops[0]);
    // printf("resources_pops[1]: %d\n\n", resources_pops[1]);

    send_own_state();
  }
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
  bool calibrated = true;
  if (current_motion_type != new_motion_type)
  {
    switch (new_motion_type)
    {
    case FORWARD:
      spinup_motors();
      if (calibrated)
        set_motors(kilo_straight_left, kilo_straight_right);
      else
        set_motors(70, 70);
      break;
    case TURN_LEFT:
      spinup_motors();
      if (calibrated)
        set_motors(kilo_turn_left, 0);
      else
        set_motors(70, 0);
      break;
    case TURN_RIGHT:
      spinup_motors();
      if (calibrated)
        set_motors(0, kilo_turn_right);
      else
        set_motors(0, 70);
      break;
    case STOP:
    default:
      set_motors(0, 0);
    }
    current_motion_type = new_motion_type;
  }
}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk()
{
  switch (current_motion_type)
  {
  case TURN_LEFT:
  case TURN_RIGHT:
    /* if turned for enough time move forward */
    if (kilo_ticks > last_motion_ticks + turning_ticks)
    {
      /* start moving forward */
      last_motion_ticks = kilo_ticks;
      set_motion(FORWARD);
    }
    break;

  case FORWARD:
    /* if moved forward for enough time turn */
    if (kilo_ticks > last_motion_ticks + straight_ticks)
    {
      /* perform a random turn */
      last_motion_ticks = kilo_ticks;
      if (rand_soft() % 2)
      {
        set_motion(TURN_LEFT);
      }
      else
      {
        set_motion(TURN_RIGHT);
      }
      double angle = 0;
      /* random angle */
      if (crw_exponent == 0)
      {
        angle = uniform_distribution(0, (M_PI));
      }
      else
      {
        angle = fabs(wrapped_cauchy_ppf(crw_exponent));
      }

      /* compute turning time */
      turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);
      straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
    }
    break;

  case STOP:
  default:
    set_motion(FORWARD);
  }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
  /* Initialise LED and motors */
  set_color(RGB(0, 0, 0));
  set_motors(0, 0);

  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  seed = rand_hard();
  srand(seed);

  /* Initialise motion variables */
  last_motion_ticks = rand() % max_straight_ticks;
  set_motion(FORWARD);

  /* Precompile possible messages*/
  messageN.type = 1;
  messageN.data[0] = kilo_uid;
  messageN.data[1] = COMMITTED_N;
  messageN.crc = message_crc(&messageN);

  messageS.type = 1;
  messageS.data[0] = kilo_uid;
  messageS.data[1] = COMMITTED_S;
  messageS.crc = message_crc(&messageS);
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
  if (wall_avoidance_start)
  {
    wall_avoidance_procedure(proximity_sensor);
    proximity_sensor = 0;
    wall_avoidance_start = false;
  }
  else
  {
    random_walk();
  }
  // send_own_state();
  take_decision();
}
int main()
{
  kilo_init();
  kilo_message_rx = rx_message;
  kilo_message_tx = message_tx;
  kilo_message_tx_success = message_tx_success;
  kilo_start(setup, loop);
  return 0;
}