#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define COLLISION_BITS 8
#define SECTORS_IN_COLLISION 2
#define ARGOS_SIMULATION

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
    RANDOM_WALKING = 0,
    TURNING_NORTH = 1,
    TURNING_SOUTH = 2,
    WALL_AVOIDANCE = 3,
    MOVING_TO_TARGET = 4,
} action_t;

typedef enum
{ // Enum for the robot position wrt to areas
    OUTSIDE = 0,
    INSIDE = 1,
} position_t;

typedef enum
{ // Enum for the robot position wrt to areas
    LEFT = 1,
    RIGHT = 2,
} Free_space;

motion_t current_motion_type = STOP; // Current motion type

action_t current_state = RANDOM_WALKING; // Current state

/* SALAH---------------------------------------------- */
// uint32_t last_turn_ticks = 0;                   // Counters for motion, turning and random_walk
// uint32_t turn_ticks = 60;
// unsigned int turning_ticks = 0;
// const uint8_t max_turning_ticks = 160;          // Constant to allow a maximum rotation of 180 degrees with \omega=\pi/5
// const uint16_t max_straight_ticks = 320;        // Set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32)
// uint32_t last_motion_ticks = 0;
// uint32_t turn_into_random_walker_ticks = 160;   // Timestep to wait without any direction message before turning into random_walker
// uint32_t last_direction_msg = 0;
/* LUIGI---------------------------------------------- */
const float std_motion_steps = 20 * 16; // variance of the gaussian used to compute forward motion
const float levy_exponent = 2;          // 2 is brownian like motion (alpha)
const float crw_exponent = 0.4;         // higher more straight (rho)
uint32_t turning_ticks = 0;             // keep count of ticks of turning
const uint8_t max_turning_ticks = 120;  /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0;        // keep count of ticks of going straight
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
int sa_type = 0;
int sa_payload = 0;
bool new_sa_msg = false;

/*informations received from ARK*/
int imposed_direction = 0; //which direction to point at
int current_kb_angle = 0;  //current orientation of the robot

int straight_timer;                 //time of straight walk toward target
int turn_timer;                     //time of turning toward target
double directed_motion_freq = 0.01; //frequency of motion toward target

/* PARAMETER: change this value to determine timeout lenght */
int TIMEOUT_CONST = 500;

//bool conditioned = false;

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
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
    // index of first element in the 3 sub-blocks of data
    uint8_t shift = kb_index * 3;

    sa_type = data[shift + 1] >> 2 & 0x0F;
    sa_payload = ((data[shift + 1] & 0b11) << 8) | (data[shift + 2]);
    new_sa_msg = true;

    imposed_direction = sa_type;

    if (sa_type != 3)
    {
        current_kb_angle = sa_payload;
    }

    else
    {
        // avoid colliding with the wall
        proximity_sensor = sa_payload;
        wall_avoidance_start = true;
    }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d)
{

    /* Unpack the message - extract ID, type and payload */
    if (msg->type == 0)
    {
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);

        if (id1 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 0);
        }
        if (id2 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 1);
        }
        if (id3 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 2);
        }
    }

#ifndef ARGOS_SIMULATION
    /* For ID identification */
    else if (msg->type == 120)
    {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid)
        {
            set_color(RGB(0, 0, 3));
        }
        else
        {
            set_color(RGB(0, 3, 0));
        }
    }
#endif
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
}

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
            // set_color(RGB(0,3,0));
            // random rotation strategy
            // if (rand_soft() % 2)
            // {
            //   set_motion(TURN_LEFT);
            // }
            // else
            // {
            //   set_motion(TURN_RIGHT);
            // }
            //rotate towards the last free space kept in memory
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
/* Function implementing kilobot FSM and state transitions           */
/*-------------------------------------------------------------------*/
void finite_state_machine()
{
    /* State transition */
    switch (current_state)
    {
    case RANDOM_WALKING:
    {
        if (imposed_direction != 0)
        {
            printf("\nGIRO: imposed dir=%d   ori=%d\n", imposed_direction, current_kb_angle);
            last_motion_ticks = kilo_ticks;
            // turning_ticks = (uint32_t)((M_PI_2 / M_PI) * max_turning_ticks);
            // straight_ticks = 300;
            if (imposed_direction == TURNING_NORTH)
            {
                set_color(RGB(3, 0, 0));
                current_state = TURNING_NORTH;
                if (current_kb_angle <= 15)
                {
                    turn_timer = (1.33 * (15 - current_kb_angle % 100));
                }
                else if (current_kb_angle > 15 && current_kb_angle < 32)
                {
                    turn_timer = (1.33 * (current_kb_angle % 100 - 15)); //exclude hundred
                }
                else if (current_kb_angle > 100 && current_kb_angle <= 115)
                {
                    turn_timer = (1.33 * (15 + current_kb_angle % 100));
                }
                else if (current_kb_angle > 115 && current_kb_angle < 132)
                {
                    turn_timer = (1.33 * (45 - current_kb_angle % 100));
                }
                printf("angle=%d ---> tmr set to %d\n\n", current_kb_angle, turn_timer);
            }
            else if (imposed_direction == TURNING_SOUTH)
            {
                set_color(RGB(0, 3, 0));
                current_state = TURNING_SOUTH;
                if (current_kb_angle <= 15)
                {
                    turn_timer = (1.33 * (current_kb_angle % 100 + 15));
                }
                else if (current_kb_angle > 15 && current_kb_angle < 32)
                {
                    turn_timer = (1.33 * (45 - current_kb_angle % 100)); //exclude hundred////////////////////////
                }
                else if (current_kb_angle > 100 && current_kb_angle <= 115)
                {
                    turn_timer = (1.33 * (15 - current_kb_angle % 100));
                }
                else if (current_kb_angle > 115 && current_kb_angle < 132)
                {
                    turn_timer = (1.33 * (current_kb_angle % 100 - 15));
                }
            }

            // else if (imposed_direction == 3)
            // {
            //     printf("kID=%d \n", kilo_uid);
            //     set_color(RGB(0, 0, 3));
            //     delay(500);
            //     set_color(RGB(3, 0, 3));
            //     delay(500);
            //     set_color(RGB(0, 3, 3));
            //     delay(500);
            //     set_color(RGB(3, 3, 3));
            //     set_motion(STOP);
            // }
        }
        break;
    }

    case TURNING_NORTH:
    {
        //printf("\nNORD timer:%d",turn_timer);
        if (turn_timer <= 0)
        {
            current_state = MOVING_TO_TARGET;
            straight_timer = 100;
            set_color(RGB(3, 3, 3));
        }
        else
        {
            if (current_kb_angle <= 15 || (current_kb_angle > 100 && current_kb_angle <= 115))
            {
                set_motion(TURN_LEFT);
            }
            else
            {
                set_motion(TURN_RIGHT);
            }
            turn_timer--;
        }
        break;
    }
    case TURNING_SOUTH:
    {
        if (turn_timer <= 0)
        {
            current_state = MOVING_TO_TARGET;
            straight_timer = 100;
            set_color(RGB(3, 3, 3));
        }
        else
        {
            if (current_kb_angle <= 15 || (current_kb_angle > 100 && current_kb_angle <= 115))
            {
                set_motion(TURN_RIGHT);
            }
            else
            {
                set_motion(TURN_LEFT);
            }
            turn_timer--;
        }
        break;
    }

    case MOVING_TO_TARGET:
    {
        set_motion(FORWARD);
        if (straight_timer <= 0)
        {
            set_color(RGB(0, 0, 0));
            current_state = RANDOM_WALKING;
            imposed_direction = 0;
        }
        else
        {
            straight_timer--;
        }
    }
    }
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
        // set_color(RGB(0,3,0));
        random_walk();
        finite_state_machine();
    }
}

int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);
    return 0;
}