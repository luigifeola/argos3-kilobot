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


/*Flag for the existence of information*/
bool new_information = false;
int num_message_received = 0;
/* Message send to the other kilobots */
message_t messageA;

uint32_t last_motion_ticks = 0;
/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;

void setup()
{
  /* Initialise LED and motors */
  set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);
}

void loop()
{
   // Blink the LED yellow whenever a message is received.
   if (new_information)
   {
      // Reset the flag so the LED is only blinked once per message.
      new_information = false;
      num_message_received += 1;  
      set_color(RGB(1, 1, 0));
      delay(100);
      set_color(RGB(0, 0, 0));

      printf("num_message_received: %d \n", num_message_received);
   }
}


void message_rx(message_t *msg, distance_measurement_t *d) {
  
  /* get id (always firt byte) */
  uint8_t id = msg->data[0];

  if (msg->type == NORMAL && id!=kilo_uid) 
  {
    new_information = true;
    set_color(RGB(3, 0, 0));
  }

}

int main()
{
   kilo_init();
    // register message reception callback
    kilo_message_rx = message_rx;
   kilo_start(setup, loop);
    
   return 0;
}
