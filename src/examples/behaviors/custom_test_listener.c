#include "kilolib.h"

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;

/*Flag for the existence of information*/
bool new_message = false;
int num_message_received = 0;
/* Message send to the other kilobots */

void setup()
{
}

void loop()
{
  if (new_message)
  {
    num_message_received += 1;
    printf("r_kid %lu, num_message_received %d\n", kilo_uid, num_message_received);
    new_message = false;
    set_color(RGB(1, 1, 0));
    delay(100);
    set_color(RGB(0, 0, 0));
  }
  
}


void message_rx(message_t *msg, distance_measurement_t *d) {
  new_message = true;
}

int main()
{
  kilo_init();
  // register message reception callback
  kilo_message_rx = message_rx;
  kilo_start(setup, loop);
    
  return 0;
}
