#include <kilolib.h>

/* Enum for boolean flags */
typedef enum {
    false = 0,
    true = 1,
} bool;

message_t message;

/* Flag for decision to send a word */
bool sending_msg = false;

/*Flag for the existence of information*/
bool new_message = false;

int num_message_sent = 0;
int num_message_received = 0;


/* counters for broadcast a message */
const uint8_t max_broadcast_ticks = 2 * 16; /* n_s*0.5*32 */
uint32_t last_broadcast_ticks = 0;
const uint16_t max_info_ticks = 7 * 16;
uint32_t last_info_ticks = 0;

uint16_t kilo_other_id;

uint32_t last_ticks = 0;


void setup()
{
  last_ticks = kilo_ticks;
  /* Initialise LED and motors */
  set_motors(0, 0);
  /* Initialise random seed */
  uint8_t seed = rand_hard();
  rand_seed(seed);
  srand(seed);

  /* Initialise KBots message */
  message.type = NORMAL;  
  message.data[0] = kilo_uid;
  message.crc = message_crc(&message);

}

void loop()
{
  // Blink the LED magenta whenever a message is sent.
  if (sending_msg && kilo_uid == 0)
  {
    // Reset the flag so the LED is only blinked once per message.
    sending_msg = false;

    set_color(RGB(1, 0, 1));
    delay(100);
    set_color(RGB(0, 0, 0));

    // uint32_t frequency_ticks = kilo_ticks-last_ticks;
    // printf("frequency_ticks: %lu \n", frequency_ticks);
    //printf("kid: %lu \n", kilo_uid);
    // printf("last_ticks: %lu \n", last_ticks);
    // printf("kilo_ticks: %lu \n", kilo_ticks);
    last_ticks = kilo_ticks;
  }

  if(new_message && kilo_uid != 0)
  {
    new_message = false;
    set_color(RGB(1, 1, 0));
    delay(100);
    set_color(RGB(0, 0, 0));
  }
}

/*-------------------------------------------------------------------*/
/* Send current kb status to the swarm                               */
/*-------------------------------------------------------------------*/
message_t *message_tx() 
{
  return &message;
}

void message_tx_success()
{
   // Set the flag on message transmission.
   num_message_sent +=1;
   sending_msg = true;
   printf("kid %lu, num_message_sent %d\n", kilo_uid, num_message_sent);
}



void message_rx(message_t *msg, distance_measurement_t *d) {
  num_message_received +=1;
  printf("kid %lu, num_message_received %d\n", kilo_uid, num_message_received);
  new_message = true;
}

int main()
{
   kilo_init();
   // register message reception callback
   kilo_message_rx = message_rx;
   // Register the message_tx callback function.
   kilo_message_tx = message_tx;
   // Register the message_tx_success callback function.
   kilo_message_tx_success = message_tx_success;
   kilo_start(setup, loop);

   return 0;
} 