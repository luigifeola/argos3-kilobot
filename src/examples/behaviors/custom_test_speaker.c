#include <kilolib.h>

message_t message;
// Flag to keep track of message transmission.
int message_sent = 0;
uint32_t last_ticks;

void setup()
{
   last_ticks = kilo_ticks;
   // Initialize message:
   // The type is always NORMAL.
   message.type = NORMAL;
   // Some dummy data as an example.
   message.data[0] = 0;
   // It's important that the CRC is computed after the data has been set;
   // otherwise it would be wrong and the message would be dropped by the
   // receiver.
   message.crc = message_crc(&message);
}

void loop()
{
   // Blink the LED magenta whenever a message is sent.
   if (message_sent == 1)
   {
      // Reset the flag so the LED is only blinked once per message.
      message_sent = 0;
        
      // set_color(RGB(1, 0, 1));
      // delay(100);
      // set_color(RGB(0, 0, 0));

      uint32_t frequency_ticks = kilo_ticks-last_ticks;
      printf("frequency_ticks: %lu \n", frequency_ticks);
      // printf("last_ticks: %lu \n", last_ticks);
      // printf("kilo_ticks: %lu \n", kilo_ticks);
      last_ticks = kilo_ticks;
   }
}

message_t *message_tx()
{
   return &message;
}

void message_tx_success()
{
   // Set the flag on message transmission.
   message_sent = 1;
}

int main()
{
   kilo_init();
   // Register the message_tx callback function.
   kilo_message_tx = message_tx;
   // Register the message_tx_success callback function.
   kilo_message_tx_success = message_tx_success;
   kilo_start(setup, loop);
    
   return 0;
}