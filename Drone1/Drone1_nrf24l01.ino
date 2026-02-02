/*************************************************
 * Includes
 *************************************************/
#include <RF24.h>

/*************************************************
 * Variables
 *************************************************/
const byte address[6] = "00001"; //Adress to the sender.
struct PotPayload {int16_t potA7;};  // Construct the payload comming from the sender.
PotPayload rxPot; // To store the sent value.
int16_t potFromNano = 0;
unsigned long lastRxMs = 0; // Last recived value (for fail safe).

/*************************************************
 * Functions
 *************************************************/
void antenna_setup(); // Radio setup functions.
void antenna_read(); // Radio read with failsafe method.

/*************************************************
 * Hardware Objects
 *************************************************/
RF24 radio(7, 8);  // CE (Chip Enable), CSN (Chip Select Not)


void setup() {
  antenna_setup();
  Serial.begin(500000);
}

void loop() {
  antenna_read();
  Serial.println(potFromNano);
}

/**
 * @brief Reads incoming RF24 payload data and applies a failsafe mechanism.
 *
 * If new data is available in the RF24 receive buffer:
 * 1. Reads the payload into the rxPot structure.
 * 2. Extracts the potentiometer value and save it in potFromNano.
 * 3. Updates the last reception timestamp.
 *
 * If no data has been received within a defined timeout window (200 ms),
 * the potentiometer value is forced to a safe default (0).
 */
void antenna_read() {
  if (radio.available()) {
    radio.read(&rxPot, sizeof(rxPot));
    potFromNano = rxPot.potA7;
    lastRxMs = millis();
  }

  if (millis() - lastRxMs > 200) {
    potFromNano = 0;
  }
}

/**
 * @brief Initializes and configures the RF24 radio module for reception.
 *
 * This function performs all required radio setup steps:
 * 1. Initializes the RF24 driver and SPI interface.
 * 2. Opens a reading pipe using the predefined radio address.
 * 3. Sets the power amplification level to minimum to reduce interference.
 * 4. Switches the radio into listening (RX) mode.
 */
void antenna_setup() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
