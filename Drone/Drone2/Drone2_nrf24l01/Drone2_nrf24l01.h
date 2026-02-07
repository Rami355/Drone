#ifndef DRONE2_NRF24L01_H
#define DRONE2_NRF24L01_H

#include <Arduino.h>
#include <RF24.h>

/*************************************************
 * Variables (extern)
 *************************************************/
extern int16_t potFromNano;

/*************************************************
 * Functions
 *************************************************/
void antenna_setup();
void antenna_read();

#endif
