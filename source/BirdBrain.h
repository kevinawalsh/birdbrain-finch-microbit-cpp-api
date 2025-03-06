#ifndef BIRDBRAIN_H
#define BIRDBRAIN_H

#include "MicroBit.h"
#include "SpiControl.h"
#include "BBMicroBit.h"

#define RESET_PIN      (2) // micro:bit pin to reset/turn off Finch

// Setting up BB specific events
#define BB_ID            (MICROBIT_ID_NOTIFY+1) // last defined eventId is MICROBIT_ID_NOTIFY==1023 in MicroBitComponent.h 
#define FLASH_MSG_EVT    (1)
#define MB_BUZZ_EVT      (2)

extern MicroBit uBit;
extern bool flashOn;

#endif

