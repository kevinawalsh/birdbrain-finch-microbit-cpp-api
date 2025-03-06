#ifndef SPICONTROL_H
#define SPICONTROL_H

#include "BirdBrain.h"
#include "BBMicroBit.h"

#define WAIT_BETWEEN_BYTES 4 // 10 microseconds, maybe this can be reduced further
#define SS_WAIT 4 //How long to wait to put SS line low/high before/after transfer

//To know what device is connected look for these
#define MICROBIT_SAMD_ID       0
#define FINCH_SAMD_ID         44
#define HUMMINGBIT_SAMD_ID     3
#define UNIDENTIFIED_DEV      25

#define FINCH_SPI_SENSOR_LENGTH  16

void spiInit();
bool spiWrite(uint8_t* writeBuffer, uint8_t length);
bool spiReadFinch(uint8_t (&readBuffer)[FINCH_SPI_SENSOR_LENGTH]);
uint8_t readFirmwareVersion();

#endif
