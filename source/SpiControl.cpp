#include "MicroBit.h"
#include "BirdBrain.h"
#include "SpiControl.h"
#include "Finch.h"

SPI spi(MOSI, MISO, SCK);
bool spiActive; // ensures we do not accidentally interleave SPI commands called from different fibers

// Initializing SPI, putting the SS pin high
void spiInit()
{
    uBit.io.P16.setDigitalValue(1);
    spi.format(8,0);
    spi.frequency(1000000);
    fiber_sleep(10);
    spiActive = false;
}

bool aquireSPI() {
    // Wait up to 5 ms for another SPI command to complete
    uint8_t timeOut = 0;
    while(spiActive && timeOut < 5) {
        fiber_sleep(1);
        timeOut++;
    }
    if (spiActive)
        return false;
    spiActive = true;
    return true;
}

bool spiWrite(uint8_t* writeBuffer, uint8_t length)
{
  if (!aquireSPI()) return false;

  uBit.io.P16.setDigitalValue(0);
  for(int i = 0; i < length; i++)
    spi.write(writeBuffer[i]);
  uBit.io.P16.setDigitalValue(1);
  NRFX_DELAY_US(50); // Ensures we don't hammer the Finch or Hummingbird with SPI packets

  spiActive = false;
  return true;
}

bool spiReadFinch(uint8_t (&readBuffer)[FINCH_SPI_SENSOR_LENGTH])
{
    if (!aquireSPI()) return false;

    uBit.io.P16.setDigitalValue(0);
    //NRFX_DELAY_US(SS_WAIT);
    readBuffer[0] = spi.write(0xDE);
    //NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    for(int i = 1; i < FINCH_SPI_SENSOR_LENGTH; i++)
    {
        readBuffer[i] = spi.write(0xFF);
    //    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    //NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);

    spiActive = false;

    return true;
}

uint8_t readFirmwareVersion()
{
    if (!aquireSPI()) return UNIDENTIFIED_DEV;

    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(SS_WAIT);
    uint8_t readBuffer[4];
    readBuffer[0] = spi.write(0x8C); // Special command to read firmware/hardware version for both Finch and HB
    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    for(int i = 1; i < 3; i++)
    {
      readBuffer[i] = spi.write(0xFF);
      NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    readBuffer[3] = spi.write(0xFF);
    NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);
    NRFX_DELAY_MS(1); // wait after reading firmware

    spiActive = false;

    if(readBuffer[0] == FINCH_SAMD_ID)
      return FINCH_SAMD_ID;
    else if((readBuffer[3] == HUMMINGBIT_SAMD_ID) || (readBuffer[3] == (HUMMINGBIT_SAMD_ID-1)) || (readBuffer[3] == (HUMMINGBIT_SAMD_ID-2)))
      return HUMMINGBIT_SAMD_ID;
    else if((readBuffer[0] + readBuffer[1] + readBuffer[2] + readBuffer[3]) == 0) // Bit hokey, but if all bytes are 0, it's a micro:bit since SPI isn't responding
      return MICROBIT_SAMD_ID;
    else   
      return UNIDENTIFIED_DEV; // can be any number that isn't the FINCH and HUMMINGBIT IDs 
}

