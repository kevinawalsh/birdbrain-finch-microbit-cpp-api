#include "MicroBit.h"
#include "BirdBrain.h"
#include "BBMicroBit.h"

char messageFlash[18]; // 18 is the maximum message length
uint8_t messageLength; 
bool flashOn;  // Allows us to cancel flashing a message if we're asked to print a symbol before we're done
bool newFlash; // Flag to show if a new message needs to update the message being flashed

uint16_t buzzPeriod;
uint16_t buzzDuration;
bool newBuzz; // Flag to show if a new buzzer value needs to update the buzzer
bool buzzerRunning; // Flag to capture if the buzzer event is currently running 

// helper function to convert the accelerometer value from milli-gs to an 8-bit val
uint8_t convertAccelVal(int accelerometerValue);
// helper function to convert from uT to a 16-bit unsigned val
int8_t convertMagVal(int magValue);

// Flash the message 
void flashMessage(MicroBitEvent)
{
    uint8_t i = 0;
    uint8_t state = 0;
    flashOn = true;
    // Checks if a different message or symbol has come by to override the current message
    // Checks every 200 ms - this prints the symbol for 400 ms and clears the screen for 200 ms
    while(flashOn && (i < messageLength))
    {
        if(state == 0)
        {
            uBit.display.printAsync(messageFlash[i]);
        }
        if(state == 2)
        {
            uBit.display.clear(); // clear the display to provide a "flash"
            i++;
            state = -1; // reset our state counter (effectively to 0 since we increment it next)
        }
        state++;
        fiber_sleep(200);

        // Check for a new message to flash and reset the loop if that's the case
        if(newFlash)
        {
            i = 0;
            state = 0;
            newFlash = false;
        }

    }
    // No longer flashing the message
    flashOn = false;
}

// Run the buzzer
void mbBuzz(MicroBitEvent)
{
    uint16_t elapsed = 0;
    uint16_t localDuration = 0;
    localDuration = buzzDuration;
    buzzerRunning = true;
    // Make a sound if the tone is below 20 KHz and longer than 10 ms
    if(buzzPeriod > 50 && localDuration > 10) {
        while(elapsed < (localDuration/4) && buzzerRunning)
        {
            // If we've updated the buzzer while it's playing, update to the new values
            if(newBuzz && buzzDuration > 0) {
                uBit.io.P0.setAnalogValue(512);
                uBit.io.P0.setAnalogPeriodUs(buzzPeriod);    
                uBit.io.speaker.setAnalogValue(0);                
                elapsed = 0; //resetting elapsed time
                localDuration = buzzDuration; //resetting the buzz duration
                newBuzz = false; //making sure we don't update again until a real new value comes in over BLE
            }
            // Check every 4 milliseconds
            fiber_sleep(4); 
            elapsed++;
        }
        uBit.io.P0.setAnalogValue(0);
    }
    // Reset the flags
    newBuzz = false;     
    buzzerRunning = false;
}

void BBMicroBitInit()
{
    // Set up a listener for flashing messages    
    uBit.messageBus.listen(BB_ID, FLASH_MSG_EVT, flashMessage);
    // Set one up for the buzzer
    uBit.messageBus.listen(BB_ID, MB_BUZZ_EVT, mbBuzz);

    flashOn = false;
    newFlash = false;
    buzzerRunning = false;
    messageLength = 0; 
    buzzPeriod = 0;
    buzzDuration = 0;
    uBit.io.speaker.setAnalogValue(0);
}

// Function used for setting the buzzer for Hummingbird and Finch
void setBuzzer(uint16_t period, uint16_t duration)
{
    // only activate the buzzer if the command is valid
    if(period > 0 && duration > 10)
    {
        buzzPeriod = period;
        buzzDuration = duration;
        if(buzzDuration > 0) {
            newBuzz = true; //Turning this on to update the loop in case the event is already active
        }
        if(!buzzerRunning) {
            // Launching the event if it's not already running
            MicroBitEvent evt(BB_ID, MB_BUZZ_EVT);
        } 
    }
}

// Get and convert the accelerometer values to 8 bit format, and check if the shake bit should be set
void getAccelerometerValsFinch(struct finch_sensors *sensor_vals)
{
    // Inverting the sign of X and Y to match how the V1 works
    sensor_vals->ax = 255-convertAccelVal(uBit.accelerometer.getX());
    sensor_vals->ay = 255-convertAccelVal(uBit.accelerometer.getY());
    sensor_vals->az = convertAccelVal(uBit.accelerometer.getZ());

    sensor_vals->shaking = (uBit.accelerometer.getGesture() == ACCELEROMETER_EVT_SHAKE);
}

// Get and convert the magnetometer values to an 8 bit Finch format - this function is probably wrong right now
void getMagnetometerValsFinch(struct finch_sensors *sensor_vals)
{
    sensor_vals->mx = convertMagVal(uBit.compass.getX());
    sensor_vals->my = convertMagVal(uBit.compass.getY());
    sensor_vals->mz = convertMagVal(uBit.compass.getZ());
}

// Get the state of the buttons
void getButtonValsFinch(struct finch_sensors *sensor_vals)
{
    sensor_vals->btn_a = uBit.buttonA.isPressed();
    sensor_vals->btn_b = uBit.buttonB.isPressed();
    sensor_vals->btn_touch = uBit.logo.isPressed();
}


uint8_t convertAccelVal(int accelerometerValue)
{
    uint8_t convertedAccelVal;
    
    if(accelerometerValue > 2000)
        accelerometerValue = 2000;
    if(accelerometerValue < -2000)
        accelerometerValue = -2000;
    if(accelerometerValue >= 0)
    {
        convertedAccelVal = (accelerometerValue * 127)/2000; 
    }
    else {
        convertedAccelVal = 255+(accelerometerValue * 127)/2000; 
    }
    return convertedAccelVal;
}

int8_t convertMagVal(int magValue)
{
    uint16_t convertedMagVal;

    // scaling to provide similar values to micro:bit V1
    magValue = magValue/100;
    if(magValue > 30000)
        magValue = 30000;
    if(magValue < -30000)
        magValue = -30000;
    if(magValue >= 0)
    {
        convertedMagVal = (magValue * 32767)/30000; 
    }
    else {
        convertedMagVal = 65535+(magValue * 32767)/30000; 
    }
    int16_t signedVal = (int16_t)convertedMagVal;
    signedVal = -signedVal/10;
    if (signedVal > 127)
        signedVal = 127;
    else if (signedVal < -127)
        signedVal = -127;
    return (int8_t)(signedVal & 0xff);
}

