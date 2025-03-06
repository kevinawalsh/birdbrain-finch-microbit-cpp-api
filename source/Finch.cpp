#include "MicroBit.h"
#include "BirdBrain.h"
#include "SpiControl.h"
#include "Finch.h"

MicroBit uBit;

static uint32_t prevLeftCounterValue = 0;
static uint32_t prevRightCounterValue = 0;
// static bool leftMotorMove = false;
static bool leftMotorForwardDirection = false;
// static bool rightMotorMove = false;
static bool rightMotorForwardDirection = false;
// static bool updatingSensors = true;
static NRF52ADCChannel *mic = NULL; // Used to increase the gain of the mic ADC channel
// static bool spiLocked = false;
static int16_t micSamplesLo, micSamplesHi;
static struct finch_sensors sensor_vals;
static uint8_t led_state[16] = { 0 }; // (r,g,b) x (beak, tail 1,2,3,4)

void Finch_resetEncoders()
{
    sensor_vals.wheel_encoder_left = 0;
    sensor_vals.wheel_encoder_right = 0;
}

void setAllLEDs()
{
    led_state[0] = 0xD0;   // SetAll LEDs Command
    spiWrite(led_state, 16);
}

void Finch_setBeak(int red, int green, int blue)
{
    led_state[1] = red;
    led_state[2] = green;
    led_state[3] = blue;
    setAllLEDs();
}

void Finch_setTail(int led_num, int red, int green, int blue)
{
    if (led_num < 1 || led_num > 4)
        microbit_panic(990);
    led_state[1+3*led_num] = red;
    led_state[2+3*led_num] = green;
    led_state[3+3*led_num] = blue;
    setAllLEDs();
}

void Finch_setTailAll(int red, int green, int blue)
{
    led_state[4] = led_state[7] = led_state[10] = led_state[13] = red;
    led_state[5] = led_state[8] = led_state[11] = led_state[14] = green;
    led_state[6] = led_state[9] = led_state[12] = led_state[15] = blue;
    setAllLEDs();
}

int Finch_getEncoder(char side)
{
    if (side == 'L')
        return sensor_vals.wheel_encoder_left;
    else if (side == 'R')
        return sensor_vals.wheel_encoder_right;
    else
        microbit_panic(991);
}

int Finch_getDistance()
{
    return sensor_vals.distance;
}

int Finch_getLight(char side)
{
    if (side == 'L')
        return sensor_vals.light_left;
    else if (side == 'R')
        return sensor_vals.light_right;
    else
        microbit_panic(992);
}

int Finch_getLine(char side)
{
    if (side == 'L')
        return sensor_vals.line_left;
    else if (side == 'R')
        return sensor_vals.line_right;
    else
        microbit_panic(993);
}

//  The micro:bit is mounted at a 40 degree angle. These equations convert the
//  accelerometer readings to the reference frame of the finch:
//  X_finch = x_micro:bit
//  Y_finch = y_micro:bit*cos(40) - z_micro:bit*sin(40)
//  Z_finch = y_micro:bit*sin(40) + z_micro:bit*cos(40)
//  We use approximations:
//    cos(40) = 23/30
//    sin(40) = 9/14
//  When multiplied and rounded, these are exact on all but a few values between
//  -127 and +127, with a maximum error of 1 unit of accurate.

vector3d Finch_getAcceleration()
{
    vector3d v;
    v.x = 2.0/127.0 * sensor_vals.ax;
    v.y = 2.0/127.0 * ((int)sensor_vals.ay)*23/30 - ((int)sensor_vals.az)*9/14;
    v.z = 2.0/127.0 * ((int)sensor_vals.ay)*9/14 + ((int)sensor_vals.az)*23/30;
    return v;
}

// todo:
vector3d Finch_getMagnetometer();
int Finch_getCompass();

char Finch_getOrientation()
{
    if (sensor_vals.shaking)  return '~'; // shaking
    if (sensor_vals.ax >  50) return 'L'; // tilted left
    if (sensor_vals.ax < -50) return 'R'; // tilted right
    int adjusted_ay = ((int)sensor_vals.ay)*23/30 - ((int)sensor_vals.az)*9/14;
    if (adjusted_ay    >  50) return 'U'; // beak up
    if (adjusted_ay    < -50) return 'D'; // beak down
    int adjusted_az = ((int)sensor_vals.ay)*9/14 + ((int)sensor_vals.az)*23/30;
    if (adjusted_az    >  50) return '^'; // upside down
    if (adjusted_az    < -50) return '-'; // level
    return '?';
}

bool Finch_getOrientationBoolean(char orientation)
{
    if (orientation == '~') return (sensor_vals.shaking);  // shaking
    if (orientation == 'L') return (sensor_vals.ax >  50); // tilted left
    if (orientation == 'R') return (sensor_vals.ax < -50); // tilted right
    int adjusted_ay = ((int)sensor_vals.ay)*23/30 - ((int)sensor_vals.az)*9/14;
    if (orientation == 'U') return (adjusted_ay    >  50); // beak up
    if (orientation == 'D') return (adjusted_ay    < -50); // beak down
    int adjusted_az = ((int)sensor_vals.ay)*9/14 + ((int)sensor_vals.az)*23/30;
    if (orientation == '^') return (adjusted_az    >  50); // upside down
    if (orientation == '-') return (adjusted_az    < -50); // level
    microbit_panic(982);
}

bool Finch_getButton(char button)
{
    if (button == 'A')
        return sensor_vals.btn_a;
    else if (button == 'B')
        return sensor_vals.btn_b;
    else if (button == 'T')
        return sensor_vals.btn_touch;
    else
        microbit_panic(994);
}

int Finch_getSound()
{
    return sensor_vals.loudness;
}

int Finch_getTemperature()
{
    return sensor_vals.temperature;
}

bool Finch_isShaking()
{
    return sensor_vals.shaking;
}

int clamp(int val, int low, int high)
{
  if (val < low) return low;
  else if (val > high) return high;
  else return val;
}

float clamp(float val, float low, float high)
{
  if (val < low) return low;
  else if (val > high) return high;
  else return val;
}

float midi_note_period[] = {
    28861.83731,  /* midi note:25  freq:34.64782887 */ 
    27241.94685,  /* midi note:26  freq:36.70809599 */ 
    25712.97386,  /* midi note:27  freq:38.89087297 */ 
    24269.81553,  /* midi note:28  freq:41.20344461 */ 
    22907.65545,  /* midi note:29  freq:43.65352893 */ 
    21621.94755,  /* midi note:30  freq:46.24930284 */ 
    20408.40088,  /* midi note:31  freq:48.99942950 */ 
    19262.96535,  /* midi note:32  freq:51.91308720 */ 
    18181.81818,  /* midi note:33  freq:55.00000000 */ 
    17161.35114,  /* midi note:34  freq:58.27047019 */ 
    16198.15851,  /* midi note:35  freq:61.73541266 */ 
    15289.02573}; /* midi note:36  freq:65.40639133 */

void Finch_playNote(int midi_note, float beats)
{
    int note = clamp(midi_note, 25, 135);
    int beats_ms = (int)(1000 * clamp(beats, 0.0, 16.0));

    int period_us;
    if (note <= 36) period_us = (int)(0.5 + midi_note_period[note - 25]);
    else if (note <= 48) period_us = (int)(0.5 + midi_note_period[note - 37]/2);
    else if (note <= 60) period_us = (int)(0.5 + midi_note_period[note - 49]/4);
    else if (note <= 72) period_us = (int)(0.5 + midi_note_period[note - 61]/8);
    else if (note <= 84) period_us = (int)(0.5 + midi_note_period[note - 73]/16);
    else if (note <= 96) period_us = (int)(0.5 + midi_note_period[note - 85]/32);
    else if (note <= 108) period_us = (int)(0.5 + midi_note_period[note - 97]/64);
    else if (note <= 120) period_us = (int)(0.5 + midi_note_period[note - 109]/128);
    else if (note <= 132) period_us = (int)(0.5 + midi_note_period[note - 121]/256);
    else period_us = (int)(0.5 + midi_note_period[note - 133]/512);

    setBuzzer(period_us, beats_ms);
}

extern char messageFlash[18]; // 18 is the maximum message length
extern uint8_t messageLength; 
extern bool flashOn;  // Allows us to cancel flashing a message if we're asked to print a symbol before we're done
extern bool newFlash; // Flag to show if a new message needs to update the message being flashed
    
MicroBitImage bleImage(5,5);

void Finch_print(ManagedString message)
{
    int msglen = message.length();
    if (msglen > 18)
        msglen = 18;
    uBit.display.clear();
    messageLength = msglen;
    for (int i = 0; i < msglen; i++)
        messageFlash[i] = message.charAt(i);
    if (!flashOn && messageLength > 0) {
        MicroBitEvent evt(BB_ID, FLASH_MSG_EVT); 
    } else {
        newFlash = true; // Set this true if we're overwriting a currently flashing message
    }
}

void Finch_println(ManagedString message)
{
    bool isFlashing = flashOn;
    bool wasFlashing = isFlashing;
    Finch_print(message);
    int tries = 0;
    while (isFlashing || (!wasFlashing && tries < 50)) {
        fiber_sleep(50);
        wasFlashing = isFlashing;
        isFlashing = flashOn;
        if (tries < 50)
            tries++;
    }
}

void setDisplay(uint32_t bitvec)
{
    for (int i = 0; i < 25; i++) {
        bleImage.setPixelValue(i%5, i/5, (bitvec & (0x1000000>>i)) ? 255 : 0);
    }
    flashOn = false;
    uBit.display.clear();
    uBit.display.printAsync(bleImage);
}

void Finch_setDisplay(bool ledValues[])
{
    for (int i = 0; i < 25; i++) {
        bleImage.setPixelValue(i%5, i/5, ledValues[i] ? 255 : 0);
    }
    flashOn = false;
    uBit.display.clear();
    uBit.display.printAsync(bleImage);
}

void Finch_setDisplay(int ledValues[])
{
    for (int i = 0; i < 25; i++) {
        int v = ledValues[i];
        if (v < 0)
            v = 0;
        else if (v > 255)
            v = 255;
        bleImage.setPixelValue(i%5, i/5, v);
    }
    flashOn = false;
    uBit.display.clear();
    uBit.display.printAsync(bleImage);
}

void Finch_setPoint(int row, int column, int value)
{
    if (row < 0 || row > 4 || column < 0 || column > 4)
        microbit_panic(995);
    if (value < 0 || value > 255)
        microbit_panic(996);
    bleImage.setPixelValue(column, row, value);
    flashOn = false;
    uBit.display.clear();
    uBit.display.printAsync(bleImage);
}

void Finch_clearDisplay()
{
  setDisplay(0);
}

void Program_pause(float num_seconds)
{
  int ms = (int)(num_seconds * 1000);
  uBit.sleep(ms);
}

void System_exit(int exitcode)
{
  microbit_panic(exitcode);
}

// here, left_speed and right_speed should be in range -100 .. +100, where
// positive means forward, negative means backwards.
void moveMotor(int left_speed, int32_t lt, int right_speed, int32_t rt, bool wait)
{
    bool isMoving = sensor_vals.moving;
    bool wasMoving = isMoving;

    // Undocumented: In the SPI commands, the high bit of the byte for speed
    // controls the direction, 1 means forward, 0 means backwards. The other
    // seven bits control the speed. In other words, it's 8-bit signed-magnitude
    // with negative meaning forward.
    leftMotorForwardDirection = left_speed >= 0;
    rightMotorForwardDirection = right_speed >= 0;
    uint8_t ls = (left_speed >= 0) ? (0x80 | (uint8_t)left_speed) : (((uint8_t)-left_speed)&0xFF);
    uint8_t rs = (right_speed >= 0) ? (0x80 | (uint8_t)right_speed) : (((uint8_t)-right_speed)&0xFF);

    uint8_t cmdBuf[16];
    cmdBuf[0] = 0xD2;              // Set Motors Command
    cmdBuf[1] = 0xFF;              // unknown
    cmdBuf[2] = ls;                // left speed
    cmdBuf[3] = (lt >> 16) & 0xFF; // left ticks byte 2
    cmdBuf[4] = (lt >>  8) & 0xFF; // left ticks byte 1
    cmdBuf[5] = (lt      ) & 0xFF; // left ticks byte 0
    cmdBuf[6] = rs;                // right speed
    cmdBuf[7] = (rt >> 16) & 0xFF; // right ticks byte 2
    cmdBuf[8] = (rt >>  8) & 0xFF; // right ticks byte 1
    cmdBuf[9] = (rt      ) & 0xFF; // right ticks byte 0
    cmdBuf[10] = 0xFF;             // unused
    cmdBuf[11] = 0xFF;             // unused
    cmdBuf[12] = 0xFF;             // unused
    cmdBuf[13] = 0xFF;             // unused
    cmdBuf[14] = 0xFF;             // unused
    cmdBuf[15] = 0xFF;             // unused
    spiWrite(cmdBuf, 16);

    if (wait) {
        int tries = 0;
        while (isMoving || (!wasMoving && tries < 50)) {
            fiber_sleep(10);
            wasMoving = isMoving;
            isMoving = sensor_vals.moving;
            if (tries < 50)
                tries++;
        }
    }
}

void Finch_move(char direction, float distance_cm, int speed_percent)
{
    if (direction != 'F' && direction != 'B')
        microbit_panic(980);
    float dist = clamp(distance_cm, 0.0, 500.0); // could be higher, but 5 meters matches Java API
    int speed = clamp(speed_percent, 0, 100);

    int ticks = (int)(FINCH_TICKS_PER_CM * dist);

    // ticks=0 in SPI command is interpreted as continuous motion, but we want it to mean stop.
    if (ticks == 0)
        speed = 0; // turn motors off

    if (direction == 'B')
        speed = -speed;
    
    if (ticks < 0) {
        ticks = -ticks;
        speed = -speed;
    }

    moveMotor(speed, ticks, speed, ticks, true);
}

void Finch_turn(char side, int angle_degrees, int speed_percent)
{
    if (side != 'L' && side != 'R')
        microbit_panic(981);

    int angle = clamp(angle_degrees, -360000, 360000); // 1000 full circles, matches Java API
    int speed = clamp(speed_percent, 0, 100);

    int ticks = (FINCH_TICKS_PER_DEGREE * angle);

    // ticks=0 in SPI command is interpreted as continuous motion, but we want it to mean stop.
    if (ticks == 0)
        speed = 0; // turn motors off
    
    if (side == 'L')
        speed = -speed;
    
    if (ticks < 0) {
        ticks = -ticks;
        speed = -speed;
    }

    moveMotor(speed, ticks, -speed, ticks, true);
}

void Finch_motors(int left_speed_percent, int right_speed_percent)
{
    int left = clamp(left_speed_percent, -100, 100);
    int right = clamp(right_speed_percent, -100, 100);

    moveMotor(left, 0, right, 0, false);
}

// bool tryAquireSPILock(uint8_t max_wait_ms)
// {
//     while (1) {
//         if (!spiLocked) {
//             spiLocked = true;
//             return true;
//         }
//         if (!max_wait_ms)
//             return false;
//         fiber_sleep(1);
//         max_wait_ms--;
//     }
// }

// bool releaseSPILock()
// {
//     spiLocked = false;
// }

void computeLoudnessVal()
{
    if ((micSamplesHi - micSamplesLo) > 255)
        sensor_vals.loudness = 255;
    else
        sensor_vals.loudness = micSamplesLo - micSamplesHi;
}

void decodeFinchSensors(uint8_t (&spi_sensors_only)[FINCH_SPI_SENSOR_LENGTH])
{
    // Extract left encoder value from SAMD sensor data
    uint32_t currentLeftCounterValue;
    currentLeftCounterValue  = ((uint32_t)spi_sensors_only[ 9])<<16;
    currentLeftCounterValue |= ((uint32_t)spi_sensors_only[10])<<8;
    currentLeftCounterValue |= ((uint32_t)spi_sensors_only[11]);

    // Extract right encoder value from SAMD sensor data
    uint32_t currentRightCounterValue;
    currentRightCounterValue  = ((uint32_t)spi_sensors_only[12])<<16;
    currentRightCounterValue |= ((uint32_t)spi_sensors_only[13])<<8;
    currentRightCounterValue |= ((uint32_t)spi_sensors_only[14]);

    // Accumulate encoder changes
    if (true /*leftMotorMove*/) {
        int32_t delta = currentLeftCounterValue - prevLeftCounterValue;
        if (leftMotorForwardDirection == true )
            sensor_vals.wheel_encoder_left += delta;
        else
            sensor_vals.wheel_encoder_left -= delta;
    }
    prevLeftCounterValue = currentLeftCounterValue;

    if (true /*rightMotorMove*/) {
        int32_t delta = currentRightCounterValue - prevRightCounterValue;
        if(rightMotorForwardDirection == true )
            sensor_vals.wheel_encoder_right += delta;
        else
            sensor_vals.wheel_encoder_right -= delta;
    }
    prevRightCounterValue = currentRightCounterValue;

    sensor_vals.distance = ((((uint16_t)spi_sensors_only[2])<<8)&0xff00)|(spi_sensors_only[3]&0xff);
    sensor_vals.light_left = spi_sensors_only[4];
    sensor_vals.light_right = spi_sensors_only[5];
    sensor_vals.line_left = spi_sensors_only[6];
    sensor_vals.line_right = spi_sensors_only[7];
    sensor_vals.battery = spi_sensors_only[8];
}

void assembleSensorData()
{
    // if (!tryAquireSPILock(5))
    //     return;

    uint8_t spi_sensors_only[FINCH_SPI_SENSOR_LENGTH]; // 16 bytes

    bool success = spiReadFinch(spi_sensors_only);

    // Catch if our SPI sensor packet failed or got interrupted by inbound BLE messages during read
    while(!success || spi_sensors_only[2] == 0x2C || spi_sensors_only[2] == 0xFF)
    {
        fiber_sleep(1);
        success = spiReadFinch(spi_sensors_only);
    }

    decodeFinchSensors(spi_sensors_only); // distance, light_*, line_*, battery, wheel_encoder_*
    if (sensor_vals.line_left & 0x80) { // semi-undocumented
        sensor_vals.line_left &= ~0x80;
        sensor_vals.moving = true;
    } else {
        sensor_vals.moving = false;
    }

    getAccelerometerValsFinch(&sensor_vals); // ax, ay, az, shaking
    getMagnetometerValsFinch(&sensor_vals); // mx, my, mz
    getButtonValsFinch(&sensor_vals); // btn_a, btn_b, btn_touch

    // // Probably not necessary as we get feedback from the LED screen
    // if(calibrationAttempt)
    // {
    //     if(calibrationSuccess)
    //     {
    //         sensor_vals[16] = sensor_vals[16] | 0x04;
    //     }
    //     else
    //     {
    //         sensor_vals[16] = sensor_vals[16] | 0x08;
    //     }
    // }

    sensor_vals.temperature = uBit.thermometer.getTemperature();

    // releaseSPILock();
}

// Updates sensor data approx every 30-40 ms (varies a bit on what else is going on).
// Also reads the microphone once per 4 ms.
void update_sensors()
{
    uint8_t loopCount = 0;
    while (true /*updatingSensors*/) {
        // Read the microphone sound value
        if (loopCount == 0) {
          micSamplesLo = micSamplesHi = uBit.io.microphone.getAnalogValue();
        } else {
          int16_t sample = uBit.io.microphone.getAnalogValue();
          if (sample < micSamplesLo)
            micSamplesLo = sample;
          if (sample > micSamplesHi)
            micSamplesHi = sample;
        }
        fiber_sleep(1); // this appears to actually take 4 ms in our testing
        loopCount++;
        if (loopCount >= MIC_SAMPLES) {
            loopCount = 0;
            computeLoudnessVal();
            assembleSensorData();
        }
    }
    release_fiber();
}

#define HAPPY_EMOJI 0x0A045C0
#define WINK_EMOJI  0x08045C0
#define SAD_EMOJI   0x0A001D1

// See if the SAMD processor has booted and is responding. If not, the finch is
// probably turned off.
bool checkForFinchPower()
{
  readFirmwareVersion(); // must be done twice, first try returns garbage (?!?)
  uint8_t ver = readFirmwareVersion();
  // Finch_println((int)ver);
  if (ver == FINCH_SAMD_ID) {
    setDisplay(HAPPY_EMOJI);
    fiber_sleep(300);
    setDisplay(WINK_EMOJI);
    fiber_sleep(100);
    setDisplay(HAPPY_EMOJI);
    fiber_sleep(300);
    return true;
  } else {
    setDisplay(SAD_EMOJI);
    fiber_sleep(1500);
    Finch_println("Turn finch on!!!");
    setDisplay(SAD_EMOJI);
    return false;
  }
}

void Finch_stop()
{
    Finch_move('F', 0, 0); // stop all motors
}

void Finch_stopAll()
{
    Finch_move('F', 0, 0); // stop all motors
    memset(led_state, 0, sizeof(led_state));
    setAllLEDs(); // clear all LEDs
    flashOn = false;
    uBit.display.clear();
    setDisplay(0);
}

int main()
{

    uBit.init(); // Initializes everything but SPI

    // a little animation
    uBit.display.disable();
    uBit.io.row1.setDigitalValue(1);
    uBit.io.col1.setDigitalValue(0);

    spiInit();   // Turn on SPI

    // Set the buzzer pin low so we don't accidentally energize the Finch or HB buzzer
    uBit.io.P0.setDigitalValue(0);

    // Toggle the reset pin on the Finch, then hold it low
    // This happens even for HB and standalone micro:bit, as it needs to happen before we can determine device type
    uBit.io.pin[RESET_PIN].setDigitalValue(1);
    fiber_sleep(200);
    uBit.io.pin[RESET_PIN].setDigitalValue(0);

    // Wait for the SAMD bootloader checker
    // fiber_sleep(1850);
    fiber_sleep(185);
    uBit.io.row2.setDigitalValue(1);
    uBit.io.col2.setDigitalValue(0);
    fiber_sleep(185);
    uBit.io.row3.setDigitalValue(1);
    uBit.io.col3.setDigitalValue(0);
    fiber_sleep(185);
    uBit.io.row4.setDigitalValue(1);
    uBit.io.col4.setDigitalValue(0);
    fiber_sleep(185);
    uBit.io.row5.setDigitalValue(1);
    uBit.io.col5.setDigitalValue(0);
    fiber_sleep(185);
    uBit.io.row1.setDigitalValue(0); fiber_sleep(93);
    uBit.io.col5.setDigitalValue(1); fiber_sleep(93);
    uBit.io.row5.setDigitalValue(0); fiber_sleep(93);
    uBit.io.col1.setDigitalValue(1); fiber_sleep(93);
    uBit.io.row2.setDigitalValue(0); fiber_sleep(93);
    uBit.io.col4.setDigitalValue(1); fiber_sleep(93);
    uBit.io.row4.setDigitalValue(0); fiber_sleep(93);
    uBit.io.col2.setDigitalValue(1); fiber_sleep(93);
    uBit.io.row3.setDigitalValue(0); fiber_sleep(93);
    uBit.io.col3.setDigitalValue(1); fiber_sleep(93);

    // Setting up an event listener for flashing messages and for running the buzzer
    BBMicroBitInit();

    // uBit.io.row1.setDigitalValue(0);
    // uBit.io.row2.setDigitalValue(0);
    // uBit.io.row3.setDigitalValue(0);
    // uBit.io.row4.setDigitalValue(0);
    // uBit.io.row5.setDigitalValue(0);
    // uBit.io.col1.setDigitalValue(1);
    // uBit.io.col2.setDigitalValue(1);
    // uBit.io.col3.setDigitalValue(1);
    // uBit.io.col4.setDigitalValue(1);
    // uBit.io.col5.setDigitalValue(1);
    uBit.display.enable();

    if (!checkForFinchPower()) {
      release_fiber();
      return 0;
    }

    // Increase the gain of the microphone ADC
    mic = uBit.adc.getChannel(uBit.io.microphone);
    mic->setGain(7,0);
    // Power up the microphone
    uBit.io.runmic.setDigitalValue(1);
    uBit.io.runmic.setHighDrive(true);

    Finch_stopAll();

    // updatingSensors = true;
    create_fiber(update_sensors);
    create_fiber(robot_main);
    release_fiber();
}
