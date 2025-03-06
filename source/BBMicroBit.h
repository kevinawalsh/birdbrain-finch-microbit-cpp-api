#ifndef BBMICROBIT_H
#define BBMICROBIT_H

// from BLESerial.h

#define SYMBOL 									 					0x80
#define SCROLL                    									0x40
#define SAMD_MINIMUM_FIRMWARE_VERSION 						0x01   

//Commands opcode
#define SETALL_SPI                                0xCA
#define SET_LEDARRAY                              0xCC
#define SET_LED_2                                 0xC1
#define SET_LED_3                                 0xC2
#define SET_BUZZER                                0xCD
#define SET_CALIBRATE                             0xCE
#define SET_FIRMWARE                              0xCF
#define STOP_ALL                                  0xCB
#define NOTIFICATIONS                             0x62
#define START_NOTIFY                              0x67
#define START_NOTIFYV2                            0x70
#define STOP_NOTIFY                               0x73

#define FINCH_SETALL_LED			              0xD0
#define FINCH_SETALL_MOTORS_MLED				  0xD2
#define FINCH_SET_FIRMWARE                        0xD4
#define FINCH_RESET_ENCODERS                      0xD5
#define FINCH_POWEROFF_SAMD                       0xD6
#define FINCH_STOPALL                             0xDF

#define BROADCAST                                 'b'
#define MICRO_IO                                  0x90

#define LENGTH_SETALL_SPI                         13
#define LENGTH_OTHER_SPI                          4
#define LENGTH_ALL_SPI                            15

#define HARDWARE_VERSION				          0x01

#define SENSOR_SEND_LENGTH                	      14
#define V2_SENSOR_SEND_LENGTH             	      16
#define FINCH_SENSOR_SEND_LENGTH                  20
#define BLE__MAX_PACKET_LENGTH                    20 

#define MIC_SAMPLES                               8

#define FULL_BATT                                 100                          //All four tail LEDS are green above this
#define BATT_THRESH1                              55							//Three tail LEDS are green above this
#define BATT_THRESH2                              40							//Two Tail LEDS are yellow above this

#define FINCH_SPI_LENGTH                          (16)

// arranges the encoder data and other data from SPI to prepare it to send over BLE
struct finch_sensors {
    uint8_t loudness; // loudness of microphone: difference between the min and max of 8 recent samples
    uint16_t distance; // ultrasound distance sensor
    uint8_t light_left, light_right; // light sensors
    uint8_t line_left, line_right;
    uint8_t battery;
    uint8_t temperature;
    bool moving; // position control flag
    int32_t wheel_encoder_left, wheel_encoder_right;
    int8_t ax, ay, az; // accelerometer, range is +/- 2g, thus units are 196/1280 m/s^2
    int8_t mx, my, mz; // magnetometer
    bool btn_touch, btn_a, btn_b; // buttons
    bool shaking;
};

#define FINCH_TICKS_PER_CM          (49.7)
#define FINCH_TICKS_PER_DEGREE      (4.335)

void BBMicroBitInit();

// Get and convert the accelerometer values to 8 bit format, and check if the shake bit should be set
void getAccelerometerVals(struct finch_sensors *sensor_vals);

// Get and convert the magnetometer values to a 16 bit format
void getMagnetometerVals(struct finch_sensors *sensor_vals);

// Get the state of the buttons
void getButtonVals(struct finch_sensors *sensor_vals);

// Get and convert the accelerometer values to 8 bit format, and check if the shake bit should be set
void getAccelerometerValsFinch(struct finch_sensors *sensor_vals);

// Get and convert the magnetometer values to a 16 bit format
void getMagnetometerValsFinch(struct finch_sensors *sensor_vals);

// Get the state of the buttons
void getButtonValsFinch(struct finch_sensors *sensor_vals);

// Function used for setting the buzzer for Hummingbird and Finch
void setBuzzer(uint16_t period, uint16_t duration);

#endif
