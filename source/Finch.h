#ifndef FINCH_H
#define FINCH_H
#include "MicroBit.h"

// direction must be 'F' or 'B'
void Finch_move(char direction, float distance_cm, int speed_percent);

// side must be 'L' or 'R'
void Finch_turn(char side, int angle_degrees, int speed_percent);

void Finch_motors(int left_speed_percent, int right_speed_percent);

void Finch_stop(); // only stops motors
void Finch_stopAll(); // turn off everything

void Finch_resetEncoders();

void Finch_setBeak(int red, int green, int blue);

void Finch_setTail(int led_num, int red, int green, int blue);

void Finch_setTailAll(int red, int green, int blue);

// side must be 'L' or 'R'
int Finch_getEncoder(char side);

int Finch_getDistance();

// side must be 'L' or 'R'
int Finch_getLight(char side);

// side must be 'L' or 'R'
int Finch_getLine(char side);

// returns 'U' for "beak up"
// returns 'D' for "beak down"
// returns 'L' for "tilted left"
// returns 'R' for "tilted right"
// returns '-' for "level"
// returns '^' for "upside down"
// returns '?' for "in between" or "none of the above"
char Finch_getOrientation();

void Finch_print(ManagedString message); // Prints in background, does not block the program
void Finch_println(ManagedString message); // Prints, and blocks program until printing is done
void Finch_setDisplay(bool ledValues[]);
void Finch_setDisplay(int ledValues[]);
void Finch_setPoint(int row, int column, int value);
void Finch_clearDisplay();

void Finch_playNote(int midi_note, float beats);

typedef struct vector3d { float x, y, z; } vector3d;
vector3d Finch_getAcceleration(); // units of G, that is, 1.0 means 1G, standard earth gravity
vector3d Finch_getMagnetometer();
int Finch_getCompass();

// button must be 'A', 'B', or 'T' (for "touch")
bool Finch_getButton(char button);

int Finch_getSound();
int Finch_getTemperature();

// orientation must be 'U' for "held beak up"
// orientation must be 'D' for "held beak down"
// orientation must be 'L' for "held tilted left"
// orientation must be 'R' for "held tilted right"
// orientation must be '-' for "held level"
// orientation must be '^' for "held upside down"
// orientation must be '~' for "shaken"
// orientation must be '?' for "in between" or "none of the above"
bool Finch_getOrientationBoolean(char orientation);
bool Finch_isShaking();

void Program_pause(float num_seconds);
void System_exit(int exitcode);

void robot_main();

#endif // FINCH_H
