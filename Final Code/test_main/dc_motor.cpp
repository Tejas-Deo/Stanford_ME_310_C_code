#include "dc_motor.h"

int ENA = D7;
int IN1 = D1;
int IN2 = D2;
int encoderPinA = D5;
int encoderPinB = D6;

long encoder_value = 0;
long encoder_counts_per_rev = 2443;
int wheel_rev = 2;
int wheel_rev_turn = 1;
int encoder_subtraction_value = 750;     // as i just want to move the walker by 50 cms
long encoderValue = 0;