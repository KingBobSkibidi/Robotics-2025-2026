#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::Motor intake;
extern pros::Motor intake_stage2;

// deloader (pneumatics)
void deload_set(bool raised);
void deload_toggle();
bool deload_is_raised();

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
