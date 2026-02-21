#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::Motor intake;
extern pros::Motor intake_stage2;

// pneumatics
constexpr bool PISTON_CONTRACTED = false;
constexpr bool PISTON_EXTENDED = true;
inline ez::Piston matchload('E');
inline ez::Piston deload('D');

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
