#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"


extern Drive chassis;

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor L1(-3);
inline pros::Motor L2(-4);
inline pros::Motor L3(-5);


inline pros::Motor R1(6);
inline pros::Motor R2(8);
inline pros::Motor R3(9);

inline pros::MotorGroup Lift({-10, 1});
inline pros::Motor Lift10(-10);
inline pros::Motor Lift1(1);

inline pros::Motor Intake(-2);

inline pros::Imu Inertial(11);

inline pros::Distance Distance_Back(3);
inline pros::Distance Distance_Match_Lift(21);
inline pros::Distance Distance_Front(99);
inline pros::Distance Clamp_Sensor(18);

inline pros::AIVision Vision(20);

inline pros::Rotation Lift_Rotation(-19);

inline ez::Piston Clamp('H');
inline ez::Piston Blooper('B'); 
inline ez::Piston Clamp_Indexer('A');