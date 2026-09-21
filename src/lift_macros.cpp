/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * SPDX-License-Identifier: MPL-2.0
 */


#include "main.h"
#include <chrono>
extern pros::MotorGroup Lift;

// Define the states for the lift
enum LiftState {
    MANUAL,
    MACRO_HOMING,
    MACRO_MOVING_TO_TARGET,
    MACRO_SETTING_UP
};

// State and PID Variables (static keeps them persistent but private to this file)
static LiftState currentState = MANUAL;
static double kP = .15;
static double kI = 0.0;
static double kD = 1;
static double slew_lift = 2;
static double prevError = 0;
static double prev_output = 0;
static double lift_integral = 0;
static double targetPosition = 0;

int time_wait;

// Private helper function for PID
static double calculateLiftPID(double currentPosition, double target) {
    double error = target - currentPosition;
    
    if (std::abs(error) < 500) { 
        lift_integral += error; 
    } else {
        lift_integral = 0;
    }
    
    double derivative = error - prevError;
    prevError = error;
    double output = (error * kP) + (lift_integral * kI) + (derivative * kD);

    //Change are these lines. Comment these out if something breaks
    if (output > 0){
        if (output - prev_output > slew_lift){
            output = output + slew_lift;
        }
    }
    prev_output = output;

    return output;

}

namespace LiftControl {

    void initialize() {
        Lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    }

    void update() {
        // 1. Read controller inputs
        bool btnUp = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool btnDown = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool btnMacro1 = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool btnMacro2 = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);

        // 2. Handle State Transitions
        if (btnMacro1 && currentState == MANUAL) {
            currentState = MACRO_HOMING;
        }
        if (btnMacro1 && currentState == MANUAL) {
            currentState = MACRO_SETTING_UP;
        }
        
        if ((btnUp || btnDown) && currentState != MANUAL) {
            currentState = MANUAL;
        }

        // 3. Execute State Logic
       switch (currentState) {
            case MANUAL:
                if (btnUp) {
                    Lift.move(127);
                } else if (btnDown) {
                    Lift.move(-80);
                } else {
                    Lift.move(0); 
                }
                break;

            case MACRO_HOMING:
                Lift.move(-80); 
                
                if (Lift.get_current_draw() > 2200) {
                    Lift.move(0); // Stop the motor immediately so momentum doesn't fight the reset
                    Lift_Rotation.reset_position();
                    
                    // Allow the CAN bus 20ms to process the sensor reset
                    pros::delay(20);
                    
                    targetPosition = 110; 
                    prevError = 0; 
                    lift_integral = 0;
                    
                    currentState = MACRO_MOVING_TO_TARGET;
                }
                break;

            case MACRO_MOVING_TO_TARGET: {
                double currentPos = Lift_Rotation.get_position();
                double power = calculateLiftPID(currentPos, targetPosition);
                
                power = std::clamp(power, -127.0, 127.0);
                Lift.move(power);

                // INCREASED TOLERANCE: Changed from 20 to 100 (1.0 full degree of tolerance)
                // Also ensures the motor is moving slowly (derivative is low) before exiting
                if (std::abs(targetPosition - currentPos) < 5) { 
                    Lift.move(0); 
                    Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    Lift.brake();
                    currentState = MANUAL;

                }
                break;
            }

            // case MACRO_SETTING_UP:
            //     targetPosition = 800;
            //     double currentPos = Lift_Rotation.get_position();
            //     double power = calculateLiftPID(currentPos, targetPosition);
                
            //     power = std::clamp(power, -127.0, 127.0);
            //     Lift.move(power);

            //     // INCREASED TOLERANCE: Changed from 20 to 100 (1.0 full degree of tolerance)
            //     // Also ensures the motor is moving slowly (derivative is low) before exiting
            //     if (std::abs(targetPosition - currentPos) < 5) { 
            //         Lift.move(0); 
            //         Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            //         Lift.brake();
            //         if (!Clamp.get()){
            //             time_wait = pros::millis();
            //         }
            //         Clamp.set(false);
            //         if (pros::millis() - time_wait > 400){
            //             Clamp_Indexer.set(true);
            //             if(pros::millis() - time_wait > 800){
            //                 Clamp.set(true);
            //                 currentState = MANUAL;
            //             }
            //         }
            //     }
            //     break;
        }
    }
}