#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

const int LIFT_FACTOR = 10.8;

///
// Constants
///
bool Lift_stop = false;
int Lift_height1 = 0;

int error = 0;
int integral_e = 0;
int prev_error = 0;

int KP = 2;
int KD = 0.2;
int KI = 0;

int speed_max_lift = 60;

int derivative = 0;
void Lift_Shift(){
  while(!Lift_stop && !driver) {
    int error = Lift_height1 * 18.4+10 - Lift.get_position();
    float output = (error * KP + derivative * KD + integral_e * KI);

    derivative = error - prev_error;
    if (output > speed_max_lift){
      output = speed_max_lift;
    }
    if (output < -speed_max_lift){
      output = -speed_max_lift;
    }

    prev_error = error;
    integral_e += error;
    Lift.move(output);
  }
}

void Lift_Shift_upgraded(){
  while(!Lift_stop && !driver) {
    int error = Lift_height1 * /*change it to smth*/18.4 + 10 - Lift_Rotation.get_position();
    float output = (error * KP + derivative * KD + integral_e * KI);

    derivative = error - prev_error;
    if (output > speed_max_lift){
      output = speed_max_lift;
    }
    if (output < -speed_max_lift){
      output = -speed_max_lift;
    }

    prev_error = error;
    integral_e += error;
    Lift.move(output);
  }
}


bool auto_claping = false;
void auto_clap(){
  if (!auto_claping){
    Clamp.set(false);
  }
  while(auto_claping){
    if (Clamp_Sensor.get_distance() < 35){
      Clamp.set(true);
    }
  }
}

void default_constants() {
  chassis.pid_drive_constants_set(22.5, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(4.2, 0.05, 26, 10);// 3.2 0 22 12.0
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  // large distance turn pid: chassis.pid_turn_constants_set(4.3, 0, 37, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(2.4,0.0,34);
  chassis.pid_odom_boomerang_constants_set(1.5, 0.0, 35);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_turn_constants_set(6_deg, 40);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.3);

  chassis.odom_look_ahead_set(10_in);
  chassis.odom_boomerang_distance_set(16_in); // Max distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.65); // How aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);
  chassis.drive_imu_scaler_set(1);
}

///
// Drive Example
///
void PID_Test () {
  Lift_height1 = 50;
  speed_max_lift = 100;
  pros::Task Lift_Shifting(Lift_Shift);
  pros::delay(3000);
  Lift_height1 = 0;
}

void pin2(){
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // flip the toggle
  Clamp.set(true);
  Lift.move(100);
    chassis.pid_drive_exit_condition_set(10_ms, 1_in, 10_ms, 3_in, 50_ms, 150_ms);
    chassis.pid_drive_set(-1, 127, true);
  pros::delay(300);
  Lift.move(-127);
  pros::delay(500);
  Lift.move(-10);
 
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  // score the first pin
  chassis.pid_drive_set(10.3, 127, true);//10.3
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); // change to smth lower if needed 

  chassis.pid_drive_set(27, 80, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_drive_set(-0.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  Clamp.set(false);

  // grab first stack
  pros::Task Lifting(Lift_Shift);
    Lift_height1 = 50;
    speed_max_lift = 100;
  pros::delay(500);
  chassis.pid_drive_set(-4, 100, true);
  chassis.pid_wait_quick_chain();
    Lift_height1 = 8;
    speed_max_lift = 50;
  chassis.pid_turn_set(-135,80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(17, 40, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(ez::RIGHT_SWING, 180, 100, 10);
  chassis.pid_wait_quick_chain();

  Clamp.set(true);
  pros::delay(400);
  Lift_height1 = 40;
  speed_max_lift = 100;
  chassis.pid_turn_set(-8, 100);
  chassis.pid_wait();
  pros::delay(500);

  
  chassis.pid_drive_set(5, 100, true);
  chassis.pid_wait_quick_chain();


  Lift_height1 = 0;
  speed_max_lift = 50;
  


}
void left_side () {

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  Lift.move(60);
  pros::delay(400);
  Lift.brake();

  chassis.pid_drive_set(4.8, 80, true);
  chassis.pid_wait_quick_chain();

  Lift.move(-60);
  pros::delay(500);
  Lift.brake();
  Clamp.set(true);
  pros::delay(600);

  chassis.pid_drive_set(-1, 80, true);
  chassis.pid_wait_quick_chain();
    Lift.move(-60);

  chassis.pid_turn_set(-100, 80, true);
  pros::delay(200);
  Lift.brake();
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(4, 50, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(ez::RIGHT_SWING, -140, 60, 0);
  chassis.pid_wait_quick_chain();

  Clamp.set(false);
  pros::delay(300);
  Lift.move(60);

  chassis.pid_turn_set(17, 80, true);
  pros::delay(150);
  Lift.brake();
  chassis.pid_wait();

  Lift.move(80);
  pros::delay(500);
  Lift.brake();

  chassis.pid_drive_set(9, 80, true);
  chassis.pid_wait_quick_chain();

  Lift.move(-60);
  pros::delay(400);
  Lift.brake();

  Clamp.set(true);
  pros::delay(200);

  Lift.move(60);

  chassis.pid_turn_set(-40, 80, true);
  pros::delay(200);
  Lift.brake();
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-10, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(35, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-15, 80, true);
  chassis.pid_wait_quick_chain();

  Lift.move(-80);
  pros::delay(500);
  Lift.brake();

  chassis.pid_drive_set(25, 80, true);
  pros::delay(200);
  Lift.move(-10);
  chassis.pid_wait_quick_chain();

  Lift.brake();


}

void left_side_5_pins() {

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // flip the toggle
  Clamp.set(true);
  Lift.move(100);
    chassis.pid_drive_exit_condition_set(10_ms, 1_in, 10_ms, 3_in, 50_ms, 150_ms);
    chassis.pid_drive_set(-1, 127, true);
  pros::delay(300);
  Lift.move(-127);
  pros::delay(350);
  Lift.move(-10);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
    
  // score the first pin
  chassis.pid_drive_set(9.7, 127, true);//10.3
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); // change to smth lower if needed 
  chassis.pid_drive_set(27.5, 90, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_drive_set(-0.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  Clamp.set(false);

  // grab first stack
  
  chassis.pid_drive_set(-4.2, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(31, 100, true);
    pros::Task Lifting(Lift_Shift);
    Lift_height1 = 10;
    speed_max_lift = 50;
  chassis.pid_wait_quick_chain();


  //split it into two separate movements

  chassis.pid_drive_set(18.5, 100, true);
  // pros::delay(900);
  // Clamp.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6, 60, true);
  pros::delay(170);//300
  Clamp.set(true);
  chassis.pid_wait_quick_chain();



  // score the first stack
  
  speed_max_lift = 75;
  Lift_height1  = 40;

  chassis.pid_turn_set(173, 100, true);//-169
  chassis.pid_wait_quick_chain();



  chassis.pid_drive_set(20, 100, true);//18
  pros::delay(100);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 40;
  Lift_height1  = 0;
  pros::delay(600);
  Clamp.set(0);
  pros::delay(300);

  chassis.pid_drive_set(-6, 100, true);
  pros::delay(250);
  Lift.move(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(119, 100, true);//-118
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20.3, 100, true);
  speed_max_lift = 100;
  Lift_height1  = -5;
  chassis.pid_wait_quick_chain();

  
  chassis.pid_drive_set(5, 60, true);
  pros::delay(170);//850
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 70;
  Lift_height1  = 55;

  chassis.pid_turn_set(-101, 100, true);//98
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(18, 70, true);//18

  chassis.pid_wait_quick_chain();

  speed_max_lift = 30;
  Lift_height1  = 0;
  pros::delay(300);//300
  Clamp.set(0);
  pros::delay(200);//300

  chassis.pid_drive_set(-5, 100, true);//-5
  chassis.pid_wait_quick_chain();

  speed_max_lift = 70;
  Lift_height1  = -10;

  chassis.pid_turn_set(-148, 100, true); //148
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.5, 100, true);
  // Lift.move(60);
  // pros::delay(180);
  // Lift.brake();
  chassis.pid_wait_quick_chain();

  
  chassis.pid_swing_set(ez::LEFT_SWING, -170, 100, 0);
  pros::delay(300);
  chassis.pid_swing_set(ez::LEFT_SWING, -170, 127, -10);
  pros::delay(400);
  Clamp.set(true);
  speed_max_lift = 127;
  Lift_height1 = 80;
  pros::delay(700);


  // chassis.pid_turn_set(0, 100, true); //Maybe change it to a swing
  // pros::delay(400);
  // Lift.brake();
  // chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 100, -70 );
  Lift.brake();
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(8, 60, true);
  pros::delay(200);
  speed_max_lift = 40;
  Lift_height1 = 0;

  // chassis.pid_wait_quick_chain();

  pros::delay(500);
  Clamp.set(0);
  pros::delay(300);


}


void right_side() {

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // flip the toggle
  Clamp.set(true);
  Lift.move(100);
    chassis.pid_drive_exit_condition_set(10_ms, 1_in, 10_ms, 3_in, 50_ms, 150_ms);
    chassis.pid_drive_set(-1, 127, true);
  pros::delay(300);
  Lift.move(-127);
  pros::delay(500);
  Lift.move(-10);
 
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  // score the first pin
  chassis.pid_drive_set(10.3, 127, true);//10.3
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); // change to smth lower if needed 

  chassis.pid_drive_set(27, 80, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_drive_set(-0.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  Clamp.set(false);

  // grab first stack
  
  chassis.pid_drive_set(-6, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-33, 100, true);
     pros::Task Lifting(Lift_Shift);
    Lift_height1 = 8;
    speed_max_lift = 50;
  chassis.pid_wait_quick_chain();

  //split it into two separate movements

  chassis.pid_drive_set(18.5, 100, true);
  // pros::delay(900);
  // Clamp.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.5, 60, true);
  pros::delay(200);//300
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  // score the first stack
  
  speed_max_lift = 70;
  Lift_height1  = 35;

  chassis.pid_turn_set(-165, 100, true);//-169
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19, 100, true);//18
  pros::delay(100);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 40;
  Lift_height1  = 0;
  pros::delay(600);
  Clamp.set(0);
  pros::delay(300);

  chassis.pid_drive_set(-6.6, 100, true);
  pros::delay(250);
  Lift.move(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-119, 100, true);//-118
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23, 80, true);
  speed_max_lift = 100;
  Lift_height1  = 0;
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6, 60, true);
  pros::delay(150);//850
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 70;
  Lift_height1  = 55;

  chassis.pid_turn_set(104, 100, true);//98
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(18, 70, true);//18

  chassis.pid_wait_quick_chain();

  speed_max_lift = 30;
  Lift_height1  = 0;
  pros::delay(500);//300
  Clamp.set(0);
  pros::delay(200);//300



  }

void right_side_alt() {

  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // flip the toggle
  Clamp.set(true);
  Lift.move(100);
    chassis.pid_drive_exit_condition_set(10_ms, 1_in, 10_ms, 3_in, 50_ms, 150_ms);
    chassis.pid_drive_set(-1, 127, true);
  pros::delay(300);
  Lift.move(-127);
  pros::delay(500);
  Lift.move(-10);
 
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  // score the first pin
  chassis.pid_drive_set(10.3, 127, true);//10.3
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); // change to smth lower if needed 

  chassis.pid_drive_set(27, 80, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_drive_set(-0.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  Clamp.set(false);

  // grab first stack
  
  chassis.pid_drive_set(-6, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-33, 100, true);
     pros::Task Lifting(Lift_Shift);
    Lift_height1 = 8;
    speed_max_lift = 50;
  chassis.pid_wait_quick_chain();

  //split it into two separate movements

  chassis.pid_drive_set(18.5, 100, true);
  // pros::delay(900);
  // Clamp.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.5, 60, true);
  pros::delay(200);//300
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  // score the first stack
  
  speed_max_lift = 70;
  Lift_height1  = 35;

  chassis.pid_turn_set(-165, 100, true);//-169
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19, 100, true);//18
  pros::delay(100);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 40;
  Lift_height1  = 0;
  pros::delay(600);
  Clamp.set(0);
  pros::delay(300);

  chassis.pid_drive_set(-6.6, 100, true);
  pros::delay(250);
  Lift.move(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-119, 100, true);//-118
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23, 80, true);
  speed_max_lift = 100;
  Lift_height1  = 0;
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6, 60, true);
  pros::delay(150);//850
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 70;
  Lift_height1  = 55;

  // chassis.pid_turn_set(0, 100, true);//98
  // chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(-19, 70, true);//18
  chassis.pid_wait_quick_chain();


  chassis.pid_turn_set(90, 100, true);//98
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(34, 70, true);//18
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(135, 100, true);//98
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21, 70, true);//18
  chassis.pid_wait_quick_chain();

  speed_max_lift = 30;
  Lift_height1  = 0;
  pros::delay(1000);//300
  Clamp.set(0);
  pros::delay(200);//300



  }


void skills () {
chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // flip the toggle
  Clamp.set(true);
  Lift.move(100);
    chassis.pid_drive_exit_condition_set(10_ms, 1_in, 10_ms, 3_in, 50_ms, 150_ms);
    chassis.pid_drive_set(-1, 127, true);
  pros::delay(300);
  Lift.move(-127);
  pros::delay(500);
  Lift.move(-10);
 
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  // score the first pin
  chassis.pid_drive_set(10.3, 127, true);//10.3
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms); // change to smth lower if needed 

  chassis.pid_drive_set(27, 80, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_drive_set(-0.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  Clamp.set(false);

  // grab first stack
  
  chassis.pid_drive_set(-6, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-33, 100, true);
     pros::Task Lifting(Lift_Shift);
    Lift_height1 = 8;
    speed_max_lift = 50;
  chassis.pid_wait_quick_chain();

  //split it into two separate movements

  chassis.pid_drive_set(18.5, 100, true);
  // pros::delay(900);
  // Clamp.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.5, 60, true);
  pros::delay(200);//300
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  // score the first stack
  
  speed_max_lift = 70;
  Lift_height1  = 35;

  chassis.pid_turn_set(-165, 100, true);//-169
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19, 100, true);//18
  pros::delay(100);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 40;
  Lift_height1  = 0;
  pros::delay(600);
  Clamp.set(0);
  pros::delay(300);

  chassis.pid_drive_set(-6.6, 100, true);
  pros::delay(250);
  Lift.move(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-119, 100, true);//-118
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23, 80, true);
  speed_max_lift = 100;
  Lift_height1  = 0;
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6, 60, true);
  pros::delay(150);//850
  Clamp.set(true);
  chassis.pid_wait_quick_chain();

  speed_max_lift = 70;
  Lift_height1  = 55;

  chassis.pid_turn_set(104, 100, true);//98
  chassis.pid_wait_quick_chain();


  chassis.pid_drive_set(18, 70, true);//18

  chassis.pid_wait_quick_chain();

  speed_max_lift = 30;
  Lift_height1  = 0;
  pros::delay(500);//300
  Clamp.set(0);
  pros::delay(350);//300

  chassis.pid_drive_set(-8, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(45, 80, true);
  chassis.pid_wait_quick_chain();

}

void drive_example() {






}

// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .