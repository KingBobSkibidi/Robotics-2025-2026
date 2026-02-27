#include "main.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127.
// Conservative anti-tip speeds for a light robot.
const int DRIVE_SPEED = 80; // full speed is 127
const int FAST_DRIVE_SPEED = 110;
const int TURN_SPEED = 50;
const int FAST_TURN_SPEED = 80;
const int SWING_SPEED = 65;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(15.0, 0.0, 25.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(8.0, 0.0, 35.0);         // IMU heading hold during drives
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(6_deg, 35);
  chassis.slew_drive_constants_set(6_in, 35);
  chassis.slew_swing_constants_set(6_in, 40);

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there 
}

namespace {
void stop_all_motors() {
  chassis.drive_set(0, 0);
  intake.move(0);
  intake_stage2.move(0);
}
}  // namespace

void left_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 40, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn left another 50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 5) Back up 30 in
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void right_start_auton() {
  chassis.drive_imu_reset();

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 40, true, false);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 4) IMU turn right another 50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);
  intake_stage2.move(0);
  intake.move(0);


  // 5) Back up 30 in
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}

void left_start_deload_fast() {
  // 0) set imu and start match with deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes, slowly
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn left another 50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in, slowly
  chassis.pid_drive_set(-20_in, 35, false, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  pros::delay(100);
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 8 inches
  chassis.pid_drive_set(8_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 10) Drive forward 13 inches
  chassis.pid_drive_set(13_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 11) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 11.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 12) Back up 32 in
  chassis.pid_drive_set(-32_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void left_start_deload() {
  // 0) set imu and start match with deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes, slowly
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn left another 50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in, slowly
  chassis.pid_drive_set(-20_in, 35, false, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  pros::delay(100);
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 8 inches
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 10) Drive forward 13 inches
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 11) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // 11.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 12) Back up 32 in
  chassis.pid_drive_set(-32_in, 50, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void right_start_deload_fast() {
  // 0) set imu and start match with deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes, slowly
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn right another 50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in slowly
  chassis.pid_drive_set(-20_in, 35, false, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  pros::delay(100);
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 8 inches
  chassis.pid_drive_set(8_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 10) Drive forward 13 inches
  chassis.pid_drive_set(13_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 11) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, FAST_TURN_SPEED);
  chassis.pid_wait();

  // 11.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 12) Back up 32 in
  chassis.pid_drive_set(-32_in, FAST_DRIVE_SPEED, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void right_start_deload() {
  // 0) set imu and start match with deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes, slowly
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn right another 50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in slowly
  chassis.pid_drive_set(-20_in, 35, false, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  pros::delay(100);
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 8 inches
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 10) Drive forward 13 inches
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 11) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // 11.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 12) Back up 32 in
  chassis.pid_drive_set(-32_in, 50, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void right_start_deload_heavy() {
  // 0) set imu and start match with deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes, slowly
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn right 120 degrees
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(43_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn right another 50 degrees (to 170 total)
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in slowly
  chassis.pid_drive_set(-20_in, 35, false, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  pros::delay(100);
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(2000);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 8 inches
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 10) Drive forward 13 inches
  chassis.pid_drive_set(14_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 11) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // 11.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 12) Back up 32 in
  chassis.pid_drive_set(-32_in, 50, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void left_start_deload_and_matchload() {
  // 0) set imu and start match with matchloader contracted and deloader extended
  chassis.drive_imu_reset();
  pros::delay(100);
  deload.set(PISTON_EXTENDED);
  matchload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 1) Drive forward 35 in while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(35_in, 50, true, false);
  chassis.pid_wait();

  // 2) IMU turn left 120 degrees
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3) Drive forward 42 in
  chassis.pid_drive_set(42_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 4) IMU turn left another 50 degrees (to -170 total)
  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5) Back up 20 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 6) Back up 20 in slowly
  chassis.pid_drive_set(-20_in, 35, true, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 7) Set IMU and run both stages to eject balls from the top
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(1500);
  intake.move(0);
  intake_stage2.move(0);

  // 8) Drive forward 25 in 
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 8.5) extend deloader while stage 1 intakes and stage 2 outtakes
  intake.move(127);
  intake_stage2.move(-127);
  matchload.set(PISTON_EXTENDED);
  pros::delay(200);

  // 9) Drive forward 10 in while stage 1 intakes and stage 2 outtakes
  chassis.pid_drive_set(10_in, 50, true, false);
  chassis.pid_wait();
  pros::delay(600);

  // 9) Back up 10 in
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 9.5) Make sure orientation 0 deg, stop intake and raise matchload
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  matchload.set(PISTON_CONTRACTED);
  intake_stage2.move(0);
  intake.move(0);

  // 10) Back up 15 in
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true, true);
  chassis.pid_wait();
  pros::delay(100);

  // 11) Back up 10 in slowly
  chassis.pid_drive_set(-20_in, 35, true, false);
  chassis.pid_wait();
  intake_stage2.move(0);
  intake.move(0);

  // 12) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  pros::delay(1500);
  intake.move(0);
  intake_stage2.move(0);

  // 13) Drive forward 8 inches
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 14) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 15) Drive forward 13 inches
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();

  // 16) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // 16.5) contract deloader
  deload.set(PISTON_CONTRACTED);
  pros::delay(200);

  // 17) Back up 32 in
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  stop_all_motors();
}

void test_deload() {
  deload.set(PISTON_EXTENDED);
  pros::delay(3000);   // wait 3 seconds
  deload.set(PISTON_CONTRACTED);
}

void temp_skills() {
  intake.move(127);
  intake_stage2.move(-127);
  pros::delay(2000);
  intake_stage2.move(0);
  intake.move(0);
  stop_all_motors();
}

void mid_start_skills() {
  chassis.drive_imu_reset();

  // 1) Drive forward 3 inches
  chassis.pid_drive_set(3_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 2) IMU turn right 90 degrees 
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 3) Drive forward 60 inches
  chassis.pid_drive_set(60_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 6) IMU turn left 90 degrees (180 total degrees)
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 7) Drive forward 23 inches and intake balls
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(23_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(3000);
  intake_stage2.move(0);
  intake.move(0);
  
  // 8) Back up 35 inches
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 9) eject balls
  chassis.drive_imu_reset();
  intake.move(127);
  intake_stage2.move(127);
  chassis.pid_wait();
  pros::delay(2000);
  intake_stage2.move(0);
  intake.move(0);

  // 10) Drive forward 15 inches
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(1000);

  // 11) IMU turn left 90 degrees
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 12) Drive forward 120 inches
  chassis.pid_drive_set(123_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();  
  pros::delay(160);

  // 13) IMU turn right 90 degrees (return 0 deg orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 14) Drive forward 15 inches and intake
  intake.move(127);
  intake_stage2.move(-127);
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(3000);
  intake_stage2.move(0);
  intake.move(0);

  // 15) Back up 35 inches
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 16) Run both stages to eject balls from the top
  intake.move(127);
  intake_stage2.move(127);
  chassis.pid_wait();
  pros::delay(2000);
  intake_stage2.move(0);
  intake.move(0);

  // 17) Drive forward 10 inches
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);
  
  // 18) IMU turn left 90 degrees
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 19) Drive forward 60 inches
  chassis.pid_drive_set(61_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(160);

  // 20) IMU turn right 90 degrees (original orientation)
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(160);

  // 21) Drive 40 inches into parking spot and clear park zone 
  intake.move(127);
  intake_stage2.move(127);
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true, false);
  chassis.pid_wait();
  pros::delay(3000);
  intake.move(0);
  intake_stage2.move(0);
  stop_all_motors();
}
