#include <AccelStepper.h>

// Convert degrees of joint position (for single motor for now) to step of that stepper
// Assume 0 steps = 0 degrees
// Assume size of array
void ConvertDegToStep(long deg_positions[5], long step_positons[5]) {
  // calculate motor internal gear ratio
  long step_position1 = 95.0/33.0*deg_positions[0];
  long step_position2 = 95.0/33.0*deg_positions[1];
  long step_position3 = 95.0/33.0*deg_positions[2];
  long step_position4 = 95.0/33.0*deg_positions[3];
  long step_position5 = 95.0/9.0*deg_positions[4];

  // take into consideration microstepping
  const float microstepping_factor = 4.0; // value is x if microstepping by 1/x
  step_position1 *= microstepping_factor;
  step_position2 *= microstepping_factor;
  step_position3 *= microstepping_factor;
  step_position4 *= microstepping_factor;
  step_position5 *= microstepping_factor;

  // take into consideration external gear ratio
  // stepper 1 no gear 
  step_positons[0] = step_position1;
  // stepper 2 2:1 reduction
  step_positons[1] = 2*step_position2;
  // stepper 3 no gear
  step_positons[2] = step_position3;
  // stepper 4 2:1 reduction
  step_positons[3] = 2*step_position4;
  // stepper 5 no gear
  step_positons[4] = step_position5;
}

// convert rpm to step/s (velocity)  
void ConvertRPMToStepPerSecond(float rpm, float steps_per_second[5]) {
  steps_per_second[0] = rpm*6.0*4*95.0/33.0; // in steps per second units, make sure not to forget gear ratio
  steps_per_second[1] = rpm*6.0*4*95.0/33.0*2.0;
  steps_per_second[2] = rpm*6.0*4*95.0/33.0;
  steps_per_second[3] = rpm*6.0*4*95.0/33.0*2.0;
  steps_per_second[4] = rpm*6.0*4*95.0/9.0; 
}

// convert rpm/s to step/s^2 (acceleration)  
void ConvertRPMSToStepPerSecond2(float rpms, float steps_per_second_2[5]) {
  steps_per_second_2[0] = rpms*6.0*4*95.0/33.0; // in steps per second units, make sure not to forget gear ratio
  steps_per_second_2[1] = rpms*6.0*4*95.0/33.0*2.0;
  steps_per_second_2[2] = rpms*6.0*4*95.0/33.0;
  steps_per_second_2[3] = rpms*6.0*4*95.0/33.0*2.0;
  steps_per_second_2[4] = rpms*6.0*4*95.0/9.0; 
}

void FillPositionArray(long positions[5], float position1, float position2, float position3, float position4, float position5) {
  positions[0] = position1;
  positions[1] = position2;
  positions[2] = position3;
  positions[3] = position4;
  positions[4] = position5;
}

void FillPositionArrayFromTo(long positions_to_copy_from[5], long positions_to_copy_to[5]) {
  positions_to_copy_to[0] = positions_to_copy_from[0];
  positions_to_copy_to[1] = positions_to_copy_from[1];
  positions_to_copy_to[2] = positions_to_copy_from[2];
  positions_to_copy_to[3] = positions_to_copy_from[3];
  positions_to_copy_to[4] = positions_to_copy_from[4];
}

// ensure positions are legal
// limits here are extremely conservative
bool ValidJointPositionsDeg(long deg_positions[5]) {
  if (deg_positions[0] < -90 || deg_positions[0] > 90) {
    Serial.println("Error: Joint limit 1 exceeded");
    return false;
  }

  if (deg_positions[1] < -95 || deg_positions[1] > 15) {
    Serial.println("Error: Joint limit 2 exceeded");
    return false;
  }

  if (deg_positions[2] < -145 || deg_positions[2] > 145) {
    Serial.println("Error: Joint limit 3 exceeded");
    return false;
  }

  if (deg_positions[3] < -135 || deg_positions[3] > 135) {
    Serial.println("Error: Joint limit 4 exceeded");
    return false;
  }

  if (deg_positions[4] < -180 || deg_positions[4] > 180) {
    Serial.println("Error: Joint limit 5 exceeded");
    return false;
  }
  
  return true;
}

// steppers for each joint 1-6
// arguments: type of motor, step pin, direction pin
AccelStepper stepper1(AccelStepper::DRIVER, 3, 4);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 7);
AccelStepper stepper3(AccelStepper::DRIVER, 9, 10);
AccelStepper stepper4(AccelStepper::DRIVER, 25, 26);
AccelStepper stepper5(AccelStepper::DRIVER, 28, 29);

// Ensure that positions are calibrated before any movement
bool calibrated_positions;

// Joint velocity and acceleration limits for all motors
const float motor_rpm_limit = 10;
const float motor_rpms_limit = 10;

// Current position
long current_step_positions[5];

// Calibration
void CalibrationProcess() {
  // move end effector to prevent strain
  long deg_positions[5]; // Array of desired stepper position in deg
  
  deg_positions[0] = 0.0;
  deg_positions[1] = -43.3937198;
  deg_positions[2] = 0.0;
  deg_positions[3] = 0.0;
  deg_positions[4] = 0.0;

  GoToJointDegAnglesWithAccel(deg_positions, 10, 5);
  delay(500);

  Serial.println("Press c to continue sequence");
  char c = 'd';
  while (c != 'c') {
    if (Serial.available()) {
      c = Serial.read();
      Serial.print("Read ");
      Serial.println(c);  
    }
  }
  Serial.println("Continuing");

  deg_positions[0] = 0.0;
  deg_positions[1] = -43.3937198;
  deg_positions[2] = 124.323738;
  deg_positions[3] = -80.9300183;
  deg_positions[4] = 0.0;

  GoToJointDegAnglesWithAccel(deg_positions, 10, 5);
  
  delay(500);

  // set this as zero position
  current_step_positions[0] = 0;
  current_step_positions[1] = 0;
  current_step_positions[2] = 0;
  current_step_positions[3] = 0;
  current_step_positions[4] = 0;
  
  calibrated_positions = true;
}

// Sets velocity and acceleration based on desired end position with constraint of arriving at the same time
// Uses max velocity as max velocity for all motors
// Calculates the time taken by the slowest motor, then adjusts the max acceleration for other motors to arrive at the same time
// Assumes current_step_positions is correct
// Units of 
void SetMotorVelAccel(long target_step_positions[5], float max_vel_steps_per_second[5], float max_accel_steps_per_second_2[5]) {
  // Configure each stepper's max speed
  stepper1.setMaxSpeed(max_vel_steps_per_second[0]);
  stepper2.setMaxSpeed(max_vel_steps_per_second[1]);
  stepper3.setMaxSpeed(max_vel_steps_per_second[2]);
  stepper4.setMaxSpeed(max_vel_steps_per_second[3]);
  stepper5.setMaxSpeed(max_vel_steps_per_second[4]); 

  // Calculate which takes the longest time
  float delta_step_1 = abs(current_step_positions[0] - target_step_positions[0]);
  float delta_step_2 = abs(current_step_positions[1] - target_step_positions[1]);
  float delta_step_3 = abs(current_step_positions[2] - target_step_positions[2]);
  float delta_step_4 = abs(current_step_positions[3] - target_step_positions[3]);
  float delta_step_5 = abs(current_step_positions[4] - target_step_positions[4]);

  float max_time = 0;
  float time_1 = (delta_step_1/max_vel_steps_per_second[0]) + (max_vel_steps_per_second[0]/max_accel_steps_per_second_2[0]);
  float time_2 = (delta_step_2/max_vel_steps_per_second[1]) + (max_vel_steps_per_second[1]/max_accel_steps_per_second_2[1]);
  max_time = max(time_1, time_2);
  float time_3 = (delta_step_3/max_vel_steps_per_second[2]) + (max_vel_steps_per_second[2]/max_accel_steps_per_second_2[2]);
  max_time = max(max_time, time_3);
  float time_4 = (delta_step_4/max_vel_steps_per_second[3]) + (max_vel_steps_per_second[3]/max_accel_steps_per_second_2[3]);
  max_time = max(max_time, time_4);
  float time_5 = (delta_step_5/max_vel_steps_per_second[4]) + (max_vel_steps_per_second[4]/max_accel_steps_per_second_2[4]);
  max_time = max(max_time, time_5);

  // calculate each stepper's accel based on time and velocity
  float stepper_1_accel = (max_vel_steps_per_second[0] * max_vel_steps_per_second[0]) / (time_1*max_vel_steps_per_second[0] - delta_step_1);
  float stepper_2_accel = (max_vel_steps_per_second[1] * max_vel_steps_per_second[1]) / (time_2*max_vel_steps_per_second[1] - delta_step_2);
  float stepper_3_accel = (max_vel_steps_per_second[2] * max_vel_steps_per_second[2]) / (time_3*max_vel_steps_per_second[2] - delta_step_3);
  float stepper_4_accel = (max_vel_steps_per_second[3] * max_vel_steps_per_second[3]) / (time_4*max_vel_steps_per_second[3] - delta_step_4);
  float stepper_5_accel = (max_vel_steps_per_second[4] * max_vel_steps_per_second[4]) / (time_5*max_vel_steps_per_second[4] - delta_step_5);
  
  // verify limits are respected
  if (stepper_1_accel > max_accel_steps_per_second_2[0] || stepper_2_accel > max_accel_steps_per_second_2[1] || stepper_3_accel > max_accel_steps_per_second_2[2] || stepper_4_accel > max_accel_steps_per_second_2[3] || stepper_5_accel > max_accel_steps_per_second_2[4]) {
    Serial.println("Error acceleration exceeded");
    return;
  }

  Serial.println("Times:\n");
  Serial.println((delta_step_1/max_vel_steps_per_second[0]) + (max_vel_steps_per_second[0]/stepper_1_accel));
  Serial.println((delta_step_2/max_vel_steps_per_second[1]) + (max_vel_steps_per_second[1]/stepper_2_accel));
  Serial.println((delta_step_3/max_vel_steps_per_second[2]) + (max_vel_steps_per_second[2]/stepper_3_accel));
  Serial.println((delta_step_4/max_vel_steps_per_second[3]) + (max_vel_steps_per_second[3]/stepper_4_accel));
  Serial.println((delta_step_5/max_vel_steps_per_second[4]) + (max_vel_steps_per_second[4]/stepper_5_accel));

  Serial.println("Accelerations:\n");
  Serial.println(stepper_1_accel);
  Serial.println(stepper_2_accel);
  Serial.println(stepper_3_accel);
  Serial.println(stepper_4_accel);
  Serial.println(stepper_5_accel);

  stepper1.setAcceleration(stepper_1_accel); 
  stepper2.setAcceleration(stepper_2_accel);
  stepper3.setAcceleration(stepper_3_accel);
  stepper4.setAcceleration(stepper_4_accel);
  stepper5.setAcceleration(stepper_5_accel);
}

// Go to joint position in degrees with maximum acceleration/deceleration and velocity in units of rpm and rpm/s
// Have all joints arrive at the same time
// blocks
// Assumes current_step_positions is correct then updates current_step_positions as necessary
// returns true if successful and false otherwise
bool GoToJointDegAnglesWithAccel(long target_deg_positions[5], float max_vel_rpm, float max_accel_rpms) {
  if (max_vel_rpm > motor_rpm_limit || max_accel_rpms > motor_rpms_limit) {
    Serial.println("Error: motor velocity and/or acceleration limits reached");
    return false;
  }

  // convert deg positions array to step and fill into step positions array
  long target_step_positions[5];
  ConvertDegToStep(target_deg_positions, target_step_positions);

  // convert velocity rpm to steps per sec
  float max_vel_steps_per_second[5];
  ConvertRPMToStepPerSecond(max_vel_rpm, max_vel_steps_per_second);
  
  // convert acceleration rpms to steps per sec^2
  float max_accel_steps_per_second_2[5];
  ConvertRPMSToStepPerSecond2(max_accel_rpms, max_accel_steps_per_second_2);
  
  // Set correct velocity and acceleration limits based on desired position
  SetMotorVelAccel(target_deg_positions, max_vel_steps_per_second, max_accel_steps_per_second_2);

  // Set target positions
  stepper1.moveTo(target_step_positions[0]);
  stepper2.moveTo(target_step_positions[1]);
  stepper3.moveTo(target_step_positions[2]);
  stepper4.moveTo(target_step_positions[3]);
  stepper5.moveTo(target_step_positions[4]);
  
  // run each motor to position with set vel and accel until finished
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0 || stepper5.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
  }

  // update current step position
  FillPositionArrayFromTo(target_step_positions, current_step_positions);

  // success
  return true;
}

// Demo 
void DemoSequence() {
  // note start position is:
  //  deg_positions[0] = 0.0;
  //  deg_positions[1] = -43.3937198;
  //  deg_positions[2] = 124.323738;
  //  deg_positions[3] = -80.9300183;
  //  deg_positions[4] = 0.0;
  
  long deg_positions[5]; // Array of desired stepper position in deg

  FillPositionArray(deg_positions, 0.0, -73.2472242, 121.615309, -48.3680854, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 2);
  delay(1000);

  FillPositionArray(deg_positions, 53.0687190, -77.1839579, 127.330756, -50.1467985, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 2);
  delay(1000);

  FillPositionArray(deg_positions, 0.0, -43.3937198, 124.323738, -80.9300183, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 2);
  delay(1000);

  FillPositionArray(deg_positions, -44.2170829, -69.8404933, 116.365655, -46.5251624, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 4);
  delay(1000);
  
  FillPositionArray(deg_positions, -44.2170829, -42.3832982, 118.930764, -76.5474666, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 4);
  delay(1000);

  FillPositionArray(deg_positions, 0.0, -73.2472242, 121.615309, -48.3680854, 0.0);
  GoToJointDegAnglesWithAccel(deg_positions, 10, 4);
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  while(!Serial); // wait for Arduino Serial Monitor

  // Configure each stepper
  // convert rpm to step/s (velocity)  
  stepper1.setMaxSpeed(motor_rpm_limit*6.0*95.0/33.0); // in steps per second units, make sure not to forget gear ratio
  stepper2.setMaxSpeed(motor_rpm_limit*6.0*95.0/33.0*2.0);
  stepper3.setMaxSpeed(motor_rpm_limit*6.0*95.0/33.0);
  stepper4.setMaxSpeed(motor_rpm_limit*6.0*95.0/33.0*2.0);
  stepper5.setMaxSpeed(motor_rpm_limit*6.0*95.0/9.0); 

  // convert rpm/s to step/s^2 (acceleration)
  stepper1.setAcceleration(motor_rpms_limit*6.0*95.0/33.0); // in steps per second units, make sure not to forget gear ratio
  stepper2.setAcceleration(motor_rpms_limit*6.0*95.0/33.0*2.0);
  stepper3.setAcceleration(motor_rpms_limit*6.0*95.0/33.0);
  stepper4.setAcceleration(motor_rpms_limit*6.0*95.0/33.0*2.0);
  stepper5.setAcceleration(motor_rpms_limit*6.0*95.0/9.0); 

  // reverse directions to match screw axes, if needed
  stepper1.setPinsInverted(true, false, false);
  stepper2.setPinsInverted(true, false, false);
  stepper4.setPinsInverted(true, false, false);
  stepper5.setPinsInverted(true, false, false);

  // set enable pins to turn on drivers
  stepper1.setEnablePin(2);
  stepper2.setEnablePin(5);
  stepper3.setEnablePin(8);
  stepper4.setEnablePin(24);
  stepper5.setEnablePin(27);
  
  calibrated_positions = false;

  Serial.println("Completed settring up");

  Serial.println("c to calibrate angles");
  Serial.println("m to move to specific angles");
  Serial.println("d to demo");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Read ");
    Serial.println(c);

    if (c == 'c') {
      CalibrationProcess();
    } else if (c == 'm') {
      if (!calibrated_positions) {
        Serial.println("Calibrate first");
        return;
      }

      Serial.println("Moving");
    } else if (c == 'd') {
      if (!calibrated_positions) {
        Serial.println("Calibrate first");
        return;
      }

      Serial.println("Demo");
      DemoSequence();
    }
  }
}
