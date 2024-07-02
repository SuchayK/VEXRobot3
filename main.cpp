#include "vex.h"
using namespace vex;

competition Competition;

double robotX = 0.0; 
double robotY = 0.0; 
double robotTheta = 0.0; 

const double wheelDiameter = 3.0; 
const double tpr = 360; 


void moveForward(double x) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns);
  updateOdometry();
}

void moveForward(double x, bool z) {
  right_drive.spinFor(fwd, x, turns, false);
  left_drive.spinFor(fwd, x, turns, z);
  updateOdometry();
}

void moveBack(double x) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns);
  updateOdometry();
}

void moveBack(double x, bool z) {
  right_drive.spinFor(reverse, x, turns, false);
  left_drive.spinFor(reverse, x, turns, z);
  updateOdometry();
}

void turnLeft(double x) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns);
  updateOdometry();
}

void turnLeft(double x, bool z) {
  left_drive.spinFor(reverse, x, turns, false);
  right_drive.spinFor(fwd, x, turns, z);
  updateOdometry();
}

void turnRight(double x) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns);
  updateOdometry();
}

void turnRight(double x, bool z) {
  left_drive.spinFor(fwd, x, turns, false);
  right_drive.spinFor(reverse, x, turns, z);
  updateOdometry();
}

void rightDegrees(double x) {
  left_drive.spinFor(fwd, x, degrees, false);
  right_drive.spinFor(reverse, x, degrees);
  updateOdometry();
}

void leftDegrees(double x) {
  left_drive.spinFor(reverse, x, degrees, false);
  right_drive.spinFor(fwd, x, degrees);
  updateOdometry();
}

void moveForward(double targetDistance) {
  double kP = 0.3;
  double kI = 0.01;
  double kD = 0.002;
  double Kc = 0.5;
  double error = targetDistance;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  double initialAngle = inertial.rotation(degrees);
  double currentAngle;
  double angleError;

  left_drive.resetPosition();
  right_drive.resetPosition();

  while (fabs(error) > 0.1) {
    currentAngle = inertial.rotation(degrees);
    angleError = initialAngle - currentAngle;

    error = targetDistance - left_drive.position(turns);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower + Kc * angleError, pct);
    right_drive.spin(fwd, motorPower - Kc * angleError, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
  updateOdometry();

}

void moveBack(double targetDistance) {
  double kP = 0.3;
  double kI = 0.01;
  double kD = 0.002;
  double Kc = 0.5;
  double error = targetDistance;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  double initialAngle = inertial.rotation(degrees);
  double currentAngle;
  double angleError;

  left_drive.resetPosition();
  right_drive.resetPosition();

  while (fabs(error) > 0.1) {
    currentAngle = inertial.rotation(degrees);
    angleError = initialAngle - currentAngle;

    error = targetDistance - left_drive.position(turns);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(reverse, motorPower + Kc * angleError, pct);
    right_drive.spin(reverse, motorPower - Kc * angleError, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
  updateOdometry();

}

void turnPID(double targetDegrees) {
  double kP = 0.3;
  double kI = 0.02;
  double kD = 0.005;
  double error = targetDegrees;
  double previousError = 0;
  double integral = 0;
  double derivative;
  double motorPower;

  inertial.resetRotation();

  while (fabs(error) > 1) {
    error = targetDegrees - inertial.rotation(degrees);
    integral += error;
    derivative = error - previousError;
    motorPower = (kP * error) + (kI * integral) + (kD * derivative);

    left_drive.spin(fwd, motorPower, pct);
    right_drive.spin(reverse, motorPower, pct);

    previousError = error;
    wait(20, msec);
  }

  left_drive.stop();
  right_drive.stop();
  updateOdometry();
}

void setLeftWing(bool state) {
  wings.left_wing.set(state);
}

void setRightWing(bool state) {
  wings.right_wing.set(state);
}

void updateOdometry() {

  static double lastLeftEncoder = 0.0;
  static double lastRightEncoder = 0.0;
  static double lastBackEncoder = 0.0;

  double currentLeftEncoder = leftEncoder.position(degrees);
  double currentRightEncoder = rightEncoder.position(degrees);
  double currentBackEncoder = backEncoder.position(degrees);

  double deltaLeft = currentLeftEncoder - lastLeftEncoder;
  double deltaRight = currentRightEncoder - lastRightEncoder;
  double deltaBack = currentBackEncoder - lastBackEncoder;

  lastLeftEncoder = currentLeftEncoder;
  lastRightEncoder = currentRightEncoder;
  lastBackEncoder = currentBackEncoder;

  double deltaLeftInches = (deltaLeft / tpr) * wheelDiameter * M_PI;
  double deltaRightInches = (deltaRight / tpr) * wheelDiameter * M_PI;
  double deltaBackInches = (deltaBack / tpr) * wheelDiameter * M_PI;

  double deltaTheta = (deltaRightInches - deltaLeftInches) / wheelBaseWidth;
  double deltaX = (deltaLeftInches + deltaRightInches) / 2.0;
  double deltaY = deltaBackInches - (deltaTheta * backWheelOffset);

  if(deltaTheta > 180)
			deltaTheta = deltaTheta - 360;
		if(deltaTheta < -180)
			deltaTheta = 360 + deltaTheta;

  robotTheta += deltaTheta;
  robotX += deltaX * cos(robotTheta) - deltaY * sin(robotTheta);
  robotY += deltaX * sin(robotTheta) + deltaY * cos(robotTheta);

}

float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

float to_deg(float angle_rad){
  return(angle_rad*(180.0/M_PI));
}

void purePursuit(std::vector<std::pair<double, double>> path) {
  double lookaheadDistance = 5.0;
  double kP = 0.6;
  double kI = 0.04;
  double kD = 0.3;

  for (auto& point : path) {
    double targetX = point.first;
    double targetY = point.second;

    while (sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2)) > lookaheadDistance) {
      double angleToTarget = atan2(targetY - robotY, targetX - robotX);
      double distanceToTarget = sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2));
      double headingError = angleToTarget - robotTheta;
      double error = distanceToTarget;
      double previousError = 0;
      double integral = 0;
      double derivative;
      double motorPower;

      while (fabs(error) > 0.1) {
        error = distanceToTarget - sqrt(pow(targetX - robotX, 2) + pow(targetY - robotY, 2));
        integral += error;
        derivative = error - previousError;
        motorPower = (kP * error) + (kI * integral) + (kD * derivative);

        left_drive.spin(fwd, motorPower - headingError, pct);
        right_drive.spin(fwd, motorPower + headingError, pct);

        previousError = error;
        wait(20, msec);
        updateOdometry();
      }

  left_drive.stop();
  right_drive.stop();
}

void pre_auton(void) {
  vexcodeInit();

  right_drive.setStopping(brake);
  left_drive.setStopping(brake);
  right_drive.setVelocity(40, pct);
  left_drive.setVelocity(40, pct);
}

void autonomous(void) {

  // Top Right - Bottom Left

	moveForward(2.0);
  wait(1, sec);

  turnPID(90);
  wait(1, sec);

  moveForward(1.0);
  wait(1, sec);

  turnPID(-90);
  wait(1, sec);

  moveForward(2.5);
  wait(1, sec);

  turnPID(180);
  wait(1, sec);

  moveBack(1.5);
  wait(1, sec);

  turnPID(-45);
  wait(1, sec);

  moveForward(3.0);
  wait(1, sec);

  turnPID(90);
  wait(1, sec);

  moveForward(1.5);
  wait(1, sec);

  moveBack(2.0);
  wait(1, sec);

  turnPID(-90);
  wait(1, sec);

  moveForward(1.0);
  wait(1, sec);

  turnPID(45);
  wait(1, sec);

  moveForward(2.5);
  wait(1, sec);

  turnPID(135);
  wait(1, sec);

  moveBack(1.5);
  wait(1, sec);

  turnPID(180);
  wait(1, sec);

  moveForward(3.0);
  wait(1, sec);

  turnPID(-135);
  wait(1, sec);

  moveForward(2.0);
  wait(1, sec);

  turnPID(45);
  wait(1, sec);

  moveBack(1.5);
  wait(1, sec);

  turnPID(-90);
  wait(1, sec);

  moveForward(1.0);
  wait(1, sec);

  turnPID(180);
  wait(1, sec);

  moveBack(2.0);
  wait(1, sec);

  turnPID(90);
  wait(1, sec);

  moveForward(1.5);
  wait(1, sec);


  // Top Left - Bottom Right

	// moveForward(2.0);
  // (1, sec);

  // turnPID(90);
  // wait(1, sec);

  // moveForward(1.5);
  // wait(1, sec);

  // turnPID(45);
  // wait(1, sec);

  // moveBack(1.0);
  // wait(1, sec);

  // turnPID(-90);
  // wait(1, sec);

  // moveForward(2.5);
  // wait(1, sec);

  // turnPID(-45);
  // wait(1, sec);

  // moveBack(1.5);
  // wait(1, sec);

  // turnPID(180);
  // wait(1, sec);

  // moveForward(3.0);
  // wait(1, sec);

  // turnPID(90);
  // wait(1, sec);

  // moveBack(2.0);
  // wait(1, sec);

  // turnPID(135);
  // wait(1, sec);

  // moveForward(1.5);
  // wait(1, sec);

  // turnPID(-135);
  // wait(1, sec);

  // moveBack(1.0);
  // wait(1, sec);

  // turnPID(-180);
  // wait(1, sec);

  // moveForward(2.0);
  // wait(1, sec);

  // turnPID(45);
  // wait(1, sec);

  // moveBack(2.5);
  // wait(1, sec);

  // turnPID(-90);
  // wait(1, sec);

  // moveForward(1.0);
  // wait(1, sec);

  // turnPID(-45);
  // wait(1, sec);

  // moveBack(2.0);
  // wait(1, sec);

  // turnPID(90);
  // wait(1, sec);

  // moveForward(1.5);
  // wait(1, sec);

}

void usercontrol(void) {
  while (true) {

    left_drive.spin(fwd, Controller1.Axis3.position(pct), pct);
    right_drive.spin(fwd, Controller1.Axis2.position(pct), pct);
    wait(20, msec);

    updateOdometry();

    Brain.Screen.print(robotX);
    Brain.Screen.print(robotY);
    Brain.Screen.print(robotTheta);

  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  
  while (true) {
    wait(100, msec);
  }
}
