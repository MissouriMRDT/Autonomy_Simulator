// File:          RobotController.cpp
// Date: 2024-0308
// Description: Interfaces between the Robot in the SIM and the external Autonomy_Software code.
// Author: Clayton Cowen (clayjay3@github, claytonraycowen@gmail.com)
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP_MS 64
///////////////////////////////////////////////////////////////////////////////////////

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  webots::Robot *pRobot = new webots::Robot();
  
  // Get motor references.
  webots::Motor* pFrontLeftMotor = pRobot->getMotor("motor1");
  webots::Motor* pFrontRightMotor = pRobot->getMotor("motor2");
  webots::Motor* pBackLeftMotor = pRobot->getMotor("motor3");
  webots::Motor* pBackRightMotor = pRobot->getMotor("motor4");
  // Disable PID controllers on motors.
  pFrontLeftMotor->setPosition(INFINITY);
  pFrontRightMotor->setPosition(INFINITY);
  pBackLeftMotor->setPosition(INFINITY);
  pBackRightMotor->setPosition(INFINITY);
  // Set initial speed to zero.
  pFrontLeftMotor->setVelocity(0);
  pFrontRightMotor->setVelocity(0);
  pBackLeftMotor->setVelocity(0);
  pBackRightMotor->setVelocity(0);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (pRobot->step(TIME_STEP_MS) != -1) {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds->getValue();
  
      // Process sensor data here.
  
      // Enter here functions to send actuator commands, like:
      pFrontLeftMotor->setVelocity(10.0);
      pFrontRightMotor->setVelocity(10.0);
      pBackLeftMotor->setVelocity(10.0);
      pBackRightMotor->setVelocity(10.0);
  };

  // Cleanup code.
  delete pRobot;
  
  // Exit.
  return 0;
}
