// File:          RobotController.cpp
// Date: 2024-0308
// Description: Interfaces between the Robot in the SIM and the external Autonomy_Software code.
// Author: Clayton Cowen (clayjay3@github, claytonraycowen@gmail.com)
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <RoveComm/RoveComm.h>

#define TIME_STEP_MS 16
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
  
  // Create RoveComm Node.
  rovecomm::RoveCommUDP* pRoveCommNode = new rovecomm::RoveCommUDP();
  // Initialize RoveComm Node.
  unsigned int unRoveCommPort = 11001;
  bool bRoveCommUDPInitSuccess = pRoveCommNode->InitUDPSocket(unRoveCommPort);
  if (bRoveCommUDPInitSuccess)
  {
      // Print status.
      std::cout << "RoveCommUDP node initialized successfully on port " << unRoveCommPort << std::endl; 
  }
  else
  {
      // Print status.
      std::cout << "Failed to initialize RoveCommUDP node on port " << unRoveCommPort << std::endl;
  }
  
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
  while (pRobot->step(TIME_STEP_MS) != -1) 
  {
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = ds->getValue();
  
      // Process sensor data here.
 
      
  };

  // Cleanup code.
  delete pRobot;
  
  // Exit.
  return 0;
}
