// File:          RobotController.cpp
// Date: 2024-0308
// Description: Interfaces between the Robot in the SIM and the external Autonomy_Software code.
// Author: Clayton Cowen (clayjay3@github, claytonraycowen@gmail.com)
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include "Rover.h"

///////////////////////////////////////////////////////////////////////////////////////

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) 
{
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
  
  // Create rover class object.
  Rover* pRover = new Rover(pRoveCommNode);
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (pRover->GetThreadState() != AutonomyThread<void>::AutonomyThreadState::eStopping) 
  {
      // Assemble Rover stats string.
      std::string szMainStatsPrint = "";
      szMainStatsPrint += "Rover Periodic Loop IPS: " + std::to_string(pRover->GetIPS().GetAverageIPS()) + "\n";
      szMainStatsPrint += "RoveCommUDP Node IPS: " + std::to_string(pRoveCommNode->GetIPS().GetAverageIPS()) + "\n";
      // Print out Rover stats.
      // std::cout << szMainStatsPrint << std::endl;
      
      // Call Rover tick. This does not process or send any info to the Autonomy_Software codebase.
      // It simply runs the simluation for a certain amount of time and all of the sensors will have
      // new values when it's done. The Rover has a different periodic loop that runs in a seperate
      // thread to process and send the data over rovecomm.
      pRover->Tick(); 
  };
  
  delete pRover;
  
  // Exit.
  return 0;
}
