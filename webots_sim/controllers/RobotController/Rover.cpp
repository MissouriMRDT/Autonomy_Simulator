/******************************************************************************
 * @brief Implements the Rover class.
 *
 * @file Rover.cpp
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#include "Rover.h"

#include <RoveComm/RoveCommManifest.h>
#include <cmath>


/******************************************************************************
 * @brief Construct a new Rover:: Rover object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
Rover::Rover(rovecomm::RoveCommUDP* pRoveCommUDPNode)
{
    // Initialize member variables.
    m_pRoveCommUDPNode = pRoveCommUDPNode;
    m_nColorCombine = 0;

    // Create class objects.
    m_pFrontLeftMotor  = this->getMotor(ROVER_FRONTLEFT_MOTOR_NAME);
    m_pFrontRightMotor = this->getMotor(ROVER_FRONTRIGHT_MOTOR_NAME);
    m_pBackLeftMotor   = this->getMotor(ROVER_BACKLEFT_MOTOR_NAME);
    m_pBackRightMotor  = this->getMotor(ROVER_BACKRIGHT_MOTOR_NAME);

    // Configure robot motors.
    m_pFrontLeftMotor->setPosition(INFINITY);
    m_pFrontRightMotor->setPosition(INFINITY);
    m_pBackLeftMotor->setPosition(INFINITY);
    m_pBackRightMotor->setPosition(INFINITY);
    // Set initial speed to zero.
    m_pFrontLeftMotor->setVelocity(0);
    m_pFrontRightMotor->setVelocity(0);
    m_pBackLeftMotor->setVelocity(0);
    m_pBackRightMotor->setVelocity(0);
    
    // Set RoveComm Callbacks.
    m_pRoveCommUDPNode->AddUDPCallback<float>(ProcessDriveData, manifest::Core::COMMANDS.find("DRIVELEFTRIGHT")->second.DATA_ID);
    m_pRoveCommUDPNode->AddUDPCallback<uint8_t>(ProcessRGBData, manifest::Core::COMMANDS.find("LEDRGB")->second.DATA_ID);
    m_pRoveCommUDPNode->AddUDPCallback<uint8_t>(ProcessStateDisplayData, manifest::Core::COMMANDS.find("STATEDISPLAY")->second.DATA_ID);
    
    // Get cameras.
    m_pMainCam = this->getCamera(ROVER_MAINCAM_NAME);
    
    // Get depth rangefinders for cameras.
    m_pMainDepthCam = this->getRangeFinder(ROVER_MAINRANGEFINDER_NAME);
    
    // Get other sensors.
    m_pGPS     = this->getGPS(ROVER_GPS_NAME);
    m_pCompass = this->getCompass(ROVER_COMPASS_NAME);
    m_pLED     = this->getLED(ROVER_LED_NAME);
    
    // Configure cameras and sensors.
    m_pMainCam->enable(ROVER_TIMESTEP_MS);
    m_pMainDepthCam->enable(ROVER_TIMESTEP_MS);
    m_pGPS->enable(ROVER_TIMESTEP_MS);
    m_pCompass->enable(ROVER_TIMESTEP_MS);
    m_pLED->set(this->CombineRGB(0, 0, 0));
    
    
    std::srand(static_cast<unsigned int>(std::time(nullptr))); // Seed the random number generator

    // Set a max IPS for the rover thread.
    this->SetMainThreadIPSLimit(ROVER_THREAD_MAX_IPS);
    // Start main Rover periodic loop in seperate thread.
    this->Start();
}

/******************************************************************************
 * @brief Destroy the Rover:: Rover object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
Rover::~Rover()
{
    // Nothing to destroy.
}

/******************************************************************************
 * @brief The code in this method runs within a seperate thread. This is the main
 *      periodic loop for the webots robot object.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
void Rover::ThreadedContinuousCode()
{
    // Create static instance variables.
    static bool bWatchdogAlreadyTriggered = false;
    
    // Add requests to update request queue.
    m_qUpdateRequests.push(eMainCamera);
    m_qUpdateRequests.push(eMainRangeFinder);
    m_qUpdateRequests.push(eGPS);
    m_qUpdateRequests.push(eCompass);
    m_qUpdateRequests.push(eAccuracy);
    // Start the thread pool to store multiple copies of the sl::Mat into the given cv::Mats.
    this->RunDetachedPool(m_qUpdateRequests.size(), ROVER_REQUEST_POOL_THREADS);

    // Get current time.
    std::chrono::system_clock::time_point tmCurrentTime = std::chrono::system_clock::now();
    // Acquire read lock for checking watchdog timer.
    std::shared_lock<std::shared_mutex> lkWatchdogTimerLock(m_muWatchdogMutex);
    // Calculate time elapsed since last motor update.
    int nTimeElapsed = std::chrono::duration_cast<std::chrono::seconds>(tmCurrentTime - m_tmWatchdogLastUpdateTime).count();
    // Unlock mutex.
    lkWatchdogTimerLock.unlock();
    
    // Check drive watchdog.
    if (nTimeElapsed > static_cast<int>(ROVER_MOTOR_WATCHDOG_TIMEOUT) && !bWatchdogAlreadyTriggered)
    {
        // Acquire write lock for updating motor powers.
        std::unique_lock<std::shared_mutex> lkMotorLock(m_muDriveMotorMutex);
        // Stop motors.
        m_pFrontLeftMotor->setVelocity(0);
        m_pFrontRightMotor->setVelocity(0);
        m_pBackLeftMotor->setVelocity(0);
        m_pBackRightMotor->setVelocity(0);
        // Release lock.
        lkMotorLock.unlock();
        
        // Set watchdog triggered.
        bWatchdogAlreadyTriggered = true;
    }
    else if (nTimeElapsed <= static_cast<int>(ROVER_MOTOR_WATCHDOG_TIMEOUT) && bWatchdogAlreadyTriggered)
    {
        // Reset wathdog triggered.
        bWatchdogAlreadyTriggered = false;
    }
    
    // Acquire write lock for updating motor powers.
    std::unique_lock<std::shared_mutex> lkMotorLock(m_muDriveMotorMutex);
    // Set motor power to values given from rovecomm.
    m_pFrontLeftMotor->setVelocity(m_fLeftMotorPower);
    m_pBackLeftMotor->setVelocity(m_fLeftMotorPower);
    m_pFrontRightMotor->setVelocity(m_fRightMotorPower);
    m_pBackRightMotor->setVelocity(m_fRightMotorPower);
    // Release locks.
    lkMotorLock.unlock();
    
    // Acquire write lock for updating motor powers.
    std::unique_lock<std::shared_mutex> lkLEDLock(m_muLEDMutex);
    // Set LED color.
    m_pLED->set(m_nColorCombine);
    // Release lock.
    lkLEDLock.unlock();
    
    // Acquire write lock for getting sensor values.
    std::unique_lock<std::shared_mutex> lkSensorsLock(m_muSensorsMutex);
    // Call Rover tick. This does not process or send any info to the Autonomy_Software codebase.
    // It simply runs the simluation for a certain amount of time and all of the sensors will have
    // new values when it's done. The Rover has a different periodic loop that runs in a seperate
    // thread to process and send the data over rovecomm.
    // Check if the robots periodic loop should stop.
    if (this->step(ROVER_TIMESTEP_MS) == -1)
    {
        // Signal this thread to stop.
        this->RequestStop();
    }
    // Release locks.
    lkSensorsLock.unlock();
    
    // Wait for thread pool to finish.
    this->JoinPool();
}

/******************************************************************************
 * @brief This method is used to store code that will have replicants ran in parallel
 *      via a threadpool. This is mainly a util method for ThreadedContinuousCode().
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
void Rover::PooledLinearCode() 
{
    // Acquire read lock for getting sensor values.
    std::shared_lock<std::shared_mutex> lkSensorsLock(m_muSensorsMutex);
    
    // Acquire write lock for getting sensor values.
    std::unique_lock<std::shared_mutex> lkUpdatesLock(m_muRequestQueueMutex);
    // Check if the queue is empty.
    if (!m_qUpdateRequests.empty())
    {
        // Get request type out of queue.
        SensorUpdateRequest eUpdateRequestType = m_qUpdateRequests.front();
        // Pop old request out of queue.
        m_qUpdateRequests.pop();
        // Release lock.
        lkUpdatesLock.unlock();
    
        // Switch statement for sensor update type.
        switch (eUpdateRequestType)
        {
            case eMainCamera:
                break;
                
            case eMainRangeFinder:
                break;
                
            case eGPS:
            {
                // Get GPS sensor location and altitude.
                const double* dLatLonAlt = m_pGPS->getValues();
                // Package into a RoveCommPacket.
                rovecomm::RoveCommPacket<double> stGPSPacket;
                stGPSPacket.unDataId    = manifest::Nav::TELEMETRY.find("GPSLATLONALT")->second.DATA_ID;
                stGPSPacket.unDataCount = manifest::Nav::TELEMETRY.find("GPSLATLONALT")->second.DATA_COUNT;
                stGPSPacket.eDataType   = manifest::Nav::TELEMETRY.find("GPSLATLONALT")->second.DATA_TYPE;
                stGPSPacket.vData.emplace_back(dLatLonAlt[0]);
                stGPSPacket.vData.emplace_back(dLatLonAlt[1]);
                stGPSPacket.vData.emplace_back(dLatLonAlt[2]);
                // Send drive command over RoveComm to drive board.
                m_pRoveCommUDPNode->SendUDPPacket<double>(stGPSPacket, "127.0.0.1", manifest::General::ETHERNET_UDP_PORT);
                break;
            }
                
            case eCompass:
            {
                // Get GPS sensor location and altitude.
                const double* dOrientation = m_pCompass->getValues();
                // Package into a RoveCommPacket.
                rovecomm::RoveCommPacket<float> stCompassPacket;
                stCompassPacket.unDataId    = manifest::Nav::TELEMETRY.find("COMPASSDATA")->second.DATA_ID;
                stCompassPacket.unDataCount = manifest::Nav::TELEMETRY.find("COMPASSDATA")->second.DATA_COUNT;
                stCompassPacket.eDataType   = manifest::Nav::TELEMETRY.find("COMPASSDATA")->second.DATA_TYPE;
                // Convert orientation vector to z-axis bearing.
                double dRad = std::atan2(dOrientation[1], dOrientation[0]);
                double dHeading = (dRad - 1.5708) / M_PI * 180.0;
                if (dHeading < 0.0)
                {
                    dHeading += 360.0;
                }
                stCompassPacket.vData.emplace_back(static_cast<float>(dHeading));
                // Send drive command over RoveComm to drive board.
                m_pRoveCommUDPNode->SendUDPPacket<float>(stCompassPacket, "127.0.0.1", manifest::General::ETHERNET_UDP_PORT);
                break;
            }
                
            case eAccuracy:
            {
                // Package into a RoveCommPacket.
                rovecomm::RoveCommPacket<float> stAccuracyPacket;
                stAccuracyPacket.unDataId    = manifest::Nav::TELEMETRY.find("ACCURACYDATA")->second.DATA_ID;
                stAccuracyPacket.unDataCount = manifest::Nav::TELEMETRY.find("ACCURACYDATA")->second.DATA_COUNT;
                stAccuracyPacket.eDataType   = manifest::Nav::TELEMETRY.find("ACCURACYDATA")->second.DATA_TYPE;
                std::vector<double> vOriginalValues = {0.05, 0.08, 1.0};            
                for (double dValue : vOriginalValues) {
                    double dRandomVariation = ((std::rand() % 201) - 100) / 10000.0; // Random variation between -0.01 and 0.01
                    stAccuracyPacket.vData.emplace_back(dValue + dRandomVariation);
                }
                // Add Fix type and diff boolean.
                stAccuracyPacket.vData.emplace_back(3.0);
                stAccuracyPacket.vData.emplace_back(1.0);
                // Send drive command over RoveComm to drive board.
                m_pRoveCommUDPNode->SendUDPPacket<float>(stAccuracyPacket, "127.0.0.1", manifest::General::ETHERNET_UDP_PORT);
                break;
            }
                
            default:
                break;
        }
        
    }
}

/******************************************************************************
 * @brief This method is used internally by the Rover class. It takes three values 
 *        (RBG) each values is from 0-255 and stitch them together into a single 
 *        integer the maintains the binary values of each individual color value.
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
int Rover::CombineRGB(int nR, int nG, int nB)
{
    // Make sure each color value is within the valid range [0, 255]
    nR = std::min(std::max(nR, 0), 255);
    nG = std::min(std::max(nG, 0), 255);
    nB = std::min(std::max(nB, 0), 255);

    // Combine RGB values into a single integer using bitwise OR and shifting
    return (nR << 16) | (nG << 8) | nB;
}
