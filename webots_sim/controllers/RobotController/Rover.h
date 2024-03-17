/******************************************************************************
 * @brief Defines the Rover class.
 *
 * @file Rover.h
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 *
 * @copyright Copyright Mars Rover Design Team 2024 - All Rights Reserved
 ******************************************************************************/

#ifndef ROVER_H
#define ROVER_H

#include "NumberOperations.hpp"

#include <shared_mutex>
#include <chrono>
#include <atomic>
#include <queue>
#include <RoveComm/RoveComm.h>
#include <interfaces/AutonomyThread.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/LED.hpp>

// Define class constants.
const unsigned int ROVER_TIMESTEP_MS                = 20;       // The amount of time to run the simulation before returning with updated values.
const unsigned int ROVER_THREAD_MAX_IPS             = 120;      // The max iterations per second of the rover periodic loop.
const unsigned int ROVER_MOTOR_WATCHDOG_TIMEOUT     = 2;        // Motor watchdog timeout in seconds.
const float ROVER_MAX_VELOCITY                      = 10;
const std::string ROVER_FRONTLEFT_MOTOR_NAME        = "motor1";
const std::string ROVER_FRONTRIGHT_MOTOR_NAME       = "motor2";
const std::string ROVER_BACKLEFT_MOTOR_NAME         = "motor3";
const std::string ROVER_BACKRIGHT_MOTOR_NAME        = "motor4";
const std::string ROVER_MAINCAM_NAME                = "camera1";
const std::string ROVER_MAINRANGEFINDER_NAME        = "rangefinder1";
const std::string ROVER_GPS_NAME                    = "gps";
const std::string ROVER_COMPASS_NAME                = "compass";
const std::string ROVER_LED_NAME                    = "led";
const std::memory_order ATOMIC_MEMORY_ORDER_METHOD  = std::memory_order_relaxed;
const unsigned int ROVER_REQUEST_POOL_THREADS       = 10;

///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * @brief This class is threaded and interfaces with the Webots API to get sensors
 *      and
 *
 *
 * @author clayjay3 (claytonraycowen@gmail.com)
 * @date 2024-03-14
 ******************************************************************************/
class Rover : public AutonomyThread<void>, public webots::Robot
{
    private:
        /////////////////////////////////////////
        // Declare private enums that are specific to and used within this class.
        /////////////////////////////////////////
        
        // Enum for storing update request types for sensor data.
        enum SensorUpdateRequest
        {
            eMainCamera,
            eMainRangeFinder,
            eGPS,
            eCompass,
            eAccuracy
        };
    
        /////////////////////////////////////////
        // Declare private member variables.
        /////////////////////////////////////////
        // RoveComm UDP Node.
        rovecomm::RoveCommUDP* m_pRoveCommUDPNode;
        webots::Motor* m_pFrontLeftMotor;
        webots::Motor* m_pFrontRightMotor;
        webots::Motor* m_pBackLeftMotor;
        webots::Motor* m_pBackRightMotor;
        webots::Camera* m_pMainCam;
        webots::RangeFinder* m_pMainDepthCam;
        webots::GPS* m_pGPS;
        webots::Compass* m_pCompass;
        webots::LED* m_pLED;
        float m_fLeftMotorPower;
        float m_fRightMotorPower;
        std::shared_mutex m_muDriveMotorMutex;
        int m_nColorCombine;
        std::shared_mutex m_muLEDMutex;
        std::shared_mutex m_muSensorsMutex;
        std::shared_mutex m_muWatchdogMutex;
        std::chrono::system_clock::time_point m_tmWatchdogLastUpdateTime;
        
        // Threadpool variables.
        std::queue<SensorUpdateRequest> m_qUpdateRequests;
        std::shared_mutex m_muRequestQueueMutex;

        /////////////////////////////////////////
        // Declare private methods.
        /////////////////////////////////////////

        void ThreadedContinuousCode() override;
        void PooledLinearCode() override;
        int CombineRGB(int nR, int nG, int nB);
        
        /////////////////////////////////////////
        // RoveComm Callbacks.
        /////////////////////////////////////////
        
        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new Accuracy data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<float>&, const sockaddr_in&)> ProcessDriveData =
            [this](const rovecomm::RoveCommPacket<float>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire write lock for updating motor powers.
            std::unique_lock<std::shared_mutex> lkWriteMotorLock(m_muDriveMotorMutex);
            // Map the incoming -1 to 1 drive power values to our max speed output for the virtual rover.
            m_fLeftMotorPower = numops::MapRange(stPacket.vData[0], -1.0f, 1.0f, -ROVER_MAX_VELOCITY, ROVER_MAX_VELOCITY);
            m_fRightMotorPower = numops::MapRange(stPacket.vData[1], -1.0f, 1.0f, -ROVER_MAX_VELOCITY, ROVER_MAX_VELOCITY);
            // Release mutex lock.
            lkWriteMotorLock.unlock();

            // Acquire write lock for updating watchdog timer.
            std::unique_lock<std::shared_mutex> lkWatchdogTimerLock(m_muWatchdogMutex);
            m_tmWatchdogLastUpdateTime = std::chrono::system_clock::now();
        };
        
        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new RGB data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> ProcessRGBData =
            [this](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            // Acquire write lock for updating LED/
            std::unique_lock<std::shared_mutex> lkWriteMotorLock(m_muLEDMutex);
            // Update LED values.
            m_nColorCombine = this->CombineRGB(stPacket.vData[0], stPacket.vData[1], stPacket.vData[2]);
            // Release mutex lock.
            lkWriteMotorLock.unlock();
        };
        
        /******************************************************************************
         * @brief Callback function that is called whenever RoveComm receives new StateDisplay data.
         *
         *
         * @author clayjay3 (claytonraycowen@gmail.com)
         * @date 2024-03-03
         ******************************************************************************/
        const std::function<void(const rovecomm::RoveCommPacket<uint8_t>&, const sockaddr_in&)> ProcessStateDisplayData =
            [this](const rovecomm::RoveCommPacket<uint8_t>& stPacket, const sockaddr_in& stdAddr)
        {
            // Not using this.
            (void) stdAddr;

            
            // Acquire write lock for updating LED.
            std::unique_lock<std::shared_mutex> lkWriteMotorLock(m_muLEDMutex);
            // Determine display state.
            switch (stPacket.vData[0])
            {
                case 0:
                {
                    // Teleop Blue
                    m_nColorCombine = this->CombineRGB(0, 0, 255);
                    break;
                }
                case 1:
                {
                    // Autonomy Red
                    m_nColorCombine = this->CombineRGB(255, 0, 0);
                    break;
                }
                case 2:
                {
                    // Goal Green
                    m_nColorCombine = this->CombineRGB(0, 255, 0);
                    break;
                }
                default:
                    break;
            }
            // Release mutex lock.
            lkWriteMotorLock.unlock();
        };

    public:
        /////////////////////////////////////////
        // Declare public methods and member variables.
        /////////////////////////////////////////

        Rover(rovecomm::RoveCommUDP* pRoveCommUDPNode);
        ~Rover();

        /////////////////////////////////////////
        // Getters.
        /////////////////////////////////////////
};

#endif
