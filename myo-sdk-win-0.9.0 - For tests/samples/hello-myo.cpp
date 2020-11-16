// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

#include <conio.h>
#include <Windows.h>
#include <stdlib.h>
#include <stdio.h>

#include "../../DynamixelSDK-3.7.31/c++/include/dynamixel_sdk/dynamixel_sdk.h"                 // Uses Dynamixel SDK library


// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                  // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY       112

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          3                   // Dynamixel ID: 1-5
#define BAUDRATE                        57600
#define DEVICENAME                      "COM6"              // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                                            // COM6 For Jonathan

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      750                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3300                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_VELOCITY_VALUE              25                  // Value of the dynamixel velocity
#define DXL_POSITION_CHANGE_SAMPLE      50                  // Value of the change of goal position, each sample
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define DXL_CLOSED_GRIPPER_VALUE        2025                   //Value for the position of the dynamixel to close,
#define DXL_OPEN_GRIPPER_VALUE          1500                   //and open the gripper

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
        : onArm(false), isUnlocked(false), currentPose()
    {
    }

    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
            // Myo becoming locked.
            myo->unlock(myo::Myo::unlockHold);

            // Notify the Myo that the pose has resulted in an action, in this case changing
            // the text on the screen. The Myo will vibrate.
            myo->notifyUserAction();
            //        } else {
                        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
                        // are being performed, but lock after inactivity.
            //            myo->unlock(myo::Myo::unlockTimed);
        }
    }


    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
        myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }

    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.

    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';

        if (onArm) {
            // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

            // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
            // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
            // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
            std::string poseString = currentPose.toString();

            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
            std::cout << "Motor: " << currentMotor << std::endl;
        }
        else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
            std::cout << "Motor: " << currentMotor << std::endl;
        }

        std::cout << std::flush;
    }

    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;

    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;

    // These values are set by onPose() above.
    myo::Pose currentPose;

    /*************************DYNAMIXEL STUFF**************************************/

    int motorAmount = 5-1;
    int currentMotor = DXL_ID;

    void ChangeMotor()
    {

        if (currentPose == myo::Pose::fist && currentMotor < motorAmount && currentMotor >= 1)
        {
            currentMotor++;
        }
        else if (currentPose == myo::Pose::fingersSpread && currentMotor <= motorAmount && currentMotor > 1)
        {
            currentMotor--;
        }
        else if (currentMotor < 1 || currentMotor > motorAmount)
        {
            std::cout << "ERROR: Motor cannot be larger than " << motorAmount << std::endl;
        }
    }

    int getCurrentMotor()
    {
        return currentMotor;
    }

    int currentMotorMaxPos = DXL_MAXIMUM_POSITION_VALUE;
    int currentMotorMinPos = DXL_MINIMUM_POSITION_VALUE;
    int32_t dxl_present_position = 0;
    int dxl_goal_position = 2048;

    void SpinMotor()
    {
        if (currentPose == myo::Pose::waveIn && dxl_present_position <= currentMotorMaxPos)
        {
            // turn left - up in position
            dxl_goal_position = dxl_goal_position + DXL_POSITION_CHANGE_SAMPLE;
            // Write goal position 1
            std::cout << "Turning left" << std::endl;
        }
        else if (currentPose == myo::Pose::waveOut && dxl_present_position <= currentMotorMinPos)
        {
            // turn right -  down in position
            dxl_goal_position = dxl_goal_position - DXL_POSITION_CHANGE_SAMPLE;
            // Write goal position 1
            std::cout << "Turning right" << std::endl;
        }
    }

    bool succesTest(bool test, int mode)
    {
        std::string firstWord;
        std::string secondWord;
        switch (mode)
        {
        case 1:
            firstWord = "open";
            secondWord = "port";
            break;
        case 2:
            firstWord = "change";
            secondWord = "baudrate";
            break;
        default:
            break;
        }

        if (test)
        {
            std::cout << ("Succeeded to " + firstWord + " the " + secondWord + "!\n");
            return 1;
        }
        else
        {
            std::cout << ("Failed to " + firstWord + " the " + secondWord + "!\n");
            printf("Press any key to terminate...\n");
            getch();
            return 0;
        }
    }

    int getch()
    {
#if defined(_WIN32) || defined(_WIN64)
        return _getch();
#endif
    }

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

//    void resultPrint(dynamixel::PacketHandler* packetHandler)
//    {
//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        }
//        else if (dxl_error != 0)
//        {
//            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        }
//    }

    uint32_t gripperCurrentPosition;
    int gripperGoalPosition;
    int positionChange;
    int dxl_openClose_position[2] = { DXL_OPEN_GRIPPER_VALUE, DXL_CLOSED_GRIPPER_VALUE };           // Gripper goal position

};

void resultPrint(int dxl_comm_result, uint8_t dxl_error, dynamixel::PacketHandler* packetHandler)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}


int main(int argc, char** argv)
{

    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("Does.This.Mean.Anything");

    std::cout << "Attempting to find a Myo..." << std::endl;

    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    myo::Myo* myo = hub.waitForMyo(10000);

    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


    // Open port
    if (!collector.succesTest(portHandler->openPort(), 1))
        return 0;

    // Set port baudrate
    if (!collector.succesTest(portHandler->setBaudRate(BAUDRATE), 2))
        return 0;

    for (int i = 0; i < collector.motorAmount; i++)
    {
        collector.dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &collector.dxl_error);
        if (collector.dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("%s\n", packetHandler->getTxRxResult(collector.dxl_comm_result));
        }
        else if (collector.dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("%s\n", packetHandler->getRxPacketError(collector.dxl_error));
        }
        else
        {
            printf("Dynamixel has been successfully connected \n");
        }
    }
    // Finally we enter our main loop.
    while (1)
    {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
//        hub.run(1000 / 20);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
//        collector.print();
//        collector.SpinMotor();

        ///////////////////////////DYNAMIXEL////////////////////

        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == VK_ESCAPE)
            break;

        resultPrint(packetHandler->read4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&collector.dxl_present_position, &collector.dxl_error), collector.dxl_error, packetHandler);
        int dxl_goal_position = collector.dxl_present_position;
        int u = 0;
        do
        {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
            hub.run(1000 / 20);
            if (u > 10)
            {
                collector.ChangeMotor();
                u = 0;
            }
            u++;
            int a = collector.currentMotor;
            if (a != collector.currentMotor)
            {
                collector.currentMotor = a;
                resultPrint(packetHandler->write4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_PROFILE_VELOCITY, DXL_VELOCITY_VALUE, &collector.dxl_error), collector.dxl_error, packetHandler);
                //switch to min/max value of current motor
                switch (collector.currentMotor)
                {
                case 1:
                    collector.currentMotorMaxPos = 4000;
                    collector.currentMotorMinPos = 430;
                    break;
                case 2:
                case 3:
                    collector.currentMotorMaxPos = DXL_MAXIMUM_POSITION_VALUE;
                    collector.currentMotorMinPos = DXL_MINIMUM_POSITION_VALUE;
                    break;
                default:
                    break;
                }
                break;
            }
            if (GetAsyncKeyState(VK_RIGHT))
            {
                dxl_goal_position = dxl_goal_position + DXL_POSITION_CHANGE_SAMPLE;
                // Write goal position 1
                resultPrint(packetHandler->write4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &collector.dxl_error), collector.dxl_error, packetHandler);
            }
            else if (GetAsyncKeyState(VK_LEFT))
            {
                dxl_goal_position = dxl_goal_position - DXL_POSITION_CHANGE_SAMPLE;
                // Write goal position 1
                resultPrint(packetHandler->write4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &collector.dxl_error), collector.dxl_error, packetHandler);
            }

            // Read present position
            resultPrint(packetHandler->read4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&collector.dxl_present_position, &collector.dxl_error), collector.dxl_error, packetHandler);
            resultPrint(packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&collector.gripperCurrentPosition, &collector.dxl_error), collector.dxl_error, packetHandler);

            if (collector.dxl_present_position != collector.positionChange)
            {
                collector.positionChange = collector.dxl_present_position;
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", collector.currentMotor, dxl_goal_position, collector.dxl_present_position);  // Print current position
            }

            //check if a open or close command is input
            if (GetKeyState(VK_CAPITAL))
            {
                if (GetAsyncKeyState(VK_UP))            //If up-key is pressed, Open the gripper
                {
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, collector.dxl_openClose_position[0], &collector.dxl_error), collector.dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, collector.dxl_openClose_position[0], &collector.dxl_error), collector.dxl_error, packetHandler);
                }
                else if (GetAsyncKeyState(VK_DOWN))       //If down-key is pressed, close the gripper
                {
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, collector.dxl_openClose_position[1], &collector.dxl_error), collector.dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, collector.dxl_openClose_position[1], &collector.dxl_error), collector.dxl_error, packetHandler);
                }
            }
            else
            {
                if (GetAsyncKeyState(VK_UP))              //If up-key is pressed, slowly closes the gripper
                {
                    collector.gripperGoalPosition = collector.gripperCurrentPosition + DXL_POSITION_CHANGE_SAMPLE;
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, collector.gripperGoalPosition, &collector.dxl_error), collector.dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, collector.gripperGoalPosition, &collector.dxl_error), collector.dxl_error, packetHandler);
                }
                else if (GetAsyncKeyState(VK_DOWN))       //If down-key is pressed, opens the gripper
                {
                    collector.gripperGoalPosition = collector.gripperCurrentPosition - DXL_POSITION_CHANGE_SAMPLE;
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, collector.gripperGoalPosition, &collector.dxl_error), collector.dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, collector.gripperGoalPosition, &collector.dxl_error), collector.dxl_error, packetHandler);
                }
            }
        } while (collector.dxl_goal_position <= collector.currentMotorMaxPos && collector.currentMotorMinPos <= collector.dxl_goal_position);    //Repeat until present position is within the threshold of goal position
        
        std::cout << ("Position out of bound, moving to nearest position\n");
        resultPrint(packetHandler->read4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&collector.dxl_present_position, &collector.dxl_error), collector.dxl_error, packetHandler);
        if (collector.dxl_present_position < collector.currentMotorMinPos)
        {
            dxl_goal_position = collector.currentMotorMinPos;
            resultPrint(packetHandler->write4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &collector.dxl_error), collector.dxl_error, packetHandler);
        }
        else if (collector.dxl_present_position > collector.currentMotorMaxPos)
        {
            dxl_goal_position = collector.currentMotorMaxPos;
            resultPrint(packetHandler->write4ByteTxRx(portHandler, collector.currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &collector.dxl_error), collector.dxl_error, packetHandler);
        }
    }

    // Disable Dynamixel Torque
    resultPrint(packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &collector.dxl_error), collector.dxl_error, packetHandler);

    // Close port
    portHandler->closePort();

    return 0;
}
