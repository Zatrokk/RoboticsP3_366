/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#include <Windows.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "../../include/dynamixel_sdk/dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include "../../../../myo-sdk-win-0.9.0 - For tests/include/myo/myo.hpp"



// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL0_ID                         3                   // Dynamixel#0 ID: 0
#define DXL1_ID                         2                   // Dynamixel#1 ID: 1
#define DXL2_ID                         1                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "COM6"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                                            // COM6 for Jonathan

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_STARTING_POSITION_VALUE     2048
#define DXL2_MINIMUM_POSITION_VALUE     400             // Dynamixel will rotate between this value
#define DXL2_MAXIMUM_POSITION_VALUE     4000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL1_MINIMUM_POSITION_VALUE     750             // Dynamixel will rotate between this value
#define DXL1_MAXIMUM_POSITION_VALUE     3300 
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define DXL_CLOSED_GRIPPER_VALUE        2025                   //Value for the position of the dynamixel to close,
#define DXL_OPEN_GRIPPER_VALUE          1500                   //and open the gripper

#define SCREEN_WIDTH                    1500
#define SCREEN_HEIGHT                   1000                //Giver dødzone i bunden af skærmen som det er nu

#define ESC_ASCII_VALUE                 0x1b

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
        }
        else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
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

    int MyPose;

    void GripperPose()
    {

        if (currentPose == myo::Pose::fist)
        {
            MyPose = 0;
            std::cout << "Shut it" << std::endl;
        }
        else if (currentPose == myo::Pose::fingersSpread)
        {
            MyPose = 1;
            std::cout << "Open sesame" << std::endl;
        }
    }

};

int getch()
{
#if defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void)
{
#if defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

double cursorConverter(int cursorPos, double screenValue, double DXL_Max, double DXL_Min)
{
    double a = (DXL_Max - DXL_Min) / screenValue;
    double jointGoal = a * cursorPos + DXL_Min;
    return jointGoal;
}

int main()
{

    myo::Hub hub("Does.This.Mean.Anything");
    std::cout << "Attempting to find a Myo..." << std::endl;
    myo::Myo* myo = hub.waitForMyo(10000);
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
    DataCollector collector;
    hub.addListener(&collector);
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupSyncWrite instance
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

    // Initialize Groupsyncread instance for Present Position
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

    POINT cursorPos;
    int mX = ((DXL_STARTING_POSITION_VALUE - DXL2_MINIMUM_POSITION_VALUE) * SCREEN_WIDTH) / (DXL2_MAXIMUM_POSITION_VALUE - DXL2_MINIMUM_POSITION_VALUE);
    int mY = ((DXL_STARTING_POSITION_VALUE - DXL1_MINIMUM_POSITION_VALUE) * SCREEN_HEIGHT) / (DXL1_MAXIMUM_POSITION_VALUE - DXL1_MINIMUM_POSITION_VALUE);
    bool dontRepeat = false;

    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result
    int dxl_goal_position = DXL_STARTING_POSITION_VALUE;  // Goal position
    int dxl_openClose_position[2] = { DXL_OPEN_GRIPPER_VALUE, DXL_CLOSED_GRIPPER_VALUE };           // Gripper goal position


    uint8_t dxl_error = 0;                            // Dynamixel error
    uint8_t param_goal_position[4];
    int32_t dxl1_present_position = 0, dxl2_present_position = 0;                         // Present position

    // Open port
    if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

  // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    for (int i = 0; i < 5; i++)
    {
        // Enable Dynamixel#i+1 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", i+1);
        }
        // Set Velocity for Dynamixel#i+1
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, 112, 50, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", i + 1);
        }
    }


    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL0_ID);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL0_ID);
        return 0;
    }
    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
        return 0;
    }
    // Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
        return 0;
    }
    // Allocate starting position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position - 1028));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position - 1028));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position - 1028));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position - 1028));
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL0_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL0_ID);
        return 0;
    }
    // Allocate starting position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return 0;
    }
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return 0;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    while (!GetKeyState(VK_CAPITAL))
    {
        if (!dontRepeat)
        {
            std::cout << "Please turn on CapsLock to control the robot\n";
            dontRepeat = true;
        }
    }
    //dontRepeat = false;
    SetCursorPos(mX, mY);

    while (1)
    {
        if (GetKeyState(VK_CAPITAL))
        {
            SetCursorPos(mX, mY);
            dontRepeat = false;
            while (GetKeyState(VK_CAPITAL))
            {
                hub.run(1000 / 20);
                collector.GripperPose();
                GetCursorPos(&cursorPos);
                if (cursorPos.x != mX || cursorPos.y != mY)
                {
                    mX = cursorPos.x;
                    mY = SCREEN_HEIGHT - cursorPos.y;
                    std::cout << mX << " , " << mY << std::endl;
                    int dm1X = cursorConverter(mX, SCREEN_WIDTH, DXL2_MAXIMUM_POSITION_VALUE, DXL2_MINIMUM_POSITION_VALUE);
                    int dm1Y = cursorConverter(mY, SCREEN_HEIGHT, DXL1_MAXIMUM_POSITION_VALUE, DXL1_MINIMUM_POSITION_VALUE);
                    int dm2Y = (1024 * 3) - dm1Y;

                    if (dm2Y <= DXL1_MAXIMUM_POSITION_VALUE && dm2Y >= DXL1_MINIMUM_POSITION_VALUE)
                    {
                        // Allocate DXL1 goal position value into byte array
                        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dm2Y));
                        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dm2Y));
                        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dm2Y));
                        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dm2Y));
                        // Add Dynamixel#2 goal position value to the Syncwrite storage
                        dxl_addparam_result = groupSyncWrite.addParam(DXL0_ID, param_goal_position);
                        if (dxl_addparam_result != true)
                        {
                            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL0_ID);
                            return 0;
                        }
                        // Allocate DXL1 goal position value into byte array
                        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dm1Y));
                        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dm1Y));
                        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dm1Y));
                        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dm1Y));
                        // Add Dynamixel#2 goal position value to the Syncwrite storage
                        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
                        if (dxl_addparam_result != true)
                        {
                            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
                            return 0;
                        }
                    }


                    // Allocate DXL2 goal position value into byte array
                    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dm1X));
                    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dm1X));
                    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dm1X));
                    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dm1X));
                    // Add Dynamixel#1 goal position value to the Syncwrite storage
                    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
                    if (dxl_addparam_result != true)
                    {
                        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
                        return 0;
                    }
                    // Syncwrite goal position
                    dxl_comm_result = groupSyncWrite.txPacket();
                    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                    // Clear syncwrite parameter storage
                    groupSyncWrite.clearParam();
                }
                if (collector.MyPose == 1)              //If up-key is pressed, Open the gripper
                {
                    packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[0], &dxl_error);
                    packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[0], &dxl_error);
                }
                else if (collector.MyPose == 0)       //If down-key is pressed, close the gripper
                {
                    packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[1], &dxl_error);
                    packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[1], &dxl_error);
                }
            }
        }
        if (!dontRepeat)
        {
            std::cout << "Your capsLock is turned off, please turn it on to continue program, or pres Esc to close" << std::endl;
            dontRepeat = true;
        }
        if (GetAsyncKeyState(VK_ESCAPE))
            break;
    }

  /*while(1)
  {
    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
      return 0;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
      return 0;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Syncread present position
      dxl_comm_result = groupSyncRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (groupSyncRead.getError(DXL1_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
      }
      else if (groupSyncRead.getError(DXL2_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
      }

      // Check if groupsyncread data of Dynamixel#1 is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
        return 0;
      }

      // Check if groupsyncread data of Dynamixel#2 is available
      dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
        return 0;
      }

      // Get Dynamixel#1 present position value
      dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // Get Dynamixel#2 present position value
      dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position);

    }while((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }*/

  // Disable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Disable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  
  // Close port
  portHandler->closePort();

  return 0;
}
