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
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#include <Windows.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#include <string>
#include <iostream>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_PROFILE_VELOCITY       112

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          3                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      750              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3300             // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_VELOCITY_VALUE              50                    // Value of the dynamixel velocity
#define DXL_POSITION_CHANGE_SAMPLE      25                 // Value of the change of goal position, each sample
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define DXL_CLOSED_GRIPPER_VALUE        2025                   //Value for the position of the dynamixel to close,
#define DXL_OPEN_GRIPPER_VALUE          1500                   //and open the gripper


// declare functions
int getch();
int kbhit(void);
bool succesTest(bool, int);
int testKeyboard(int);
void resultPrint(int, uint8_t, dynamixel::PacketHandler*);


int main()
{
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index = 0;
    int motorAmount = 5;
    int currentMotor = DXL_ID;
    int currentMotorMaxPos = DXL_MAXIMUM_POSITION_VALUE;
    int currentMotorMinPos = DXL_MINIMUM_POSITION_VALUE;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    int dxl_openClose_position[2] = { DXL_OPEN_GRIPPER_VALUE, DXL_CLOSED_GRIPPER_VALUE };           // Gripper goal position
    uint8_t dxl_error = 0;                          // Dynamixel error
    int32_t dxl_present_position = 0;               // Present position
    uint32_t gripperCurrentPosition;
    int gripperGoalPosition;
    int positionChange;

    // Open port
    if (!succesTest(portHandler->openPort(), 1))
        return 0;

    // Set port baudrate
    if (!succesTest(portHandler->setBaudRate(BAUDRATE), 2))
        return 0;

  // Enable Dynamixel Torque For All Motors
    for (int i = 0; i < motorAmount; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel has been successfully connected \n");
        }
    }
    int dxl_goal_position = 2048;
    while(1)
    {
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == VK_ESCAPE)
            break;

        resultPrint(packetHandler->read4ByteTxRx(portHandler, currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error), dxl_error, packetHandler);
        int dxl_goal_position = dxl_present_position;
        
        do
        {
            int a = testKeyboard(currentMotor);
            if (a != currentMotor)
            {
                currentMotor = a;
                resultPrint(packetHandler->write4ByteTxRx(portHandler, currentMotor, ADDR_PRO_PROFILE_VELOCITY, DXL_VELOCITY_VALUE, &dxl_error), dxl_error, packetHandler);
                //switch to min/max value of current motor
                switch (currentMotor)
                {
                case 1:
                    currentMotorMaxPos = 4000;
                    currentMotorMinPos = 430;
                    break;
                case 2:
                case 3:
                    currentMotorMaxPos = DXL_MAXIMUM_POSITION_VALUE;
                    currentMotorMinPos = DXL_MINIMUM_POSITION_VALUE;
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
                resultPrint(packetHandler->write4ByteTxRx(portHandler, currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error), dxl_error, packetHandler);
            }
            else if (GetAsyncKeyState(VK_LEFT))
            {
                dxl_goal_position = dxl_goal_position - DXL_POSITION_CHANGE_SAMPLE;
                // Write goal position 1
                resultPrint(packetHandler->write4ByteTxRx(portHandler, currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error), dxl_error, packetHandler);
            }

            // Read present position
            resultPrint(packetHandler->read4ByteTxRx(portHandler, currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error), dxl_error, packetHandler);
            resultPrint(packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&gripperCurrentPosition, &dxl_error), dxl_error, packetHandler);

            if (dxl_present_position != positionChange)
            {
                positionChange = dxl_present_position;
                printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", currentMotor, dxl_goal_position, dxl_present_position);  // Print current position
            }

            //check if a open or close command is input
            if (GetKeyState(VK_NUMLOCK))
            {
                if (GetAsyncKeyState(VK_UP))              //If up-key is pressed, Open the gripper
                {
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[0], &dxl_error), dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[0], &dxl_error), dxl_error, packetHandler);
                }
                else if (GetAsyncKeyState(VK_DOWN))       //If down-key is pressed, close the gripper
                {
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[1], &dxl_error), dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, dxl_openClose_position[1], &dxl_error), dxl_error, packetHandler);
                }
            }
            else
            {
                if (GetAsyncKeyState(VK_UP))              //If up-key is pressed, slowly closes the gripper
                {
                    gripperGoalPosition = gripperCurrentPosition + DXL_POSITION_CHANGE_SAMPLE;
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, gripperGoalPosition, &dxl_error), dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, gripperGoalPosition, &dxl_error), dxl_error, packetHandler);
                }
                else if (GetAsyncKeyState(VK_DOWN))       //If down-key is pressed, opens the gripper
                {
                    gripperGoalPosition = gripperCurrentPosition - DXL_POSITION_CHANGE_SAMPLE;
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 4, ADDR_PRO_GOAL_POSITION, gripperGoalPosition, &dxl_error), dxl_error, packetHandler);
                    resultPrint(packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PRO_GOAL_POSITION, gripperGoalPosition, &dxl_error), dxl_error, packetHandler);
                }
            }
            
        } while (dxl_goal_position <= currentMotorMaxPos && currentMotorMinPos <= dxl_goal_position);    //Repeat until present position is within the threshold of goal position
        
        std::cout << ("Position out of bound, moving to nearest position\n");
        resultPrint(packetHandler->read4ByteTxRx(portHandler, currentMotor, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error), dxl_error, packetHandler);
        if (dxl_present_position < currentMotorMinPos)
        {
            dxl_goal_position = currentMotorMinPos;
            resultPrint(packetHandler->write4ByteTxRx(portHandler, currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error), dxl_error, packetHandler);
        }
        else if (dxl_present_position > currentMotorMaxPos)
        {
            dxl_goal_position = currentMotorMaxPos;
            resultPrint(packetHandler->write4ByteTxRx(portHandler, currentMotor, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error), dxl_error, packetHandler);
        }
    }

    // Disable Dynamixel Torque
    resultPrint(packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error), dxl_error, packetHandler);

    // Close port
    portHandler->closePort();

    return 0;
}


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}


int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
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
        std::cout<<("Succeeded to " + firstWord + " the " + secondWord + "!\n");
        return 1;
    }
    else
    {
        std::cout<<("Failed to " + firstWord + " the " + secondWord + "!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }
}


int testKeyboard(int newMotor)
{
    if (GetAsyncKeyState(VK_NUMPAD1)!=0)
        newMotor = 1;
    else if (GetAsyncKeyState(VK_NUMPAD2)!=0)
        newMotor = 2;
    else if (GetAsyncKeyState(VK_NUMPAD3)!=0)
        newMotor = 3;

    return newMotor;
}


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