/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 19-December-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description: Example of RobotiQ 2F gripper responding to contact detection from tactile sensor
 *
 *********************************************************************************
 */

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "visuo_tactile/processArguments.h"
#include "std_msgs/Int32.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"


// global variable to hold the status of the gripper
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;

robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;

// global variable to hold the status of the tactile sensor
bool touchDetected = false;

// callback function to get the status signals from the gripper
void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripperStatus = *msg;

}

// callback function to get the status from the tactile sensor
void tactileStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    touchDetected = msg->data;
    if( touchDetected == true )
        ROS_INFO("=== TACTILE STATUS: TRUE ===");
    else
        ROS_INFO("=== TACTILE STATUS: FALSE ===");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rq_gripper_2F140_tactile_response");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(1000);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusSub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1, gripperStatusCallback);
    ros::Subscriber tactileSensorStatusSub = node_handle.subscribe("icmTactileResponse", 1, tactileStatusCallback);


    printf("==================================================\n");
    // reset the robotic gripper (needed to activate the robot)
    outputControlValues.rACT = 0;
    outputControlValues.rGTO = 0;
    outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 0;
    outputControlValues.rFR = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "RESET GRIPPER" << std::endl;

    // give some time the gripper to reset
    sleep(3);


    // activate the robotic gripper
    outputControlValues.rACT = 1;
    outputControlValues.rGTO = 1;
    outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 150;
    outputControlValues.rFR = 100;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "ACTIVATE GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
    while( gripperStatus.gSTA != 3 )
    {
//        printf("IN PROGRESS: gSTA [%d]\n", gripperStatus.gSTA);
//        usleep(1000);
    }

    printf("COMPLETED: gSTA [%d]\n", gripperStatus.gSTA);
    sleep(1);

    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(1);


    while( ros::ok() )
    {
        // close the gripper to the maximum value of rPR = 255
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rSP = 25;
        outputControlValues.rPR = 255;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "CLOSE GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 && touchDetected == false )
        {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;
        outputControlValues.rPR = 0;
        outputControlValues.rSP = 0;
        outputControlValues.rFR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);


        // open the gripper to the maximum value of rPR = 0
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rSP = 50;
        outputControlValues.rPR = 0;
        outputControlValues.rFR = 100;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "OPEN GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 )
        {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);
    }

    // reset the robotic gripper (needed to activate the robot)
    outputControlValues.rACT = 0;
    outputControlValues.rGTO = 0;
    outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 0;
    outputControlValues.rFR = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << " GRIPPER" << std::endl; 
    usleep(1000);

//    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}
