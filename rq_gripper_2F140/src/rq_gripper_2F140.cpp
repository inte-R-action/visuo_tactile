/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 14-December-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 *
 *********************************************************************************
 */

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"


// global variable to hold the status of the gripper
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;

// callback function to get the status signals from the gripper
void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripperStatus = *msg;

    if( msg->gOBJ == 0 )
   		ROS_INFO("ROBOT FINGERS MOVING");
   	else if( msg->gOBJ == 1 )
   		ROS_INFO("ROBOT FINGERS STOPPED DUE TO CONTACT DETECTED WHILE OPENING");
   	else if( msg->gOBJ == 2 )
   		ROS_INFO("ROBOT FINGERS STOPPED DUE TO CONTACT DETECTED WHILE CLOSING");
   	else if( msg->gOBJ == 3 )
   		ROS_INFO("ROBOT FINGERS AT THE REQUESTED POSITION");
   	else
   		ROS_INFO("ROBOT FINGERS ERROR");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rq_gripper_2F140");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusPub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1000, gripperStatusCallback);

    ros::spinOnce();
    loop_rate.sleep();

    while( ros::ok() )
    {
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
        outputControlValues.rSP = 255;
        outputControlValues.rFR = 150;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "ACTIVATE GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gSTA != 3 )
        {
            printf("IN PROGRESS: gSTA [%d]\n", gripperStatus.gSTA);
            usleep(100000);
        }

        printf("COMPLETED: gSTA [%d]\n", gripperStatus.gSTA);
        sleep(1);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);


        // close the gripper to the maximum value of rPR = 255
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rPR = 255;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "CLOSE GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 )
        {
            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);


        // open the gripper to the maximum value of rPR = 0
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rPR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "OPEN GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 )
        {
            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);


        // move the robot gripper to a specific position (rPR) using a defined speed (rSP) and force (rFR)
        // the values for the robot movement are defined by gripperPosition, gripperSpeed and gripperForce
        // rGTO = 1 allows the robot to perform an action
        int gripperSpeed = 20;
        int gripperForce = 50;
        int gripperPosition = 180;

        outputControlValues.rGTO = 1;
        outputControlValues.rSP = gripperSpeed;
        outputControlValues.rFR = gripperForce;
        outputControlValues.rPR = gripperPosition;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "[speed, force, position] = " << gripperSpeed << ", " << gripperForce << ", " << gripperPosition << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 )
        {
            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);


        // open the gripper to the maximum value of rPR = 0
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rPR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "OPEN GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3 )
        {
            printf("ACTION IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(1);
    }

    ros::shutdown();

    return 0;
}
