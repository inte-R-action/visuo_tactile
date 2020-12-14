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
#include "visuo_tactile/processArguments.h"
#include "std_msgs/Int32.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"


robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;


void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripperStatus = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rq_gripper_2F140");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;
//    robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;

    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusPub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1000, gripperStatusCallback);

    ros::spinOnce();
    loop_rate.sleep();

    while( ros::ok() )
    {
        // reset the robotic gripper (needed to activate the robot)
        outputControlValues.rACT = 0;
        outputControlValues.rGTO = 0;
        outputControlValues.rATR = 0;
        outputControlValues.rPR = 0;
        outputControlValues.rSP = 0;
        outputControlValues.rFR = 0;


        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "RESET GRIPPER" << std::endl;

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

        while( gripperStatus.gSTA != 3 )
        {
            printf("ACTION IN PROGRESS: gSTA [%d]\n", gripperStatus.gSTA);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gSTA [%d]\n", gripperStatus.gSTA);
        sleep(3);

        // close robot gripper to maximum value
        outputControlValues.rPR = 255;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "CLOSE GRIPPER" << std::endl; 

        while( gripperStatus.gOBJ != 3 )
        {
            printf("ACTION IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
        sleep(3);


        // open robot gripper to maximum value
        outputControlValues.rPR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "OPEN GRIPPER" << std::endl; 

        while( gripperStatus.gOBJ != 3 )
        {
            printf("ACTION IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
        sleep(3);

        int gripperSpeed = 20;
        int gripperForce = 50;
        int gripperPosition = 180;

        // decrease speed
        outputControlValues.rSP = gripperSpeed;
        // decrease force
        outputControlValues.rFR = gripperForce;
        // close robot gripper
        outputControlValues.rPR = gripperPosition;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "[speed, force, position] = " << gripperSpeed << ", " << gripperForce << ", " << gripperPosition << std::endl; 

        while( gripperStatus.gOBJ != 3 )
        {
            printf("ACTION IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
        sleep(3);

        // open robot gripper to maximum value
        outputControlValues.rPR = 0;

        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "OPEN GRIPPER" << std::endl; 

        while( gripperStatus.gOBJ != 3 )
        {
            printf("ACTION IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
            usleep(100000);
        }

        printf("ACTION COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
        sleep(3);
    }

    ros::shutdown();

    return 0;
}
