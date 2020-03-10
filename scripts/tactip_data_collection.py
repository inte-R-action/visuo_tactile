#!/usr/bin/env python


'''
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 5-March-2020
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 * - Data collection with the TacTip sensor
 *
 *********************************************************************************
'''


import rospy
from std_msgs.msg import String
import time
import sys
from visuo_tactile.msg import processArguments


control_data = ""
outputControlString = ""
tactipReady = False

operationMode = "";
folderPath = "";
objectLabel = "";
numOfIterations = 0;


def controlInCallback(data_in):
    global control_data;
    global tactipReady;

    if( tactipReady == False ):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s from TacTip ", data_in.data)
        control_data = data_in.data
        tactipReady = True


def processArgumentsCallback(data_in):
    global operationMode;
    global folderPath;
    global objectLabel;
    global numOfIterations;

    operationMode = data_in.mode;
    folderPath = data_in.folder;
    objectLabel = data_in.label;
    numOfIterations = data_in.iterations;

    rospy.loginfo(rospy.get_caller_id() + " Process arguments: %s, %s, %s, %d", operationMode, folderPath, objectLabel, numOfIterations)


def tactipSensorMain(sys):
    global tactipReady

    rospy.init_node('tactip_sensor');

    tactipPub = rospy.Publisher('tactip_in', String, queue_size = 1)
    rospy.Subscriber('tactip_out', String, controlInCallback);

    rospy.Subscriber('process_arguments', processArguments, processArgumentsCallback);

    rate = rospy.Rate(10);

    while not rospy.is_shutdown():
        if( tactipReady == True ):
            if( control_data == "tactip_down" ):
                rospy.loginfo("Saving images")
                outputControlString = "data_collected";

                # Data collection, training or classification
                
            
            elif( control_data == "tactip_up" ):
                rospy.loginfo("Waiting for sensor")
                outputControlString = "waiting_for_tactip";

            elif( control_data == "end_of_process" ):
                rospy.loginfo("Shutting down python")
                outputControlString = "Shutting_down";
                break

            else:
                rospy.loginfo("Unrecognised input signal")
                outputControlString = "unrecognised_signal";

            tactipPub.publish(outputControlString);
            outputControlString = "";
            tactipReady = False;

            rate.sleep();

        else:
            pass

    rospy.signal_shutdown("End of process");



if __name__ == '__main__':
    try:
        tactipSensorMain(sys);
    except rospy.ROSInterruptException:
        pass

