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
from std_msgs.msg import Int32
import time
import sys
from visuo_tactile.msg import processArguments
import os

control_data = ""
outputControlString = ""
tactipReady = False

operationMode = "";
folderPath = "";
objectLabel = "";
numOfIterations = 0;
tactipIteration = 0;

def controlInCallback(data_in):
    global control_data;
    global tactipReady;

    if( tactipReady == False ):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s from TacTip ", data_in.data)
        control_data = data_in.data
        tactipReady = True


def createDirectory():
    if not os.path.exists(folderPath + '/' + objectLabel):
        os.makedirs(folderPath + '/' + objectLabel)
        print("Directory doesn't exist");
        print("Directory created: ", folderPath + '/' + objectLabel);
    else:
        print("Directory exists");


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

    createDirectory();



def tactipIterationCallback(data_in):
    global tactipIteration;

    tactipIteration = data_in.data;

    rospy.loginfo(rospy.get_caller_id() + " Tactip iteration: %d", tactipIteration)


def tactipSensorMain(sys):
    global tactipReady

    rospy.init_node('tactip_sensor');

    tactipPub = rospy.Publisher('tactip_in', String, queue_size = 1)
    rospy.Subscriber('tactip_out', String, controlInCallback);

    rospy.Subscriber('process_arguments', processArguments, processArgumentsCallback);
    rospy.Subscriber('tactip_iteration', Int32, tactipIterationCallback);

    rate = rospy.Rate(10);

    while not rospy.is_shutdown():
        if( tactipReady == True ):
            if( control_data == "tactip_down" ):
                rospy.loginfo("Saving images")

                # Data collection, training or classification
                file = open(folderPath + '/' + objectLabel + '/' + 'sample_' + str(tactipIteration), "w");
                file.write(folderPath + '/' + objectLabel + '/' + 'sample_' + str(tactipIteration));
                file.close();
                
                outputControlString = "data_collected";
            
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

