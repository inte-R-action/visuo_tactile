/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 4-March-2020
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>

using namespace std;


string objectString = "";
bool robotMove = false;

void robotMoveCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);

//  if( msg->data != "" )
//      robotMove = true;
//  else
//      robotMove = false;
 
 	objectString.clear();

    objectString = msg->data; 

    cout << "Robot move: " << robotMove << endl; 
    cout << "Object: " << objectString << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hri_static_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


	ros::Subscriber subRobotPosition = node_handle.subscribe("RobotMove", 1000, robotMoveCallback);


    // Setup
    // ^^^^^
    //
    // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Tactip Grid Exploration - v 0.1.0", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


    // Now, we call the planner to compute the plan and visualize it.
    // The plan variable contains the movements that the robot will perform to move
    // from one point to another
    moveit::planning_interface::MoveGroupInterface::Plan plan;


    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(0.10);
    move_group.setMaxAccelerationScalingFactor(0.10);


    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to a 'home' position
    // Begin


    std::map<std::string, double> targetJoints;

    while( true )
    {
    // wait position
        cout << "waiting for command" << endl;

            targetJoints.clear();

            if( objectString == "bring_side_1" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = -31.85*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -43.48*3.1416/180;
	            targetJoints["elbow_joint"] = 55.18*3.1416/180;
	            targetJoints["wrist_1_joint"] = -103.14*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.87*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);

	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(10);
            }
            else if( objectString == "bring_side_2" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = -19.00*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -43.48*3.1416/180;
	            targetJoints["elbow_joint"] = 55.18*3.1416/180;
	            targetJoints["wrist_1_joint"] = -103.14*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.87*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);

	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(10);
            }
            else if( objectString == "bring_side_3" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = -2.74*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -43.48*3.1416/180;
	            targetJoints["elbow_joint"] = 55.18*3.1416/180;
	            targetJoints["wrist_1_joint"] = -103.14*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.87*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);

	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(10);
            }
            else if( objectString == "bring_side_4" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = 12.00*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -43.48*3.1416/180;
	            targetJoints["elbow_joint"] = 55.18*3.1416/180;
	            targetJoints["wrist_1_joint"] = -103.14*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.87*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);
	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(10);
            }
            else if( objectString == "take_box" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = 46.68*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -67.95*3.1416/180;
	            targetJoints["elbow_joint"] = 108.8*3.1416/180;
	            targetJoints["wrist_1_joint"] = -125.00*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.0*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);
	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(10);
            }
            else // if( objectString == "take_box" )
			{            
	            targetJoints.clear();
	            targetJoints["shoulder_pan_joint"] = -11.75*3.1416/180;	// (deg*PI/180)
	            targetJoints["shoulder_lift_joint"] = -83.80*3.1416/180;
	            targetJoints["elbow_joint"] = 47.90*3.1416/180;
	            targetJoints["wrist_1_joint"] = -125.0*3.1416/180;
	            targetJoints["wrist_2_joint"] = -90.0*3.1416/180;
	            targetJoints["wrist_3_joint"] = 0.26*3.1416/180;

	            move_group.setJointValueTarget(targetJoints);
	            bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	            ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

	            move_group.execute(plan);
	            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

//	            sleep(1);
            }
            
    }

    ros::shutdown();
    return 0;
}
