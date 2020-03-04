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
#include <iostream>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tactip_grid_exploration");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

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
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
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


    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the tactile exploration");


    // Definition of parameters for planning Cartesian paths
    double PLAN_TIME_ = 50.0;
    bool MOVEIT_REPLAN_ = true;
    double CART_STEP_SIZE_ = 0.01;

    // The documentation says that 'jump threshold' should be set to 0.0 for simulation and different from 0.0 for the real robot
    // However, it seems that values grater than 0.0 doesn't allow the robot to move
    double CART_JUMP_THRESH_ = 0.0;	
    bool AVOID_COLLISIONS_ = true;
    double MAX_VEL_SCALE_FACTOR = 0.3;

    move_group.setPlanningTime(PLAN_TIME_);
    move_group.allowReplanning(MOVEIT_REPLAN_);
    move_group.setMaxVelocityScalingFactor(MAX_VEL_SCALE_FACTOR);


    // Now, we call the planner to compute the plan and visualize it.
    // The plan variable contains the movements that the robot will perform to move
    // from one point to another
    moveit::planning_interface::MoveGroupInterface::Plan plan;


    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to a 'home' position
    // Begin

    std::map<std::string, double> targetJoints;
    targetJoints["shoulder_pan_joint"] = -0.0;
    targetJoints["shoulder_lift_joint"] = -1.56;
    targetJoints["elbow_joint"] = -0.02;
    targetJoints["wrist_1_joint"] = -1.57;
    targetJoints["wrist_2_joint"] = -1.51;
    targetJoints["wrist_3_joint"] = 0.1;  

    move_group.setJointValueTarget(targetJoints);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing HOME position plan (%.2f%% acheived)",success * 100.0);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // End
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to a pre-start position
    // Begin

    targetJoints.clear();

    targetJoints["shoulder_pan_joint"] = -0.0;
    targetJoints["shoulder_lift_joint"] = -0.1;
    targetJoints["elbow_joint"] = 0.1;
    targetJoints["wrist_1_joint"] = -1.56;
    targetJoints["wrist_2_joint"] = -1.56;
    targetJoints["wrist_3_joint"] = 0.01;

    move_group.setJointValueTarget(targetJoints);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // End
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot to the start position for the exploration of the grid
    // Begin

    robot_state::RobotStatePtr kinematic_state = robot_state::RobotStatePtr(move_group.getCurrentState());

    // Gets the current pose of the robot
    geometry_msgs::Pose target_pose1 = move_group.getCurrentPose().pose;


    // Predefined distances in meters to move the robot
    double UP_MOVE = 0.1;    // 0.1m or 10cm 
    double DOWN_MOVE = 0.1;
    double LEFT_MOVE = 0.02; // 0.02m or 2cm
    double RIGHT_MOVE = 0.02;
    double BACKWARD_MOVE = 0.02;
    double FORWARD_MOVE = 0.02;


    // Prints initial pose (position and orientation) of the end-effector
    std::cout << "Pose" << std::endl;
    std::cout << "Position [x, y, z]: " << target_pose1.position.x << ", " << target_pose1.position.y << ", " << target_pose1.position.z << std::endl;
    std::cout << "Orientation [x, y, z, w]: " << target_pose1.orientation.x << ", " << target_pose1.orientation.y << ", " << target_pose1.orientation.z << ", " << target_pose1.orientation.w << std::endl;


    // Vector to store the waypoints for the planning process
    std::vector<geometry_msgs::Pose> waypoints;
    // Stores the first target pose or waypoint
    waypoints.push_back(target_pose1);

    // Decrements current X position by BACKWARD_MOVE*3
    target_pose1.position.x -= BACKWARD_MOVE*3;
    // Increments current Z position by UP_MOVE
    target_pose1.position.z += UP_MOVE;
    // Stores the first target pose or waypoint
    waypoints.push_back(target_pose1);
    // Set the final pose target of the robot
    move_group.setPoseTarget(target_pose1);


    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    // The trajectory is computed and store in trajectory_
    moveit_msgs::RobotTrajectory trajectory_;
    double fraction = move_group.computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);
    robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(), "manipulator");

    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);

    ROS_INFO_STREAM("Pose reference frame: " << move_group.getPoseReferenceFrame ());

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_);

    ROS_INFO("Visualizing START pose plan for exploration (%.2f%% acheived)",fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // Finally plan and execute the trajectory
    plan.trajectory_ = trajectory_;

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // Get the current kinemtic state of the robot
    kinematic_state = move_group.getCurrentState();

    // Get current pose and store it as the initial pose for iterative exploration
    geometry_msgs::Pose initialGridPose = move_group.getCurrentPose().pose;


    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code perform the iterative exploration over a grid with the TacTip
    // Begin

    // This exploration is performed on a 5x5 grid
    for( int iterJ = 0; iterJ < 5; iterJ++ )
    {
        target_pose1 = move_group.getCurrentPose().pose;

        if( iterJ > 0 )
        {
            target_pose1.position.x = initialGridPose.position.x;
            target_pose1.position.y += RIGHT_MOVE;
        }

        for( int iterI = 0; iterI < 5; iterI++ )
        {
            waypoints.clear();
            waypoints.push_back(target_pose1);  // start pose

            target_pose1.position.z -= DOWN_MOVE;
            waypoints.push_back(target_pose1);  // move end-effector down

            target_pose1.position.z += UP_MOVE;
            waypoints.push_back(target_pose1);  // move end-effector up

            target_pose1.position.x -= BACKWARD_MOVE;
            waypoints.push_back(target_pose1);  // move end-effector back

            move_group.setPoseTarget(target_pose1);

            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

            fraction = move_group.computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);

            rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);

            ROS_INFO_STREAM("Pose reference frame: " << move_group.getPoseReferenceFrame ());

            // Thrid create a IterativeParabolicTimeParameterization object
            success = iptp.computeTimeStamps(rt);
            ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");


            // Get RobotTrajectory_msg from RobotTrajectory
            rt.getRobotTrajectoryMsg(trajectory_);


            ROS_INFO("Visualizing Iteration [%d, %d] and GRID position [%f, %f] - plan (%.2f%% acheived)",iterJ, iterI, target_pose1.position.x, target_pose1.position.y, fraction * 100.0);

            // Visualize the plan in RViz
            visual_tools.deleteAllMarkers();
            visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
            visual_tools.trigger();

            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

            // Finally plan and execute the trajectory
            plan.trajectory_ = trajectory_;

            // IMPORTANT!!!!!     
            // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
            // The current version of this program still does NOT work properly with the 'move' method
            // Using the 'move' method in this program might generate unexected and dangerous robot movements
            move_group.execute(plan);
            //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

            // Get the current kinemtic state of the robot
            kinematic_state = move_group.getCurrentState();
        }
    }

    // End of iteration
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // The following code moves the robot back to the START position

    waypoints.clear();
    waypoints.push_back(initialGridPose);  // up

    move_group.setPoseTarget(initialGridPose);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualise the planned robot movement");

    fraction = move_group.computeCartesianPath(waypoints,CART_STEP_SIZE_,CART_JUMP_THRESH_,trajectory_,AVOID_COLLISIONS_);

    rt.setRobotTrajectoryMsg(*kinematic_state, trajectory_);

    ROS_INFO_STREAM("Pose reference frame: " << move_group.getPoseReferenceFrame ());

    // Thrid create a IterativeParabolicTimeParameterization object
    success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");


    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_);


    ROS_INFO("Visualizing START pose plan for exploration (%.2f%% acheived)",fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot movement");

    // Finally plan and execute the trajectory
    plan.trajectory_ = trajectory_;

    // IMPORTANT!!!!!     
    // Use always the 'execute' method to move the robot in the simulation environment and the real robot!
    // The current version of this program still does NOT work properly with the 'move' method
    // Using the 'move' method in this program might generate unexected and dangerous robot movements
    move_group.execute(plan);
    //    move_group.move(); // DO NOT USE THIS METHOD IN THIS PROGRAM

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with the next action");

    // Get the current kinemtic state of the robot
    kinematic_state = move_group.getCurrentState();

    // End
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


    ros::shutdown();
    return 0;
}
