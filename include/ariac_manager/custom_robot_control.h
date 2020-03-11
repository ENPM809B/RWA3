#ifndef SRC_CUSOM_ROBOT_CONTROL_H
#define SRC_CUSTOM_ROBOT_CONTROL_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <trajectory_msgs/JointTrajectory.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotControl{

public:

    RobotControl();
    ~RobotControl();
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void customPartPick();
    void moveToInitialLocation();

private:
    ros::NodeHandle robot_controller_nh_;
	ros::ServiceClient gripper_client_;
	ros::NodeHandle gripper_nh_;
	ros::Subscriber gripper_subscriber_;
	ros::Publisher arm1_cmd_pub_ = robot_controller_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

	geometry_msgs::Pose target_pose_;

	osrf_gear::VacuumGripperControl gripper_service_;
	osrf_gear::VacuumGripperState gripper_status_;

	std::string object;
	int counter_;
	bool gripper_state_;
};

#endif
