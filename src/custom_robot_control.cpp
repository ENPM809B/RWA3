#include "custom_robot_control.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotControl::RobotControl() {

	ROS_WARN(">>>>> RobotController");

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, &RobotControl::GripperCallback, this);

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
    counter_ = 0;
}

RobotControl::~RobotControl(){}

void RobotControl::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

void RobotControl::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


void RobotControl::moveToInitialLocation(){
	ROS_INFO_STREAM("Moving robot to intial location");
    // Publish starting point
	trajectory_msgs::JointTrajectory traj;
	traj.joint_names.push_back("linear_arm_actuator_joint");
	traj.joint_names.push_back("shoulder_pan_joint");
	traj.joint_names.push_back("shoulder_lift_joint");
	traj.joint_names.push_back("elbow_joint");
	traj.joint_names.push_back("wrist_1_joint");
	traj.joint_names.push_back("wrist_2_joint");
	traj.joint_names.push_back("wrist_3_joint");

	traj.points.resize(1);

    int ind = 0;

    traj.points[ind].positions.resize(7);//7 joints
	traj.points[ind].positions[0] = 0.47;
	traj.points[ind].positions[1] = 3.24;
	traj.points[ind].positions[2] = -2.41;
	traj.points[ind].positions[3] = -1.78;
	traj.points[ind].positions[4] = 5.80;
	traj.points[ind].positions[5] = -4.71;
	traj.points[ind].positions[6] = -1.13;

	// Velocities
	traj.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j){
	 traj.points[ind].velocities[j] = 0.1;
	}

	// Reach the second point with 4 seconds after starting along the traj
	traj.points[ind].time_from_start = ros::Duration(1.0);

	traj.header.stamp = ros::Time::now() + ros::Duration();
	arm1_cmd_pub_.publish(traj);
}


void RobotControl::customPartPick() {
			// publish at at least 5 Hz, or else ur10 switches back to Position mode and holds position
	ros::Rate loop_rate(10);

	trajectory_msgs::JointTrajectory traj;
	// unsigned short ind{0};
	traj.joint_names.push_back("linear_arm_actuator_joint");
	traj.joint_names.push_back("shoulder_pan_joint");
	traj.joint_names.push_back("shoulder_lift_joint");
	traj.joint_names.push_back("elbow_joint");
	traj.joint_names.push_back("wrist_1_joint");
	traj.joint_names.push_back("wrist_2_joint");
	traj.joint_names.push_back("wrist_3_joint");

	traj.points.resize(2);

	// ros::Duration(1.0).sleep();

	//--Only one point
	int ind{0};
	traj.points[ind].positions.resize(7);//7 joints
	traj.points[ind].positions[0] = 0.47;
	traj.points[ind].positions[1] = 3.24;
	traj.points[ind].positions[2] = -2.43;
	traj.points[ind].positions[3] = -1.78;
	traj.points[ind].positions[4] = 5.80;
	traj.points[ind].positions[5] = -4.71;
	traj.points[ind].positions[6] = -1.13;

	// Velocities
	traj.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j){
	  traj.points[ind].velocities[j] = 0.1;
	}

	// Reach the second point with 4 seconds after starting along the traj
	traj.points[ind].time_from_start = ros::Duration(1.2);
	//
	traj.header.stamp = ros::Time::now() + ros::Duration();
	arm1_cmd_pub_.publish(traj);

	ind++;
	traj.points[ind].positions.resize(7);//7 joints
	traj.points[ind].positions[0] = 0.00;
	traj.points[ind].positions[1] = 3.14;
	traj.points[ind].positions[2] = -2.00;
	traj.points[ind].positions[3] = 2.14;
	traj.points[ind].positions[4] = 3.27;
	traj.points[ind].positions[5] = -1.51;
	traj.points[ind].positions[6] = -0.0;

	// Velocities
	traj.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j){
	  traj.points[ind].velocities[j] = 0.1;
	}

	// Reach the second point with 4 seconds after starting along the traj
	traj.points[ind].time_from_start = ros::Duration(2.0);

	traj.header.stamp = ros::Time::now() + ros::Duration();
	arm1_cmd_pub_.publish(traj);
	this->GripperToggle(true);
}
