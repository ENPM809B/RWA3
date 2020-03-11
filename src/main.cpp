#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <iostream>
#include "competition.h"

void StartCompetition(ros::NodeHandle & node) {
    ROS_INFO("Competition function.");
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv);  // Call the start Service.
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition started!");
    }
}

void EndCompetition(ros::NodeHandle &node) {
    ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be End...");
        start_client.waitForExistence();
        ROS_INFO("Competition  now Ended.");
    }
    ROS_INFO("Requesting competition End...");
    std_srvs::Trigger srv;
    start_client.call(srv);
    if (!srv.response.success) {
        ROS_ERROR_STREAM(
                "Failed to End the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition Ended!");
    }
}

int main(int argc, char **argv) {
    ROS_INFO("Starting main function");
    ros::init(argc, argv, "ariac_manager_node");
    ros::NodeHandle node;
    // AriacOrderManager manager;
    Competition comp(node);

    ros::Subscriber current_score_subscriber = node.subscribe(
            "/ariac/current_score", 10,
            &Competition::current_score_callback, &comp);

    // Subscribe to the '/ariac/competition_state' topic.
    ros::Subscriber competition_state_subscriber = node.subscribe(
            "/ariac/competition_state", 10,
            &Competition::competition_state_callback, &comp);
    ros::Subscriber break_beam_subscriber = node.subscribe(
              "/ariac/break_beam_1_change", 10,
              &Competition::break_beam_callback, &comp);

    ros::Publisher arm1_cmd_pub = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
            // publish at at least 5 Hz, or else ur10 switches back to Position mode and holds position
    ros::Rate loop_rate(10);


    trajectory_msgs::JointTrajectory traj;

    traj.joint_names.push_back("linear_arm_actuator_joint");
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    traj.points.resize(1);

    ROS_INFO("Setup complete.");


    StartCompetition(node);
    ros::Duration(2.0).sleep();
    //manager.SetScannedProducts();
    // manager.ExecuteOrder();
    // while(ros::ok()) {

    //--Only one point
    int ind{0};
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
    traj.points[ind].time_from_start = ros::Duration(3.0);


      traj.header.stamp = ros::Time::now() + ros::Duration();
      arm1_cmd_pub.publish(traj);
      ros::spin();
      loop_rate.sleep();
    // }
    // ros::spin();  // This executes callbacks on new data until ctrl-c.

    return 0;
}
