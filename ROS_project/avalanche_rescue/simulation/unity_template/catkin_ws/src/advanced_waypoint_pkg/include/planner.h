#ifndef ADVANCED_WAYPOINT_PKG_PLANNER_H
#define ADVANCED_WAYPOINT_PKG_PLANNER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <std_msgs/Int32.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void waypointCallback(const nav_msgs::Odometry waypoint);
    void currentStateCallback(const nav_msgs::Odometry::ConstPtr& current_state);

    void numberOfWaypointsCallback(const std_msgs::Int32 number);

    void setMaxSpeed(double max_v);

    void setMaxAcc(double max_a);

    void setNumberOfRecentWaypoints(int number_of_recent_waypoints);

    bool planTrajectory(const Eigen::VectorXd& new_pos,
                        mav_trajectory_generation::Trajectory* trajectory);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_waypoint;
    ros::Subscriber sub_numberOfWaypoints;

    ros::NodeHandle& nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;
    int number_of_recent_waypoints_;
    Eigen::Matrix3Xd waypoint_matrix;
    int waypointCounter;
    int numberOfWaypoints;

};

#endif // ADVANCED_WAYPOINT_PKG_PLANNER_H
