/*
 * Advanced planner node
  sets the maximum acceleration and speed for the trajectory and instantiates the planner

  written by Maximilian
 */

#include  "ros/ros.h"
#include <planner.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "advanced_planner");
    ros::NodeHandle n;

    // instantiate basic planner
    BasicPlanner planner(n);

    //set maximum acceleration and speed of the drone
    planner.setMaxSpeed(50);
    planner.setMaxAcc(50);

    std::cout << "[ planner_node ] started!" << std::endl;
    ros::spin();
    return 0;
}
