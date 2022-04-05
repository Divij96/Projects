/*
  environment_generator_node

  reads the avalanche corners from the src/environment_generator/config/environment_params.yaml file

  publishes the avalanche message for the waypoint_publisher_node
  publishes the markers for rviz

  creates the avalanche message from the yaml file. Pretty straight forward.

  written by Nicholas
  */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <algorithm>
#include <random>
#include <visualization_msgs/Marker.h>



class environmentPublisherNode{

  ros::NodeHandle nh;

  ros::Publisher env_pub;
  ros::Publisher cornerMarker_pub;


  ros::Timer timer;
  double hz;

public:
  environmentPublisherNode(){

      hz = 2;

      env_pub = nh.advertise<geometry_msgs::PoseArray>("avalanche", 10);
      cornerMarker_pub = nh.advertise<visualization_msgs::Marker>("corner_markers", 0);

      timer = nh.createTimer(ros::Rate(hz), &environmentPublisherNode::generateEnv, this);

  }



  void generateEnv(const ros::TimerEvent& t) {

    Eigen::Vector3d A;
    Eigen::Vector3d B;
    Eigen::Vector3d C;
    
    nh.getParam("/transmitter_beacon/avalanche/cornerA/x", A[0]);
    nh.getParam("/transmitter_beacon/avalanche/cornerA/y", A[1]);
    nh.getParam("/transmitter_beacon/avalanche/cornerA/z", A[2]);

    nh.getParam("/transmitter_beacon/avalanche/cornerB/x", B[0]);
    nh.getParam("/transmitter_beacon/avalanche/cornerB/y", B[1]);
    nh.getParam("/transmitter_beacon/avalanche/cornerB/z", B[2]);

    nh.getParam("/transmitter_beacon/avalanche/cornerC/x", C[0]);
    nh.getParam("/transmitter_beacon/avalanche/cornerC/y", C[1]);
    nh.getParam("/transmitter_beacon/avalanche/cornerC/z", C[2]);

/*     Eigen::Vector3d A(41, 12.2, 40.0);
    Eigen::Vector3d B(6, 17, 20);
    Eigen::Vector3d C(3.6, -1.5, 20); */
    Eigen::Vector3d e1 = (C - B).normalized();
    Eigen::Vector3d e2 = (B - A).normalized();
    Eigen::Vector3d D = A + (C - B) - 2 * (B - A).dot(e1) * e1;
    visualization_msgs::Marker cornerzz[4];

    for (int i = 0; i < 4; i++) {
      cornerzz[i].header.frame_id = "world";
      cornerzz[i].header.stamp = ros::Time();
      cornerzz[i].id = i;
      cornerzz[i].pose.orientation.x = 1;
      cornerzz[i].pose.orientation.y = 1;
      cornerzz[i].pose.orientation.z = 1;
      cornerzz[i].pose.orientation.w = 1;
      cornerzz[i].scale.x = 5;
      cornerzz[i].scale.y = 5;
      cornerzz[i].scale.z = 5;
      cornerzz[i].color.a = 1.0; // Don't forget to set the alpha!
      cornerzz[i].color.r = 0.0;
      cornerzz[i].color.g = 1.0;
      cornerzz[i].color.b = 0.0;
      cornerzz[i].type = visualization_msgs::Marker::CUBE;
      cornerzz[i].action = visualization_msgs::Marker::ADD;
    }


    geometry_msgs::PoseArray avalanche;
    avalanche.header.stamp = ros::Time::now();
    avalanche.header.frame_id = "map";
    geometry_msgs::Pose p;
    p.position.x = A[0];
    p.position.y = A[1];
    p.position.z = A[2];
    avalanche.poses.push_back(p);

    cornerzz[0].pose = p;
    p.position.x = B[0];
    p.position.y = B[1];
    p.position.z = B[2];
    avalanche.poses.push_back(p);
    cornerzz[1].pose = p;
    p.position.x = C[0];
    p.position.y = C[1];
    p.position.z = C[2];
    avalanche.poses.push_back(p);
    cornerzz[2].pose = p;
    p.position.x = D[0];
    p.position.y = D[1];
    p.position.z = D[2];
    avalanche.poses.push_back(p);
    cornerzz[3].pose = p;

    env_pub.publish(avalanche);

    cornerMarker_pub.publish(cornerzz[0]);
    cornerMarker_pub.publish(cornerzz[1]);
    cornerMarker_pub.publish(cornerzz[2]);
    cornerMarker_pub.publish(cornerzz[3]);

  }
};



int main(int argc, char* argv[])
{
    std::cout << "envPub running ..." << std::endl;
    ros::init(argc, argv, "enbPubNode");
    environmentPublisherNode envPN;
    ros::spin();
}
