// ==========================================
//  Title:  transmitterNode
//  Author: Benedikt Fischhaber
//  Date:   21 MAR 2022
// ==========================================
//
//  Description: Reads avalanche dimensions
//  from parameter server, creates random
//  positions and sets orientation of trans-
//  mitters. Pose of transmitters is published
//  as marker and as transmitter_array message.

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <transmitters/transmitter.h>
#include <transmitters/transmitter_array.h>
#include <tf2/LinearMath/Quaternion.h>
#include "eigen3/Eigen/Geometry"
#include <vector>
#include <random>
using namespace std;

#define PI M_PI


class transmitterNode {
    ros::NodeHandle nh;

    struct beacon_transmitter {
        int id;
		Eigen::Vector3d p;
		tf2::Quaternion q;
    };
    
    vector<beacon_transmitter> beacons;

	private: ros::Timer timer;
	private: ros::Publisher _transmitter_pose_pub;
    private: ros::Publisher _transmitter_marker_pub;

    double hz;

    transmitters::transmitter cur_transmitter;
    transmitters::transmitter_array msg;
    visualization_msgs::Marker cur_marker;
    visualization_msgs::MarkerArray marker_msg;

    int beacon_num;

    public: transmitterNode():hz(10) {

        nh.getParam("/transmitter_beacon/numberOfBeacons", beacon_num);

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

        Eigen::Vector3d e1 = (C - B);
        Eigen::Vector3d e2 = (B - A);

        random_device rd; // random number generation
        mt19937 gen(rd());
        uniform_real_distribution<> pos_range(0.1, 0.9);
        uniform_real_distribution<> rot_range(0, 2*M_PI);

        beacon_transmitter temp_trans;

        for (int i = 1; i <= beacon_num; ++i) {
            temp_trans.id = i;
            temp_trans.p = A + pos_range(gen)*e2 + pos_range(gen)*e1;
            //include that for random orientation of transmitters
            // temp_trans.q.setRPY(rot_range(gen), rot_range(gen), rot_range(gen));
            // temp_trans.q.normalize();
            temp_trans.q = {0, 0, 0, 1};
            beacons.push_back(temp_trans);
        }

        create_msg();
        _transmitter_pose_pub = nh.advertise<transmitters::transmitter_array>("transmitter/pose", 3);
        _transmitter_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("transmitter/marker", 3);
        
        timer = nh.createTimer(ros::Rate(hz), &transmitterNode::transmitter_loop, this);
    }

    public: void create_marker_msg(){
        // creates MarkerArray message for rviz
        int i = 0;
        marker_msg.markers.resize(beacon_num);

        for (const auto &beacon : beacons){
            cur_marker.header.frame_id = "world";
            cur_marker.header.stamp = ros::Time::now();
            cur_marker.id = beacon.id;
            cur_marker.ns = "beacon_visualisation_transmitter_markers";
            cur_marker.type = visualization_msgs::Marker::CUBE;
            cur_marker.action = visualization_msgs::Marker::ADD;
            cur_marker.pose.position.x = beacon.p[0];
            cur_marker.pose.position.y = beacon.p[1];
            cur_marker.pose.position.z = beacon.p[2];
            cur_marker.pose.orientation.x = beacon.q[0];
            cur_marker.pose.orientation.y = beacon.q[1];
            cur_marker.pose.orientation.z = beacon.q[2];
            cur_marker.pose.orientation.w = beacon.q[3];
            cur_marker.scale.x = 2.0;
            cur_marker.scale.y = 1.0;
            cur_marker.scale.z = 1.0;
            cur_marker.color.r = 1.0f;
            cur_marker.color.g = 0.0f;
            cur_marker.color.b = 0.0f;
            cur_marker.color.a = 1.0;
            cur_marker.lifetime = ros::Duration();
            marker_msg.markers[i] = cur_marker;
            ++i;
        };

    }

    public: void create_msg(){
        // creates transmitter array for message
        int i = 0;
        msg.transmitter_arr.resize(beacon_num);

        for (const auto &beacon : beacons){
            cur_transmitter.beacon_id = beacon.id;
            cur_transmitter.position_vector.x = beacon.p[0];
            cur_transmitter.position_vector.y = beacon.p[1];
            cur_transmitter.position_vector.z = beacon.p[2];
            cur_transmitter.rotation_quaternion.x = beacon.q[0];
            cur_transmitter.rotation_quaternion.y = beacon.q[1];
            cur_transmitter.rotation_quaternion.z = beacon.q[2];
            cur_transmitter.rotation_quaternion.w = beacon.q[3];
            msg.transmitter_arr[i] = cur_transmitter;
            ++i;
        }
    }

    public: void transmitter_loop( const ros::TimerEvent& t ) {
        _transmitter_pose_pub.publish(msg);
        create_marker_msg();
        _transmitter_marker_pub.publish(marker_msg);
    }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "transmitter_node");
    ROS_INFO_NAMED("transmitter_node", "Transmitter started!");
    transmitterNode n;
    while (ros::ok()) {    
        ros::spin();
    }
    return 0;
}
