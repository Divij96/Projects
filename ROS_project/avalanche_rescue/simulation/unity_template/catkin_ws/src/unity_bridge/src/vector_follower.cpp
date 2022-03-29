/*
  calculated the movement of the drone when vector-following

  subscribes to the current_state of the drone which is sent by the unity_state
  subscribes to the receiver_signal send by the receiver_node
  gets the corners of the avalanche as defined in the environment_generator pkg using rosparam

  calculates the desired_state of the drone when vector-following

  publishes the desired_state for the controller

  Written by Samuel Zeitler
*/





#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <beacon/receiver.h>
#include <algorithm>


class vector_follower {

    tf::Vector3 origin_tf;
    Eigen::Vector3d origin_eigen;
    ros::Timer timer;
    ros::Publisher desired_state_pub;
    ros::Subscriber current_state_sub;
    ros::Subscriber vector_sub;

    double hz;
    double x, y, z;
    double step;
    double distance;
    double offset_to_plane;

    Eigen::Vector4d plane_equ_koeff;

public:

    vector_follower() {

        hz = 1000;
        
        //Initial step size for the vector to follow
        step = 1;
        
        //Initial vector to follow
        x = 0;
        y = 0;
        z = 1;
        
        //The drone mustn't go further down from the avalanche plane than this
        offset_to_plane = 3; // [m]

        ros::NodeHandle n;
        desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state",100);
        current_state_sub = n.subscribe("current_state", 1, &vector_follower::onCurrentState, this);
        vector_sub = n.subscribe("receiver/signal", 100, &vector_follower::onVectorMsg, this);
        timer = n.createTimer(ros::Rate(hz), &vector_follower::vectorLoop, this);

        Eigen::Vector3d A;
        Eigen::Vector3d B;
        Eigen::Vector3d C;
        Eigen::Vector4d plane_eqauation_koeff;

        //Get the corners of the avalanche
        n.getParam("/transmitter_beacon/avalanche/cornerA/x", A[0]);
        n.getParam("/transmitter_beacon/avalanche/cornerA/y", A[1]);
        n.getParam("/transmitter_beacon/avalanche/cornerA/z", A[2]);

        n.getParam("/transmitter_beacon/avalanche/cornerB/x", B[0]);
        n.getParam("/transmitter_beacon/avalanche/cornerB/y", B[1]);
        n.getParam("/transmitter_beacon/avalanche/cornerB/z", B[2]);

        n.getParam("/transmitter_beacon/avalanche/cornerC/x", C[0]);
        n.getParam("/transmitter_beacon/avalanche/cornerC/y", C[1]);
        n.getParam("/transmitter_beacon/avalanche/cornerC/z", C[2]);

        //Calculate the plane equation of the avalanche plane
        plane_equ_koeff = equation_plane(A[0],A[1],A[2],B[0],B[1],B[2],C[0],C[1],C[2]);

    };
    
    //Calculate the coefficients (a,b,c,d) for the plane equation ax + by + cz + d = 0 
    //Using 3 given points of the plane
    Eigen::Vector4d equation_plane(float x1, float y1,
                        float z1, float x2,
                        float y2, float z2,
                        float x3, float y3, float z3)
    {
        float a1 = x2 - x1;
        float b1 = y2 - y1;
        float c1 = z2 - z1;
        float a2 = x3 - x1;
        float b2 = y3 - y1;
        float c2 = z3 - z1;
        float a = b1 * c2 - b2 * c1;
        float b = a2 * c1 - a1 * c2;
        float c = a1 * b2 - b1 * a2;
        float d = (- a * x1 - b * y1 - c * z1);

        Eigen::Vector4d plane_equ_koeff;
        plane_equ_koeff[0] = a;
        plane_equ_koeff[1] = b;
        plane_equ_koeff[2] = c;
        plane_equ_koeff[3] = d;

        return plane_equ_koeff;

    }

    //Calculate and publish the desired_state message for the controller
    void vectorLoop(const ros::TimerEvent &t) {

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        desired_pose.setOrigin(
                origin_tf + tf::Vector3(x * step, y * step, z * step)

        );

        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z();

    };

        //Calculate the desired vector to follow
    void onVectorMsg(const beacon::receiver found_beacon){

        x = found_beacon.direction_vector.x;
        y = found_beacon.direction_vector.y;
        z = found_beacon.direction_vector.z;

        //Calculate the speed for the vector_following using the distance to the beacon
        //Drone should get slower as it approaches beacon
        step = std::max(0.5f,std::min(2.f, found_beacon.distance/500));

        double a = plane_equ_koeff[0];
        double b = plane_equ_koeff[1];
        double c = plane_equ_koeff[2];
        double d = plane_equ_koeff[3];

        double x_drone = origin_eigen[0];
        double y_drone = origin_eigen[1];
        double z_plane = (-a*x_drone-b*y_drone-d)/c;

        // make sure drone doesn't touch snow by restricting the distance the drone is allowed
        // to move down from the avalanche plane
        if(origin_eigen[2] <= z_plane - offset_to_plane){
            z = 0.001f;
        }
    };

    // get the current state for the calculation of the distance of the drone to the beacon
    void onCurrentState(const nav_msgs::Odometry cur_state) {

        origin_eigen << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        tf::vectorEigenToTF(origin_eigen, origin_tf);
    };
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vector_follower");
    vector_follower vf;
    ros::spin();
    return 0;
}
