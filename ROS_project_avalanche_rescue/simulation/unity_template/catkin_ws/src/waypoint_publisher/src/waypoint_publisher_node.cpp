/*
  waypoint_publisher_node

  subscribes to current_state of the drone send by unity_state
  subscribes to the avalanche message send by the environment_generator

  publishes the numberOfWaypoints message for the planner
  publishes the waypoint message for the planner

  Generates the search pattern for the given corners in the avalanche message.
  The search pattern is calculated with the following parameters:
  1. the sensor range
  2. the overlap (safety margin)
  3. the stepSize (densitiy of waypoints along the searchpath)
  The final search pattern is stored as a list of waypoints.

  The number of waypoints of the final search pattern is then send to the planner followed by the waypoints in the list.

  After the inital generation and publishing of the search pattern waypoints the waypoint_publisher stays idle for the rest of the simulation.

  Somewhere here the easter egg is implemented ;)

  written by Nicholas and Maximilian
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <beacon/receiver.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <std_msgs/Int32.h>


class waypointPublisherNode{

  //ros subscriber and publisher
  ros::NodeHandle nh;
  ros::Publisher waypoint_pub, numberOfWaypoints_pub;
  ros::Subscriber avalanche_sub, current_state_sub;
  // ros::Subscriber beacon_status_sub;

  ros::Timer timer;
  double hz;

  int counter;
  float stepSize;
  int numberofWaypoints;

  bool searchAktiv_;
  bool unityStarted = false;
  bool trajectoryPublished;

  bool receivedAvalanche;
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<Eigen::Vector3d> waypointsBackup;

  nav_msgs::Odometry waypoint;
  nav_msgs::Odometry oldMessage;

  std_msgs::Int32 numberOfWaypointsMsg;

  Eigen::Vector3d x;
  Eigen::Vector3d v;

  Eigen::Vector3d lastPositionDuringTrajectory;



public:
  waypointPublisherNode(){

      hz = 10;
      counter = 0;

      // Stepsize
      stepSize = 30.f;

      //waypoints in the list
      numberofWaypoints = 0;

      searchAktiv_ = false;
      receivedAvalanche = false;
      trajectoryPublished = false;

      waypoint_pub = nh.advertise<nav_msgs::Odometry>("waypoint", 1000);
      numberOfWaypoints_pub = nh.advertise<std_msgs::Int32>("numberOfWaypoints", 10);
      //beacon_status_sub = nh.subscribe("receiver/signal", 100, &waypointPublisherNode::onBeaconMsg, this);
      avalanche_sub = nh.subscribe("avalanche", 10, &waypointPublisherNode::onAvalancheMsg, this);
      current_state_sub = nh.subscribe("current_state", 10, &waypointPublisherNode::onCurrentStateMsg, this);

      timer = nh.createTimer(ros::Rate(hz), &waypointPublisherNode::publisherLoop, this);

  }

  // check if receiver found beacon
  /*void onBeaconMsg(const beacon::receiver beacon_msg){

    if(beacon_msg.distance < 0 ){
      searchAktiv_ = false;
    } else if(beacon_msg.distance < 3500) {
      searchAktiv_ = true;
    }
    //std::cout << "[ waypoint_publisher ] searchAktiv set to " << searchAktiv_ << std::endl;

  }*/

  // Callback for avalanche message --> generates search pattern
  void onAvalancheMsg(const geometry_msgs::PoseArray avalanche_msg){

    if (receivedAvalanche) {
      return;   // do nothing if avalanche coordinates already received
    }
    std::cout << "[ waypoint_publisher ] Received new avalanche message" << std::endl;
    receivedAvalanche = true;

    int startCorner = 0;
    int sensorRange = 30;
    int overlap = 5;
    int effRng = sensorRange - overlap;

    int offset_z = 20;  // necessary offset for the drone to have enough groundclearance and not to crash into the snow

    // Generate pattern
    Eigen::Vector3d avalancheCorners[4];
    for (int i = 0; i < 4; i++) {
      avalancheCorners[i][0] = avalanche_msg.poses[i].position.x;
      avalancheCorners[i][1] = avalanche_msg.poses[i].position.y;
      avalancheCorners[i][2] = avalanche_msg.poses[i].position.z + offset_z;
    }

    Eigen::Vector3d vec_bc = (avalancheCorners[2] - avalancheCorners[1]).normalized();
    Eigen::Vector3d vec_ab = (avalancheCorners[1] - avalancheCorners[0]).normalized();
    //std::cout << "vec_ab: " << vec_ab << std::endl;
    //std::cout << "vec_bc: " << vec_bc << std::endl;

    // conditions for trajectory finish
    bool visitedC = false;
    bool visitedB = false;
    bool swapDir = false;
    bool horizontal = true;
    float distToGo = 0;

    Eigen::Vector3d origin = Eigen::Vector3d(0,0,2);
    Eigen::Vector3d wp = origin;
    waypoints.push_back(origin);
    numberofWaypoints++;

    // fly to first corner
    distToGo = L2Distance(avalancheCorners[2], origin);
    Eigen::Vector3d origToStart = (avalancheCorners[2] - origin).normalized();
    for (int i = 0; i < std::floor(distToGo / stepSize); i++) {
      wp = wp + origToStart * stepSize;
      waypoints.push_back(wp);
      numberofWaypoints ++;
    }

    while (!visitedC and !visitedB) {
      if (horizontal) {
        distToGo = std::max(L2Distance(avalancheCorners[1], avalancheCorners[2]), L2Distance(avalancheCorners[1], avalancheCorners[2]));
        vec_bc(0) *= -1;
        vec_bc(1) *= -1;
        for (int i = 0; i < std::floor(distToGo / stepSize); i++) {
            wp = wp + vec_bc * stepSize;
            waypoints.push_back(wp);
            numberofWaypoints ++;
          }
      } else {
        distToGo = 2*sensorRange - overlap;
        for (int i = 0; i < std::floor(distToGo / stepSize); i++) {
          wp = wp + -vec_ab * stepSize;
          waypoints.push_back(wp);
          numberofWaypoints ++;
        }
      }
      //std::cout << "Debug: " << distToGo << std::endl;
      horizontal = !horizontal;
      if (L2Distance(wp, avalancheCorners[0]) < effRng) {
        visitedC = true;
        //std::cout << "C check" << std::endl;
      }
      if (L2Distance(wp, avalancheCorners[3]) < effRng) {
        visitedB = true;
        //std::cout << "B check" << std::endl;
      }

    }

    // Easter egg
    wp << 200, 50, 70;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 80, 120;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 70, 120;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 50, 100;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 30, 120;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 20, 120;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 48, 70;
    waypoints.push_back(wp);
    numberofWaypoints ++;
    wp << 200, 50, 70;
    waypoints.push_back(wp);
    numberofWaypoints ++;

    waypoints.push_back(origin);
    numberofWaypoints ++;

    std::reverse(waypoints.begin(), waypoints.end());
    std::cout << "[ waypoint_publisher ] generation of search pattern finished " << std::endl;
    std::cout << "[ waypoint_publisher ] Number of waypoints " << numberofWaypoints << std::endl;

    waypointsBackup = waypoints;

    //publish numberOfWaypoints to planner
    numberOfWaypointsMsg.data = numberofWaypoints;
    numberOfWaypoints_pub.publish(numberOfWaypointsMsg);

  }

  //Callback for current state
  void onCurrentStateMsg(const nav_msgs::Odometry& cur_state) {
    //std::cout << "Got state msg" << std::endl;
    unityStarted = true;
    x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;

  }

  //Loop for waypoint publishing
  void publisherLoop(const ros::TimerEvent& t){

    /*if(searchAktiv_){
      return;
    } else */if(!receivedAvalanche or !unityStarted){
      std::cout << "[ waypoint_publisher ] No avalanche or unity not started!" << std::endl;
      return;
    }

    //only executed once
    if(!trajectoryPublished){
      for(int i = 0; i < numberofWaypoints; i++){
        waypoint.pose.pose.position.x = waypoints.back()[0];
        waypoint.pose.pose.position.y = waypoints.back()[1];
        waypoint.pose.pose.position.z = waypoints.back()[2];
        waypoint_pub.publish(waypoint);
        //std::cout << "[ waypoint_publisher ] Published new waypoint:" << waypoints.back()[0] << "  " << waypoints.back()[1] << "  "<< waypoints.back()[2] << std::endl;
        waypoints.pop_back();
        if (waypoints.empty()) {
          break;
        }
      }
      trajectoryPublished = true;
    }
}

  // function to calculate the L2 distance
  float L2Distance (Eigen::Vector3d a, Eigen::Vector3d b) {
    float dist = pow(a[0] - b[0],2);
    dist += pow(a[1] - b[1],2);
    dist += pow(a[2] - b[2],2);
    dist = sqrt(dist);
    //std::cout << "Debug: " << dist << std::endl;
    return dist;
  }
};



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "talker");
    waypointPublisherNode wPN;
    ros::spin();
}
