/*
  planner for the trajectory generation

  subscribes to the current_state of the drone which is send by the unity_state
  subscribes to the numberOfWaypoints and the waypoint messages send by the waypoint publisher

  calculates the snap trajectory for the given number of waypoints by using the mav_trajectory_generation package

  publishes the trajectory as trajectory and the trajectory_markers for rviz_config

  Written by Samuel and Maximilian
*/


#include <planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(10),
        max_a_(2),
        number_of_recent_waypoints_(1),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {


    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for current state
    sub_odom_ = nh.subscribe("current_state", 1, &BasicPlanner::currentStateCallback, this);

    // subscriber for waypoint
    sub_waypoint = nh.subscribe("waypoint", 0 ,&BasicPlanner::waypointCallback, this);

    // subscriber for number of waypoints
    sub_numberOfWaypoints = nh.subscribe("numberOfWaypoints", 1000, &BasicPlanner::numberOfWaypointsCallback, this);

    waypointCounter = 0;
    numberOfWaypoints = 0;

}

//Callback for number of waypoints
void BasicPlanner::numberOfWaypointsCallback(const std_msgs::Int32 number){

  //std::cout << "[ planner ] Received " << number.data << " waypoints" << std::endl;
  numberOfWaypoints = number.data;

}

// Callback to get published waypoints
void BasicPlanner::waypointCallback(const nav_msgs::Odometry waypoint) {

  //if no numberOfWaypoints message was published yet
  if(numberOfWaypoints == 0){
    std::cout << "[ planner ] Received no number of waypoints" << std::endl;
    return;
  }

    //std::cout << "\n[ planner ] Planner received new waypoint: " << waypoint.pose.pose.position.x << " "
    //          << waypoint.pose.pose.position.y
    //          << " " << waypoint.pose.pose.position.z << "\n";

    Eigen::Vector3d new_pos;
    new_pos << waypoint.pose.pose.position.x, waypoint.pose.pose.position.y, waypoint.pose.pose.position.z;

    //std::cout << "[ planner ] Adding waypoint to matrix" << std::endl;
    if (waypoint_matrix.cols() < numberOfWaypoints) {
        //std::cout << "[ planner ] Resizing matrix" << std::endl;
        waypoint_matrix.conservativeResize(waypoint_matrix.rows(), waypoint_matrix.cols() + 1);
        waypoint_matrix.col(waypoint_matrix.cols() - 1) = new_pos;
    }
    // if the waypoint matrix is already full then waypoints
    // are shifted to the past and the new waypoint (new_pos) is added to the end
    else{
      //std::cout << "[ planner ] Shifting matrix" << std::endl;
        for (int row = 0; row<3; row++){
            for (int col = 0; col <numberOfWaypoints-1; col ++){
                waypoint_matrix(row,col) = waypoint_matrix(row,col+1);
            }
            waypoint_matrix(row,numberOfWaypoints-1) = new_pos(row);
        }
    }


    waypointCounter ++;
    //std::cout << "[ planner ] waypoint counter: " << waypointCounter << std::endl;

    //all waypoints sampled --> generate trajectory
    if(waypointCounter == numberOfWaypoints){
      std::cout << "[ planner ] Matrix full, Counter: " << waypointCounter << " recentWP: " << numberOfWaypoints << std::endl;
      mav_trajectory_generation::Trajectory trajectory;
      if (planTrajectory(new_pos, &trajectory)) {
        publishTrajectory(trajectory);
        waypointCounter = 0;
        waypoint_matrix.resize(3, 1);
        waypoint_matrix.setZero();
      }
    }

}

// Callback to get current Pose of UAV
void BasicPlanner::currentStateCallback(const nav_msgs::Odometry::ConstPtr& current_state) {

    // store current position in our planner
    tf::poseMsgToEigen(current_state->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(current_state->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
    std::cout << "[ planner ] Max speed set to: " << max_v_<< std::endl;
}

// Method to set maximum acceleration.
void BasicPlanner::setMaxAcc(const double max_a) {
    max_a_ = max_a;
    std::cout << "[ planner ] Max acc set to: " << max_a_<< std::endl;
}

// Plans a trajectory from the current position to the given waypoints
bool BasicPlanner::planTrajectory(const Eigen::VectorXd& new_pos,
                                    mav_trajectory_generation::Trajectory* trajectory) {

    std::cout << "[ planner ] Planning trajectory" << std::endl;

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 3 vertices:
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    //first waypoint
   start.makeStartOrEnd(waypoint_matrix.col(0),
                        derivative_to_optimize);
    vertices.push_back(start);


    //add all waypoints of the waypoint matrix to the new trajectory
    for(int i = 1; i < waypoint_matrix.cols() - 1; i++) {

        middle.addConstraint(0, waypoint_matrix.col(i));
        vertices.push_back(middle);
    }

    //last waypoint
    end.makeStartOrEnd(waypoint_matrix.col(waypoint_matrix.cols() - 1),
                         derivative_to_optimize);
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear <N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    std::cout << "[ planner ] Start optimizing, please wait..." << std::endl;
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    std::cout << "[ planner ] Got trajectory" << std::endl;
    return true;
}

//publish trajectory and markers for rviz
bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){

    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 4.0; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    std::cout << "[ planner ] Published trajectory" << std::endl;

    return true;
}
