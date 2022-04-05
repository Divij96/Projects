#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <beacon/receiver.h>


class controllerNode{

ros::NodeHandle nh;
 ros::Subscriber desired_state_vector, desired_state_waypoint, current_state, searchAktiv_sub, receiver_msg_sub;
 ros::Publisher prop_speeds;
 ros::Timer timer;


  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  bool searchAktiv_ = false; // searchAktiv true --> vector-following ; searchAktiv false --> waypoint-following


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){

      receiver_msg_sub = nh.subscribe("receiver/signal", 100, &controllerNode::onReceiverMsg, this);
      desired_state_waypoint = nh.subscribe("command/trajectory", 1, &controllerNode::onDesiredStateWaypoint, this);
      desired_state_vector = nh.subscribe("desired_state", 1, &controllerNode::onDesiredStateVector, this);
      current_state = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);
      prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

      // controller gains
      kx = 12.7;
      kv = 5.8;
      kr = 8.8;
      komega = 1.15;

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
  }

  // use waypoint_following when !seachAktiv to get the desired position
  void onDesiredStateWaypoint(const trajectory_msgs::MultiDOFJointTrajectory& des_state){
    if (!searchAktiv_){

                geometry_msgs::Vector3 t = des_state.points[0].transforms[0].translation;
                xd << t.x, t.y, t.z;

                geometry_msgs::Vector3 v = des_state.points[0].velocities[0].linear;
                vd << v.x, v.y, v.z;

                geometry_msgs::Vector3 a = des_state.points[0].accelerations[0].linear;
                ad << a.x, a.y, a.z;

                tf::Quaternion q;
                tf::quaternionMsgToTF(des_state.points[0].transforms[0].rotation, q);
                yawd = tf::getYaw(q);
                if (v.x == 0) {
                    yawd = 0;
                }
                else {
                    yawd = atan2(v.y, v.x);
                }
                yawd = - yawd;
        }
  }

  // use vector_following when seachAktiv to get the desired position
    void onDesiredStateVector(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      if (searchAktiv_){

                  //desired position
                  xd << des_state.transforms[0].translation.x, des_state.transforms[0].translation.y,
                  des_state.transforms[0].translation.z;
                  //desired velocity
                  vd << des_state.velocities[0].linear.x, des_state.velocities[0].linear.y, des_state.velocities[0].linear.z;
                  //desired acceleration
                  ad << des_state.accelerations[0].linear.x, des_state.accelerations[0].linear.y, des_state.accelerations[0].linear.z;
                  //desired yaw from quaternion instance
                  yawd = tf::getYaw(des_state.transforms[0].rotation);
          }
    }

    void onCurrentState(const nav_msgs::Odometry& cur_state){
      x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
      omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
      R = q.toRotationMatrix();
      // Rotate omega
      omega = R.transpose()*omega;
  }


  //determine if vector_following or waypoint_following by comparing the distance to the next beacon
  void onReceiverMsg(const beacon::receiver receiver_msg){
    if(receiver_msg.distance < 0 ){
      searchAktiv_ = false;
    } else if(receiver_msg.distance < 3500) {
      searchAktiv_ = true;
    }
  }

  //controller loop --> unchanged from Lab_6
  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;


    ex = x - xd;
    ev = v - vd;


    Eigen::Vector3d b_3d = -kx*ex - kv*ev + m*g*e3 + m*ad;
    b_3d.normalize();

    Eigen::Vector3d b_1d(cos(yawd), sin(yawd), 0);
    Eigen::Vector3d b_2d = b_3d.cross(b_1d);
    b_2d.normalize();
    Eigen::Vector3d b_1d_true(b_2d.cross(b_3d));
    b_1d_true.normalize();

    Eigen::Matrix3d Rd;
    Rd << b_1d_true, b_2d, b_3d;

    er = 0.5 * Vee(Rd.transpose()*R - R.transpose()*Rd);
    eomega = omega;


    Eigen::Vector3d errs = -kx*ex - kv*ev + m*g*e3 + m*ad;
    double f = errs.dot(R*e3);
    Eigen::Vector3d torques = -kr*er -komega*eomega + omega.cross(J*omega);

    Eigen::Vector4d wrench(f, torques.x(), torques.y(), torques.z());


    double d_hat = d/sqrt(2);
    Eigen::Matrix4d F;

    F << cf, cf, cf, cf,
        cf*d_hat, cf*d_hat, -cf*d_hat, -cf*d_hat,
        -cf*d_hat, cf*d_hat, cf*d_hat, -cf*d_hat,
        cd, -cd, cd, -cd;


    Eigen::Vector4d props = F.inverse()*wrench;

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = signed_sqrt(props[0]);
    msg.angular_velocities[1] = signed_sqrt(props[1]);
    msg.angular_velocities[2] = signed_sqrt(props[2]);
    msg.angular_velocities[3] = signed_sqrt(props[3]);

    prop_speeds.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
