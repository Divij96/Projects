// ==========================================
//  Title:  receiver node
//  Author: Divij Grover
//  Last updated: 24 March 2022
// ==========================================

//This node calculates the flux lines at the position of the drone

#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
//messages
#include <beacon/receiver.h>
#include <beacon/beacon_msg.h>
#include <transmitters/transmitter.h>
#include <transmitters/transmitter_array.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

using namespace Eigen;

#define PI M_PI

  class receiverNode {
    ros::NodeHandle nh;

  	typedef struct BEACON_TRANSMITTER {
  		int id;
  		Eigen::Vector3d pos;
  		Eigen::Matrix3d orientation;
  		}BEACON_TRANSMITTER;

	struct found_position {
		Eigen::Vector3d pos;
		float distance;
    };


  	private: ros::Timer timer;
  	private: ros::Publisher _beacon_marker_pub;
  	private: ros::Subscriber _transmitter_sub;
  	private: ros::Subscriber _current_state_sub;
  	private: ros::Publisher _beacon_pub;
  	private: std::vector< BEACON_TRANSMITTER > _beacon_sensors;
	private: std::vector< BEACON_TRANSMITTER > _beacon_sensors_permanent;
	private: std::vector< found_position > _found_positions;
	private: BEACON_TRANSMITTER _current_beacon_location;

    ros::Publisher found_beacon_marker_pub;


  	private: int size = 0;
	private: int size_message = 0;
	private: double counter = 0;
	private: double trigger = 0;
	private: bool use_old_data = false;

  	private: beacon::receiver _detected_beacon;
	private: beacon::receiver _detected_beacon_old_data;
	private: beacon::receiver _publish_beacon;
 
  	private: beacon::receiver _undetected_beacon;
	private: bool beacon_identified = false;
	private: float r_noise = 50; //% in [m]
	private: float threshold_distance = 250; // in [cm]
	private: float tertiary_search_distance = 5; // in [m]]


  	private: Eigen::Vector3d _receiver_pos;
	private: Eigen::Vector3d _receiver_pos_old;
  	private: Eigen::Matrix3d _receiver_rot;
  	private: visualization_msgs::Marker _beacon_marker;

  	private: bool _new_data = false;

  	private: Eigen::Vector3d _marker_colors;
    double hz; 


    public: receiverNode():hz(50.0)  {


      if (!ros::isInitialized()) {
     ROS_FATAL_STREAM("ROS has not been initialized, unaable to load receiver node.");
     return;
 		}

	  _receiver_pos_old = {0, 0, 0};

     // Initialize ROS callback handlers
	 
	 //pose and orientation of the beacons
     _transmitter_sub = nh.subscribe("transmitter/pose", 1, &receiverNode::transmitters_sub, this ); 
	 
	 //pose and orientation of the drone
	 _current_state_sub = nh.subscribe("current_state", 1, &receiverNode::onCurrentState, this ); //requires pose of the beacons
	 
	 //publishes the flux vector and distance to the beacona dn corrosponding beacon id of the drone     
	 _beacon_pub = nh.advertise<beacon::receiver>("receiver/signal", 0);

	 //publishes the marker for flux vector     
     _beacon_marker_pub = nh.advertise<visualization_msgs::Marker>("receiver/visualization", 0);

	 //publishes the estimated location of found beacon     
     found_beacon_marker_pub = nh.advertise<visualization_msgs::Marker>("found_beacon_marker", 1);

     timer = nh.createTimer(ros::Rate(hz), &receiverNode::receiver_loop, this);


		}

    public: void create_arrow_marker(Eigen::Vector3d Hb) {
			//creates an arrow marker at the drone pointing in flux direction
			// written by Benedikt Fischhaber

			visualization_msgs::Marker arrow_marker;
			arrow_marker.header.frame_id = "world";
			arrow_marker.header.stamp = ros::Time::now();
			arrow_marker.ns = "beacon_visualisation_receiver_markers";
			arrow_marker.id = 20;
			arrow_marker.type = visualization_msgs::Marker::ARROW;
			arrow_marker.action = visualization_msgs::Marker::ADD;
			arrow_marker.lifetime = ros::Duration(0.5);
			arrow_marker.pose.orientation.x = 0.0;
			arrow_marker.pose.orientation.y = 0.0;
			arrow_marker.pose.orientation.z = 0.0;
			arrow_marker.pose.orientation.w = 1.0;
			arrow_marker.scale.x = 0.5;
			arrow_marker.scale.y = 1;
			arrow_marker.scale.z = 1.5;

			arrow_marker.color.a = 1.0;
			arrow_marker.color.r = 1;
			arrow_marker.color.g = 0;
			arrow_marker.color.b = 0;
			arrow_marker.points.resize(2);
			arrow_marker.points[0].x = _receiver_pos[0];
			arrow_marker.points[0].y = _receiver_pos[1];
			arrow_marker.points[0].z = _receiver_pos[2];
			arrow_marker.points[1].x = _receiver_pos[0] + 4*Hb(0);
			arrow_marker.points[1].y = _receiver_pos[1] + 4*Hb(1);
			arrow_marker.points[1].z = _receiver_pos[2] + 4*Hb(2);

			_beacon_marker_pub.publish(arrow_marker);

		}

	public: void logger(bool real_position){
		//saves positions of detected and real beacons for location report
		//written by Benedikt Fischhaber
		if (real_position == true){
			_beacon_sensors_permanent = _beacon_sensors;
		}
		else if (real_position == false){
			found_position temp;
			temp.distance = _detected_beacon.distance;
			//approximation of beacon position
			temp.pos[0] = _receiver_pos[0] + _detected_beacon.direction_vector.x*_detected_beacon.distance/100;
          	temp.pos[1] = _receiver_pos[1] + _detected_beacon.direction_vector.y*_detected_beacon.distance/100;
         	temp.pos[2] = _receiver_pos[2] + _detected_beacon.direction_vector.z*_detected_beacon.distance/100;
			
			_found_positions.push_back(temp);
		}
	}

	public: void create_location_report(bool final){
		//creates a intermediate location report and final location report
		//written by Benedikt Fischhaber
		int size_r = static_cast<int>(_beacon_sensors_permanent.size());
		int size_f = static_cast<int>(_found_positions.size());
		std::vector< BEACON_TRANSMITTER > _beacon_sonsors_found;
		
		if (final == true){
			//creates a BEACON_TRANSMITTER vector with correct Beacon IDs depending on L2 error
			float L2 = 0;
			float L2_old = 10000000; //unreachable L2 error
			_beacon_sonsors_found.resize(size_f);
			for (int q = 0; q < size_f; ++q){
				for (int i = 0; i < size_r; ++i){
					L2 = pow(_beacon_sensors_permanent[i].pos[0] - _found_positions[q].pos[0], 2) +
						pow(_beacon_sensors_permanent[i].pos[1] - _found_positions[q].pos[1], 2) +
						pow(_beacon_sensors_permanent[i].pos[2] - _found_positions[q].pos[2], 2);
					if(L2 < L2_old){
						_beacon_sonsors_found[q].id = i+1;
						_beacon_sonsors_found[q].pos = _found_positions[q].pos;
						L2_old = L2;
					}
				}
				L2 = 0;
				L2_old = 10000000; 
			}
		}


		std::cout<<"---------------------------------------------------"<<"\n";

		if (final == true){
			std::cout<<"FINAL LOCATION REPORT"<<"\n\n";
		}
		else{
			std::cout<<"LOCATION REPORT"<<"\n\n";
		}

		std::cout<<"Real locations of beacons:"<<"\n";

		for (int i = 0; i < size_r; ++i){
			std::cout<<"Beacon ID: "<<_beacon_sensors_permanent[i].id<<" x: "<< _beacon_sensors_permanent[i].pos[0]<<" y: "<< _beacon_sensors_permanent[i].pos[1]<<" z: "<< _beacon_sensors_permanent[i].pos[2]<<"\n";
		}

		std::cout<<"\n"<<"Approximated locations of beacons:"<<"\n";

		if (final == true){
			for (int q = 0; q < size_r; ++q)
				//sort according to id
				for (int i = 0; i < size_f; ++i){
					if (q +1 == _beacon_sonsors_found[i].id){
						std::cout<<"Beacon ID: "<<_beacon_sonsors_found[i].id<<" x: "<<_beacon_sonsors_found[i].pos[0]<<" y: "<<_beacon_sonsors_found[i].pos[1]<<" z: "<<_beacon_sonsors_found[i].pos[2]<<"\n";
					}
				}
		}
		else{
			for (int i = 0; i < size_f; ++i){
				std::cout<<"Beacon: #"<< i+1 <<" x: "<< _found_positions[i].pos[0]<<" y: "<< _found_positions[i].pos[1]<<" z: "<< _found_positions[i].pos[2]<<"\n";
			}
		}
		if (final == true){
			std::cout<<"\n"<<size_f<<"/"<<size_r<<" beacons found!"<<"\n";
		}

        std::cout<<"\n";
		std::cout<<"---------------------------------------------------"<<"\n";
	}

    public: void get_beacon_data(Eigen::Vector3d p, Eigen::Matrix3d R,Eigen::Vector3d pt, Eigen::Matrix3d Rt, Eigen::Vector3d & Hb, float & d, float & delta ) {

			//calculates flux line based on the orientation and position of the beacons and drone.
			//ref. J. Cacace, N. Mimmo and L. Marconi, "A ROS Gazebo plugin to simulate ARVA sensors,
			//" 2020 IEEE International Conference on Robotics and Automation (ICRA), 2020, 
			//pp. 7233-7239, doi: 10.1109/ICRA40945.2020.9196914.
			//https://github.com/jocacace/arva_sim

			Eigen::Vector3d m_vec;
			m_vec << 1.0, 0.0, 0.0;

			m_vec = Rt*m_vec;
			float n_H_noise = 1 / pow(r_noise, 3) * (1 / (4*M_PI))*1.5420;

			Eigen::Vector3d r;
			r = p - pt;

			Eigen::Matrix3d A;
			A << 	2*pow(r[0],2)-pow(r[1],2)-pow(r[2],2), 3*r[0]*r[1], 3*r[0]*r[2],
					3*r[0]*r[1], 2*pow(r[1],2)-pow(r[0],2)-pow(r[2],2), 3*r[1]*r[2],
					3*r[0]*r[2], 3*r[1]*r[2], 2*pow(r[2],2)-pow(r[0],2)-pow(r[1],2);

			Eigen::Vector3d Am;
			Am << A(0,0)*m_vec[0]+A(0,1)*m_vec[1]+A(0,2)*m_vec[2],
				  A(1,0)*m_vec[0]+A(1,1)*m_vec[1]+A(1,2)*m_vec[2],
				  A(2,0)*m_vec[0]+A(2,1)*m_vec[1]+A(2,2)*m_vec[2];


			float Am_x = A(0,0)*m_vec[0]+A(0,1)*m_vec[1]+A(0,2)*m_vec[2];
			float Am_y = A(1,0)*m_vec[0]+A(1,1)*m_vec[1]+A(1,2)*m_vec[2];
			float Am_z = A(2,0)*m_vec[0]+A(2,1)*m_vec[1]+A(2,2)*m_vec[2];

			float rd = r.norm();

			Eigen::Vector3d H;
			H << 1/(4*M_PI*pow(rd,5))*Am_x, 1/(4*M_PI*pow(rd,5))*Am_y, 1/(4*M_PI*pow(rd,5))*Am_z;
			Hb = R*H;

			float d1 = powf( (1.0/fabs(Hb[0])*(1.0/(4.0*M_PI))*1.5420), 0.33);
			float d3 = powf( (1.0/Hb.norm()*(1.0/(4.0*M_PI))*1.5420), 0.33);


			if ( d3 <= tertiary_search_distance ) {
				d = d3;
				delta = 0.0; 
			}
			else if( d1 <= r_noise ) {
				d = d1;
				delta = atan2(Hb[1], Hb[0]);
			}
			else {
				d = -1;
				delta = 0;
			}

			if( d!=-1) {
				d = round(d*100);
				delta = round( delta*180.0/M_PI);
			}

	}

    public: void receiver_loop( const ros::TimerEvent& t ) {

		//calculates flux line based on the orientation and position of the beacons and drone
		//and sends the flux line vector and the estimated distance between the transmiiter and drone
		

		Eigen::Vector3d Hb;
        float  d;
        float  delta;
		float  min;

        //Calculate all the magnetic field flows once the data is read from the transmitter node
		if (_new_data){
			//if the beacon is not identified, node keeps on calculating if the drone is in range
			//when the beacon signal is identified(i.e drone finds a beacon below r_noise rnage), 
			//beacon_identifiedis set true
			if( beacon_identified == false ){

				_publish_beacon.beacon_id = -1;
				_publish_beacon.distance = -1;
				_publish_beacon.direction_vector.x  = -1;
				_publish_beacon.direction_vector.y  = -1;
				_publish_beacon.direction_vector.z  = -1;

				_beacon_pub.publish( _publish_beacon );

				min = r_noise*1000; //help variable to find the beacon closest to the drone

				for (int i=0;i<size;i++){
					//calculates flux line based on the orientation and position of the beacons and drone
					get_beacon_data( _beacon_sensors[i].pos, _beacon_sensors[i].orientation, _receiver_pos, _receiver_rot, Hb, d, delta);

					//find the beacon which has the minimum distance to the drone 
					if (min>d && d > 0){
						min = d;
						_detected_beacon.distance = d;
						_current_beacon_location.pos = _beacon_sensors[i].pos;
						_current_beacon_location.orientation  = _beacon_sensors[i].orientation;
						_current_beacon_location.id = _beacon_sensors[i].id;
						beacon_identified = true;
					}
				}

			}
			else if( beacon_identified == true ){

				get_beacon_data( _current_beacon_location.pos, _current_beacon_location.orientation, _receiver_pos, _receiver_rot, Hb, d, delta);

				_detected_beacon.beacon_id = _current_beacon_location.id;
				_detected_beacon.distance = d;
				Hb.normalize();
				_detected_beacon.direction_vector.x  = Hb[0];
				_detected_beacon.direction_vector.y  = Hb[1];
				_detected_beacon.direction_vector.z  = Hb[2];

				_publish_beacon.beacon_id = _detected_beacon.beacon_id;
				_publish_beacon.distance =  _detected_beacon.distance;
				_publish_beacon.direction_vector.x  = _detected_beacon.direction_vector.x;
				_publish_beacon.direction_vector.y  = _detected_beacon.direction_vector.y;
				_publish_beacon.direction_vector.z  = _detected_beacon.direction_vector.z;

				//last reliable data is sent to the drone for atleast 3 seconds after the drone is lost
				//drone is given 3 seconds to find the reliable data, if no new data is found within the 
				//time period, the beacon is considered to be lost
				if (_detected_beacon.distance < 0){
					counter = counter + 1/hz;
					
					if (counter >=3){
						beacon_identified = false;
						counter = 0;
					}

					use_old_data = true;

					//send old data 
					_publish_beacon.beacon_id = _detected_beacon_old_data.beacon_id ;
					_publish_beacon.distance =_detected_beacon_old_data.distance;
					_publish_beacon.direction_vector.x =_detected_beacon_old_data.direction_vector.x;
					_publish_beacon.direction_vector.y =_detected_beacon_old_data.direction_vector.y;
					_publish_beacon.direction_vector.z =_detected_beacon_old_data.direction_vector.z;						
					
				}
				else if (_detected_beacon.distance > 0 ){
		
					use_old_data = false;
					counter = 0;

				}
				
				// if(tertiary_search_distance < 500){
				// 	_publish_beacon.beacon_id = _detected_beacon.beacon_id;
				// 	_publish_beacon.distance =  _detected_beacon.distance;
				// 	_publish_beacon.direction_vector.x  = _detected_beacon.direction_vector.x;
				// 	_publish_beacon.direction_vector.y  = _detected_beacon.direction_vector.y;
				// 	_publish_beacon.direction_vector.z  = _detected_beacon.direction_vector.z;

				// }
				
				//saves old data before the drone is lost
				if (use_old_data == false){
					_detected_beacon_old_data.beacon_id = _detected_beacon.beacon_id ;
					_detected_beacon_old_data.distance =_detected_beacon.distance;
					_detected_beacon_old_data.direction_vector.x =_detected_beacon.direction_vector.x;
					_detected_beacon_old_data.direction_vector.y =_detected_beacon.direction_vector.y;
					_detected_beacon_old_data.direction_vector.z =_detected_beacon.direction_vector.z;					

				}

				//removes beacon if the drone on the top or distance to drone is less than threshold distance

				if(_detected_beacon.distance > 0 && _detected_beacon.distance <= threshold_distance ||
					(abs(abs(_detected_beacon.direction_vector.z)-abs(_detected_beacon.direction_vector.x)) > 0.9 &&
					 abs(abs(_detected_beacon.direction_vector.z)-abs(_detected_beacon.direction_vector.y)) > 0.9)) {
					//remove detected beacon from the list
					for (int i=0;i<size;i++){
						if ( _beacon_sensors[i].id == _detected_beacon.beacon_id ){
							_beacon_sensors.erase(_beacon_sensors.begin()+i);
						}
					}
					logger(false);
					size = _beacon_sensors.size();

					beacon_identified = false;
					std::cout << "found beacon" << std::endl;

                    place_found_beacon_marker(_receiver_pos);
					//create_location_report(false);

					_publish_beacon.beacon_id = -1;
					_publish_beacon.distance = -1;
					_publish_beacon.direction_vector.x  = -1;
					_publish_beacon.direction_vector.y  = -1;
					_publish_beacon.direction_vector.z  = -1;

					_beacon_pub.publish( _publish_beacon );

				}
				create_arrow_marker(Hb);
				_beacon_pub.publish( _publish_beacon );
			}

			//checks if drone returns to start box and triggers location report
			if((abs(_receiver_pos[0])<5 && abs(_receiver_pos[1])<5 && abs(_receiver_pos[2])<5) && 
			  (abs(_receiver_pos_old[0]) > 5 || abs(_receiver_pos_old[1]) > 5 || abs(_receiver_pos_old[2]) > 5)){
				  create_location_report(true);
				  _receiver_pos_old = _receiver_pos;
			}

			if(trigger == hz*3){
				//position is saved only every 3sek to prevent report to be published at startup, because of glitchy
				//drone positions form unity
				_receiver_pos_old = _receiver_pos;
				trigger = 0;
			}
			trigger++;
		}
		}

    public: void onCurrentState(const nav_msgs::Odometry& cur_state){
		//gets the cuurent position and orientation of the drone
        _receiver_pos << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
        _receiver_rot = q.toRotationMatrix();

  }

 	public: void place_found_beacon_marker(const Eigen::Vector3d found_beacon_pos ){

          visualization_msgs::Marker marker;
          // Set the frame ID and timestamp.  See the TF tutorials for information on these.
          marker.header.frame_id = "world";
          marker.header.stamp = ros::Time::now();

          // Set the namespace and id for this marker.  This serves to create a unique ID
          // Any marker sent with the same namespace and id will overwrite the old one
          marker.ns = "basic_shapes";
          marker.id = _current_beacon_location.id;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
          uint32_t shape = visualization_msgs::Marker::SPHERE;
          marker.type = shape;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = found_beacon_pos[0] + _detected_beacon.direction_vector.x*_detected_beacon.distance/100;
          marker.pose.position.y = found_beacon_pos[1] + _detected_beacon.direction_vector.y*_detected_beacon.distance/100;
          marker.pose.position.z = found_beacon_pos[2] + _detected_beacon.direction_vector.z*_detected_beacon.distance/100;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          // Set the scale of the marker -- 1x1x1 here means 1m on a side
          marker.scale.x = _detected_beacon.distance/100;
          marker.scale.y = _detected_beacon.distance/100;
          marker.scale.z = _detected_beacon.distance/100;

          // Set the color -- be sure to set alpha to something non-zero! 52, 235, 222
          marker.color.r = 0.52;
          marker.color.g = 2.35;
          marker.color.b = 2.22;
          marker.color.a = 0.5;

          // Publish the marker
          while (found_beacon_marker_pub.getNumSubscribers() < 1)
          {
              if (!ros::ok())
              {
                  return;
              }
              ROS_WARN_ONCE("Please create a subscriber to the found beacon marker");
              sleep(1);
          }
          found_beacon_marker_pub.publish(marker);

    }

    public: void transmitters_sub(const transmitters::transmitter_array& transmitters_info){

		//read data
		//calculate number of transmitter

		size_message = transmitters_info.transmitter_arr.size();


		if (_new_data == false && size_message > 0)
		{
			// _beacon_sensors.resize(size);
			size = size_message;
			_beacon_sensors.resize(size);
			//read the data
			for(int i = 0; i<size; i++){
				_beacon_sensors[i].id = transmitters_info.transmitter_arr[i].beacon_id;
				_beacon_sensors[i].pos << transmitters_info.transmitter_arr[i].position_vector.x,
											transmitters_info.transmitter_arr[i].position_vector.y,
											transmitters_info.transmitter_arr[i].position_vector.z;
				Eigen::Quaterniond q_trans;
				tf::quaternionMsgToEigen (transmitters_info.transmitter_arr[i].rotation_quaternion, q_trans);
				_beacon_sensors[i].orientation = q_trans;
			}
			logger(true);
			_new_data = true;
		}
	}


  };

int main(int argc, char** argv){

  ros::init(argc, argv, "receiver_node");
  receiverNode n;
  ros::spin();
}
