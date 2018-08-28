// Weilun Peng
// 08/12/2018

#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <chrono>

#include "HybridAstar_truck_cplusplus/src/map.hpp"
#include "HybridAstar_truck_cplusplus/src/pathfinder_hybrid_astar.hpp"
#include "HybridAstar_truck_cplusplus/src/def_all.hpp"
#include "MPC/mpc_controller.hpp"
#include "MPC/pid_controller.hpp"

// message libraries
#include "ackermann_msgs/AckermannDrive.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace std::chrono;

static void toEulerAngle(const vector<double>& q, double& roll, double& pitch, double& yaw);

static void toEulerAngle(const vector<double>& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double q_x = q[0];
  double q_y = q[1];
  double q_z = q[2];
  double q_w = q[3];
  double sinr = +2.0 * (q_w * q_x + q_y * q_z);
  double cosr = +1.0 - 2.0 * (q_x * q_x + q_y * q_y);
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q_w * q_y - q_z * q_x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (q_w * q_z + q_x * q_y);
  double cosy = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
  yaw = atan2(siny, cosy);
}

class AutoparkingControl
{
public:
  AutoparkingControl()
  {

    //Topic you want to subscribe
    submapreallimit_ = n_.subscribe<std_msgs::Float32MultiArray>("/mapreallimit", 1, &AutoparkingControl::mapmapreallimitcallback, this);
    submap_ = n_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &AutoparkingControl::mapcallback, this);
    subveh_ = n_.subscribe<visualization_msgs::Marker>("/player_vehicle", 1, &AutoparkingControl::mainCallback, this);

    //Topic you want to publish
    pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 1000);

  }

  void mapmapreallimitcallback(const std_msgs::Float32MultiArray::ConstPtr& mapreallimit_info)
  {
    min_x_m = mapreallimit_info->data[0];
    min_y_m = mapreallimit_info->data[1];
    max_x_m = mapreallimit_info->data[2];
    max_y_m = mapreallimit_info->data[3];
    cout << "dur_x: " << min_x_m << " " << max_x_m << endl;
    cout << "dur_y: " << min_y_m << " " << max_y_m << endl;
  }

  void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& map_info)
  {
    // cout << "Map info: " << map_info->info.width << " " << map_info->info.height << " " << map_info->info.resolution << endl;
    // cout << "Map origin: " << map_info->info.origin.position.x << " " << map_info->info.origin.position.y << " " << map_info->info.origin.position.z << endl;
    // cout << "Map size: " << map_info->data.size() << endl;
    for (int i = 0; i < map_info->info.width; i++) {
      for (int j = 0; j < map_info->info.height; j++) {
        if (map_info->data[i+j*map_info->info.width]) {
          // ox_map.emplace_back(min_x_m + (map_info->info.width - 1 - i)*map_info->info.resolution);
          // ox_map.emplace_back(min_x_m + map_info->info.resolution*i-2.5);
          // oy_map.emplace_back(min_y_m + map_info->info.resolution*j+2.5);
          ox_map.emplace_back(min_x_m + map_info->info.resolution*i);
          oy_map.emplace_back(min_y_m + map_info->info.resolution*j);
        }
      }
    }
    // int i = 641;
    // int j = 191;
    // cout << "fsdfa: " << int(map_info->data[i+j*map_info->info.width]) << endl;
    // cout << "x: " << min_x_m + (map_info->info.width - 1 - i)*map_info->info.resolution << endl;
    // cout << "y: " << min_y_m + map_info->info.resolution*j << endl;
    // cout << "resolution: " << map_info->info.resolution << endl;
  }

  void mainCallback(const visualization_msgs::Marker::ConstPtr& vehicle_current_info)
  {
    vector<double> q = {vehicle_current_info->pose.orientation.x, vehicle_current_info->pose.orientation.y, vehicle_current_info->pose.orientation.z, vehicle_current_info->pose.orientation.w};
    toEulerAngle(q, current_roll, current_pitch, current_yaw);
    // cout << "hahaha" << endl;
    if (!first_com) {
      init_x = vehicle_current_info->pose.position.x;
      init_y = -vehicle_current_info->pose.position.y;
      init_yaw = current_yaw;
      first_com = true;

      // build the map by own setting
      // cout << "oxsize: " << ox_map.size() << " " << oy_map.size() << endl;
      // ofstream savefile;
      // savefile.open ("obstacle.dat");
      // for (size_t i = 0; i < ox_map.size(); i++) {
      //     savefile << ox_map[i] << " " << oy_map[i] << endl;
      // }
      // savefile.close();
      //find the final path
      // double gx = -8;  // [m]
      // double gy = 126;  // [m]
      // double gyaw0 = 90.0*D2R;
      // double gyaw1 = 90.0*D2R;
      double gx = 100;  // [m]
      double gy = 199;  // [m]
      double gyaw0 = -180.0*D2R;
      double gyaw1 = -180.0*D2R;
      // double gx = 88;  // [m]
      // double gy = 160;  // [m]
      // double gyaw0 = -90.0*D2R;
      // double gyaw1 = -90.0*D2R;
      cout << "Start Configuration: (" << init_x << ", " << init_y << ", " << init_yaw*R2D << ", " << init_yaw*R2D << ")" << endl;
      cout << "Goal Configuration: (" << gx << ", " << gy << ", " << gyaw0*R2D << ", " << gyaw1*R2D << ")" << endl;
      ifstream infile;
      infile.open("path_2.dat");
      double x, y;
      bool direction;
      while (infile >> x >> y >> direction)
      {
          path_.x.emplace_back(x);
          path_.y.emplace_back(y);
          // cout << direction << endl;
          path_.direction.emplace_back(direction);
      }
      cout << "Program Done!!" <<endl;
      prev_x = init_x;
      prev_y = init_y;
      prev_timer = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    }
    else {
      ackermann_msgs::AckermannDrive current_cmd;
      //cout << "Current index: " << curr_idx << endl;
      current_x = vehicle_current_info->pose.position.x;
      current_y = -vehicle_current_info->pose.position.y;
      curr_timer = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      cout<<"Delta Time is (milliseconds) !!!!!!!!!!!!!!!!!!"<< curr_timer.count() - prev_timer.count() <<endl;
/////////////////////////////////////////////////////////////////////////////////////////////////
      //MPC controller
      curr_idx = find_curr_idx_2(&path_, current_x, current_y, prev_idx);

      // if (abs(path_.x[current_idx] - current_x) < 1.0 && abs(path_.y[current_idx] - current_y) < 1.0) {
      //   current_idx++;
      // }
      // curr_idx = current_idx;

      control_command = pid_control_command(&path_, current_x, current_y, prev_x, prev_y, current_yaw, curr_idx, (curr_timer.count() - prev_timer.count())/1000.0);
      steering_angle_command = control_command[0];
      throttle_command = control_command[1];
      speed_command = control_command[2]; 
      real_speed = control_command[3];    
      cout<< "Vehicle Real Speed is: " << real_speed <<endl;

/////////////////////////////////////////////////////////////////////////////////////////////////
      if (curr_idx > path_.x.size() - 6) {
        cout << "Reach Final Goal Position" << endl;
        current_cmd.speed = 0.0;
      }
      else {
        //current_cmd.speed = speed_command * 1.609344;
        int throttle_sign = 1;
        int speed_sign = 1;
        if(throttle_command < 0){throttle_sign = -1;}
        if(speed_command < 0){speed_sign = -1;}
        // if(throttle_sign != speed_sign) {speed_command -= 1;}
        current_cmd.speed = speed_command;
        current_cmd.acceleration = throttle_command * throttle_sign;
        current_cmd.steering_angle = steering_angle_command;
        cout << "command: speed/throttle/steering_angle/throttle_sign " << current_cmd.speed << " " << current_cmd.acceleration << " " << current_cmd.steering_angle << " " <<throttle_sign << endl;
      }
      prev_x = current_x;
      prev_y = current_y;
      prev_timer = curr_timer;
      cout << "curr_idx: " << curr_idx << endl;
      cout << "simulatedpath: " << path_.x[curr_idx] << " " << path_.y[curr_idx] << endl;
      cout << "realpath: " << current_x << " " << current_y << " " << current_yaw << endl;
      
      pub_.publish(current_cmd);
    }
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber submap_;
  ros::Subscriber subveh_;
  ros::Subscriber submapreallimit_;
  Path_Final path_;
  int current_idx = 0;
  int prev_idx = 0;
  int curr_idx = 0;
  bool first_com;
  double init_x;
  double init_y;
  double init_yaw;
  double current_x;
  double current_y;
  double prev_x;
  double prev_y;
  double current_roll;
  double current_pitch;
  double current_yaw;
  double min_x_m;
  double max_x_m;
  double min_y_m;
  double max_y_m;
  double steering_angle_command;
  double throttle_command;
  double speed_command;
  double real_speed;
  milliseconds curr_timer = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  milliseconds prev_timer = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  vector<double> ox_map;
  vector<double> oy_map;
  vector<double> control_command;
};//End of class AutoparkingControl

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "autoparking_control");

  ros::NodeHandle n;

  // bool carla_autopilot;
  // n.getParam("carla_autopilot", carla_autopilot);
  
  // if (!carla_autopilot) {
    ROS_INFO_STREAM("Reading Vechile Info from topic");
    AutoparkingControl autopark_control;
  // }
  // else ROS_INFO_STREAM("Autopilot Mode");

  ros::spin();

  return 0;
}
