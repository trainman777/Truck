#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <queue>
#include <geodesy/utm.h>
void loadKMLGoalFile(const std::string &goal_filename);
void loadOdomGoalFile(const std::string &goal_filename);

ros::Publisher vel_pub;
ros::Timer state_machine_timer;

enum class States {
  IDLE,
  GO,
  DRIVE_ODOM,
  DRIVE_UTM
};

States state = States::IDLE;
States next_state = States::IDLE;
std::string command;
geometry_msgs::Pose2D current_pose_odom;
geometry_msgs::Pose2D current_pose_utm;
std::queue<geometry_msgs::Pose2D> goals_odom;
std::queue<geometry_msgs::Pose2D> goals_utm;
ros::Duration elapsed_time;
ros::Time state_start_time;

geometry_msgs::Pose2D poseToPose2D(geometry_msgs::Pose pose) {
  geometry_msgs::Pose2D p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  geometry_msgs::Quaternion q = pose.orientation;
  p.theta = atan2(2.0 * (q.z * q.w + q.x * q.y) , 
      - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)); 
  return p;
}
std::ostream &operator<<(std::ostream &os, const geometry_msgs::Pose2D &p) {
  return os << "(" << p.x << "," << p.y << "," << p.theta*180.0/M_PI << ")";
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  current_pose_odom = poseToPose2D(odom->pose.pose);
}

void utmCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  current_pose_utm = poseToPose2D(odom->pose.pose);
}

void goalOdomCallback(const geometry_msgs::Pose2D::ConstPtr &goal) {
  ROS_INFO_STREAM("New Odom Goal\n" << *goal);
  goals_odom.push(*goal);
}
void goalUtmCallback(const geometry_msgs::Pose2D::ConstPtr &goal) {
  ROS_INFO_STREAM("New UTM Goal\n" << *goal);
  goals_utm.push(*goal);
}

void cmdCallback(const std_msgs::String::ConstPtr &cmd) {
  ROS_INFO_STREAM("Command received" << *cmd << "\n");
  command = cmd->data;
}

void distanceBetweenPoses(
    const geometry_msgs::Pose2D &a,
    const geometry_msgs::Pose2D &b, 
    float &distance, float &angle) {
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  distance = sqrt(dx*dx + dy*dy);
  angle = atan2(dy, dx);
}
float limit(float v, float limit) {
  if (v>limit) return limit;
  if (v<-limit) return -limit;
  return v;
}
// create cmd_vel to get to odom goal.  return true if arrived, false otherwise
bool driveToGoal(geometry_msgs::Pose2D goal, geometry_msgs::Twist &cmd_vel) 
{
  float d, a, steering_error;
  distanceBetweenPoses(goal, current_pose_odom, d, a);
  steering_error = angles::shortest_angular_distance(a,
      current_pose_odom.theta);
  ROS_INFO_STREAM("at " << current_pose_odom << " goal "<< 
      goal <<
      " relative=(" <<d<<","<<a<<") err="<<steering_error);
  if(d < 0.5) {
    return true;
  } else {
    cmd_vel.linear.x =0.5;
    cmd_vel.angular.z = limit(-steering_error,0.3);
    return false;
  }
}

geometry_msgs::Pose2D utmToOdom(geometry_msgs::Pose2D pose_utm) {
  geometry_msgs::Pose2D pose_odom;
  pose_odom.x = current_pose_utm.x - pose_utm.x + current_pose_odom.x; 
  pose_odom.y = current_pose_utm.y - pose_utm.y + current_pose_odom.y; 
  ROS_INFO_STREAM("utmToOdom: utm="<<pose_utm<<" current UTM="<<current_pose_utm<<" odom="<<pose_odom);
  return pose_odom;
}
void stateMachineCallback(const ros::TimerEvent &e) {
  elapsed_time = ros::Time::now() - state_start_time;
  //ROS_INFO_STREAM("State machine "<< (int)state);
  // handle commands
  if (!command.empty()) {
    ROS_INFO_STREAM("Handled command " << command);
    if(command == "GO") {
      if (state == States::IDLE) {
        next_state = States::GO;
        ROS_INFO("Statring");
      }
    } else if(command == "STOP") {
      if (state != States::IDLE) {
        next_state = States::IDLE;
        ROS_INFO("Stopping");
      }
    } else if(command == "DRIVE_ODOM") {
      if (state == States::IDLE) {
        next_state = States::DRIVE_ODOM;
        ROS_INFO_STREAM("Driving to Odom goals");
      }
    } else if(command == "DRIVE_UTM") {
      if (state == States::IDLE) {
        next_state = States::DRIVE_UTM;
        ROS_INFO_STREAM("Driving to UTM goals");
      }
    } else if(command == "RESET") {
      while(!goals_odom.empty()) goals_odom.pop();
      while(!goals_utm.empty()) goals_utm.pop();
      next_state = States::IDLE;
    } else if(command == "CLEAR_GOALS") {
      while(!goals_odom.empty()) goals_odom.pop();
      while(!goals_utm.empty()) goals_utm.pop();
    } else if(command.substr(0,8) == "LOAD_KML") {
      loadKMLGoalFile(command.substr(9));
    } else if(command.substr(0,9) == "LOAD_ODOM") {
      loadOdomGoalFile(command.substr(10));
    }
    command.clear();
  }
  // handle state changes
  if(next_state != state) {
    state_start_time = ros::Time::now();
    state = next_state;
  }
  geometry_msgs::Twist cmd_vel;
  // handle state operations
  // one of these states need to set cmd_vel to something if you want to move
  switch(state) {
    case States::IDLE:
      break;
    case States::GO:
      cmd_vel.linear.x = 0.1;
      break;
    case States::DRIVE_ODOM:
      {
        if (goals_odom.empty()) {
          ROS_INFO("DRIVE_ODOM: No goals");
          break;
        } else if (driveToGoal(goals_odom.front(), cmd_vel)) {
          ROS_INFO("DRIVE_ODOM: arrived");
          goals_odom.pop();
        }
      }
      break;
    case States::DRIVE_UTM:
      {
        if (goals_utm.empty()) {
          ROS_INFO("DRIVE_UTM: No goals");
          break;
        } else if (driveToGoal(utmToOdom(goals_utm.front()), cmd_vel)) {
          ROS_INFO("DRIVE_UTM: arrived");
          goals_utm.pop();
        }
      }
      break;
  }
  vel_pub.publish(cmd_vel); 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "truck_base_node");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odometry/filtered", 1, odomCallback);
  ros::Subscriber utm_sub = nh.subscribe("odom_utm", 1, utmCallback);
  ros::Subscriber goal_odom_sub = nh.subscribe("goal_odom_cmd", 1,
      goalOdomCallback);
  ros::Subscriber goal_utm_sub = nh.subscribe("goal_utm_cmd", 1, 
      goalUtmCallback);
  ros::Subscriber cmd_sub = nh.subscribe("cmd", 1, cmdCallback);
  state_start_time = ros::Time::now();
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  state_machine_timer = nh.createTimer(ros::Duration(0.1), 
      stateMachineCallback);
  ros::spin();
  return 0;
}

/*** load a kml file as utm goals */
void loadKMLGoalFile(const std::string &goal_filename) {
  ROS_INFO_STREAM("Loading goal file: "<< goal_filename); 
  std::ifstream inf(goal_filename);
  if(!inf.good()) {
    ROS_WARN_STREAM("Cannot read file " << goal_filename);
    return;
  }
  std::string line;
  geometry_msgs::Pose2D goal;
  geographic_msgs::GeoPoint geo;
  geodesy::UTMPoint utm;
  while(std::getline(inf, line)) {
    if(line.substr(0,10) != "<gx:coord>") continue;
    std::stringstream ss(line.substr(10));
    double lon, lat, alt;
    ss >> lon >> lat >> alt;
    if (!ss.fail()) {
      geo = geodesy::toMsg(lat, lon, alt);
      geodesy::fromMsg(geo, utm);
      goal.x = utm.easting;
      goal.y = utm.northing;
      goals_utm.push(goal);
      ROS_INFO_STREAM(goal);
    }
  }
  ROS_INFO_STREAM("Done Loading goal file: "<< goal_filename); 
  return;
}
/*** load an odom waypoint (whitespace separated) file as goals */
void loadOdomGoalFile(const std::string &goal_filename) {
  ROS_INFO_STREAM("Loading Odom goal file: "<< goal_filename); 
  std::ifstream inf(goal_filename);
  if(!inf.good()) {
    ROS_WARN_STREAM("Cannot read file " << goal_filename);
    return;
  }
  std::string line;
  geometry_msgs::Pose2D goal;
  while(std::getline(inf, line)) {
    std::stringstream ss(line);
    ss >> goal.x >> goal.y;
    if (!ss.fail()) {
      goals_odom.push(goal);
      ROS_INFO_STREAM(goal);
    }
  }
  return;
}
