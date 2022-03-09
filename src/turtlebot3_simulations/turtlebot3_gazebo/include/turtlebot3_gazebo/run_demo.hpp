#ifndef RUN_DEMO_HPP_ // NOLINT
#define RUN_DEMO_HPP_


#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/empty.hpp>

#define LINEAR_VELOCITY  0.2
#define ANGULAR_VELOCITY 1.5

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class RunDemo : public rclcpp::Node 
{
public:
  RunDemo ();
  int state;
  int action;
  int QT[4][3];
  double scan_data_[3];
  bool left, centre, right;
  bool obsL, obsC, obsR;
  // bool crash;
  double robot_pose_;
  geometry_msgs::msg::Point  cur_pos;
  double des_yaw;
  double err_pos , err_yaw;
  double x, y;
  int flag;
  bool read;
  int A[8];

  int findState(bool left, bool centre, bool right);
  int decideAction(int state);
  void readTable();


private:

  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Gazebo reset client
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resetWorld;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resetSimulation;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;


  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void performAction(int action);
  void fixYaw();
};

#endif /* RUN_DEMO_HPP_ */ 