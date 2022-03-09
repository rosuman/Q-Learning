#ifndef QCLASS_H_ // NOLINT
#define QCLASS_H_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/empty.hpp>
//#include <std_msgs/msg/string.hpp>

#define LINEAR_VELOCITY  0.2
#define ANGULAR_VELOCITY 1.5

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class QLearning : public rclcpp::Node
{
public:
  QLearning();

  int state;
  int action;
  int reward;
  int Q[8][3];
  int episode;
  double robot_pose_;
  double alpha;
  double gamma;
  double epsilon;
  double scan_data_[3];
  bool left, centre, right;
  bool crash;
  bool actionPerformed;
  int findState(bool left,bool centre, bool right);
  int decideAction(int state);
  int assignReward(int prevAction, int prevState, bool crash);
  int maxFuture(int state);
  void updateTable(int prevAction, int prevState, bool crash, int state);
  void performAction(int action);
  void printTable();


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

  // Function prototypes
  void train_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif /* QCLASS_H_ */ 