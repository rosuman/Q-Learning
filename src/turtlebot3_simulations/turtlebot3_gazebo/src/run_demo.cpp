
#include "turtlebot3_gazebo/run_demo.hpp"

#include <memory>
#include <random>
#include <fstream>

using namespace std::chrono_literals;
using namespace std;

RunDemo::RunDemo()
:Node("Run_demo_node") {
    QT[8][3] = {0};
    scan_data_[0] = 0.0;
    scan_data_[1] = 0.0;
    scan_data_[2] = 0.0;
    A[8] = {0};
    x = 5.0;
    y = 5.0;
    flag = 1;
    read = false;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

      // Initialise subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&RunDemo::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(&RunDemo::odom_callback, this, std::placeholders::_1));
    update_timer_ = this->create_wall_timer(200ms, std::bind(&RunDemo::update_callback, this));
    
    //resetWorld = this->create_client<std_srvs::srv::Empty>("/reset_world");
    //resetSimulation = this->create_client<std_srvs::srv::Empty>("/reset_simulation");

    RCLCPP_INFO(this->get_logger(), "Run_domo node has been initialised");
}

void RunDemo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

    cur_pos.x = msg->pose.pose.position.x;
    cur_pos.y = msg->pose.pose.position.y;
    cur_pos.z = msg->pose.pose.position.z;

    err_pos = sqrt((pow(x-cur_pos.x,2)+pow(y-cur_pos.y,2)));
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "err_pos : '%f' ", err_pos);
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
    robot_pose_ = yaw;
}

void RunDemo::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
  centre = scan_data_[0]<0.5;
  left = scan_data_[1]<0.4;
  right = scan_data_[2]<0.4;

  obsC = scan_data_[0]<0.8;
  obsL = scan_data_[1]<0.8;
  obsR = scan_data_[2]<0.8;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left '%f', centre '%f', right '%f'", scan_data_[0],scan_data_[1],scan_data_[2]);
}

void RunDemo::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

int RunDemo::findState(bool left,bool centre, bool right){
    if (!left && !centre && !right){
        state = 0;
    }else if (!left && !centre && right){
        state = 1;
    }else if (!left && centre && !right){
        state = 2;
    }else if (left && !centre && !right){
        state =  3;
    }else if (!left && centre && right){
        state = 4;
    }else if (left && !centre && right){
        state = 5;
    }else if (left && centre && !right){
        state = 6;
    }else {
        state = 7;
    }
    return state;
}

int RunDemo::decideAction(int state){
    int action;
    action = A[state];
    return action;
}

void RunDemo::update_callback() {

    des_yaw = atan2((y-cur_pos.y), (x - cur_pos.x));
    err_yaw = (des_yaw - robot_pose_);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current yaw '%f', des_yaw '%f'", robot_pose_*RAD2DEG, des_yaw*RAD2DEG);

    if (!read) {
        readTable();
    }

    if (err_pos> 0.2){
        if (obsC || obsL || obsR || flag != 0){
            int state = findState(left, centre, right);
            int action = decideAction(state);
            performAction(action);
            flag++;
            if(flag == 2){
                flag = 0;
            }
        } else {  
            if (abs(err_yaw) > 0.3){
                fixYaw();
            } else {
                int state = findState(left, centre, right);
                int action = decideAction(state);
                performAction(action);
            }
        }
    }else {
        update_cmd_vel(0.0, 0.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal REACHED!");
    }

       
}

void RunDemo::performAction(int action) {
    switch (action)
    {
    case 0:
        // go forward
        update_cmd_vel(LINEAR_VELOCITY, 0.0);
        break;
    case 1:
        // turn right in place

        update_cmd_vel(-1*LINEAR_VELOCITY/3, -1 * ANGULAR_VELOCITY);
        break;
    case 2:
        // turn left in place
        update_cmd_vel(-1*LINEAR_VELOCITY/3, ANGULAR_VELOCITY);
        break;
    }
}

void RunDemo::fixYaw() {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fixing yaw");

        if (err_yaw<0 && err_yaw> -3.149 + des_yaw){
            update_cmd_vel(0.0, -1*ANGULAR_VELOCITY);
        } else if (err_yaw>3.1459 && err_yaw<3.1459+des_yaw){
            update_cmd_vel(0.0, -1*ANGULAR_VELOCITY);
        } else {
            update_cmd_vel(0.0, ANGULAR_VELOCITY);
        }

        flag = 1;

}

void RunDemo::readTable() {
    std::ifstream inputfile;
    inputfile.open("qtable.txt");
    for (int i =0; i<8; i++){
        for (int j=0; j<3; j++){
            inputfile >> QT[i][j];
        }
    }
    for (int i =0; i<8; i++){
        for (int j=0; j<3; j++){
            cout << QT[i][j] << " ";
        }
        cout << "\n";
    } 

    
    for (int i = 0; i<8; i++){
        int max = 0;
        for (int j = 0; j<3; j++){
            if (QT[i][j]>=max){
                max = QT[i][j];
                A[i] = j;
            }
        }
    }

    for (int i = 0; i<8; i++){
        cout << A[i] << "\n";
    }
    read = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RunDemo>());
  rclcpp::shutdown();

  return 0;
}