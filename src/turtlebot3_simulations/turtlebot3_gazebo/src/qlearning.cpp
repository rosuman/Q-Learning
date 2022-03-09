#include "turtlebot3_gazebo/qlearning.hpp"

#include <memory>
#include <random>
#include <fstream>

using namespace std::chrono_literals;


QLearning::QLearning()
:Node("Q_learning_node") {
    alpha = .5; // Determines to what extent newly acquired information overrides old information.  
    gamma = .8;
    episode = 0;
    Q[8][3] = {0};
    epsilon = 0.70;
    scan_data_[0] = 0;
    scan_data_[1] = 0;
    scan_data_[2] = 0;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

      // Initialise subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&QLearning::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(&QLearning::odom_callback, this, std::placeholders::_1));
    update_timer_ = this->create_wall_timer(250ms, std::bind(&QLearning::train_callback, this));
    
    resetWorld = this->create_client<std_srvs::srv::Empty>("/reset_world");
    resetSimulation = this->create_client<std_srvs::srv::Empty>("/reset_simulation");

    RCLCPP_INFO(this->get_logger(), "QLearing simulation node has been initialised");
}

void QLearning::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
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

void QLearning::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

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
 // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left '%f', centre '%f', right '%f'", scan_data_[1],scan_data_[0],scan_data_[2]);
}

void QLearning::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

int QLearning::findState(bool left,bool centre, bool right){
    if (!left && !centre && !right){
        state = 0; // no obstacle
    }else if (!left && !centre && right){
        state = 1; // only right obs
    }else if (!left && centre && !right){
        state = 2; // only centre obs
    }else if (left && !centre && !right){
        state =  3; // only left obs
    }else if (!left && centre && right){
        state = 4; // centre and right obs
    }else if (left && !centre && right){
        state = 5; // left and right obs
    }else if (left && centre && !right){
        state = 6; // left and centre
    }else {
        state = 7; // all obs
    }
    return state;
}


int QLearning::decideAction(int state){
    int action;
    int Rand = rand() % 100;
    if (Rand < epsilon*100) {
        int randAction = rand() % 3;  // random number between 0 and 2
        action = randAction;
    } else {
        int bestAction;  // initiate best action var
        int bestQ = -100000;  // initiate best Q value, this corresponds to a state action pair value

        for (int i = 0; i < 3; i++) {  // cycle through every action for current state
            if (Q[state][i] > bestQ) {  // if state action pair Q value is greater than current best value
                bestQ = Q[state][i];  // set new highest Q value
                bestAction = i;  // set new best action
            }
        }
    action = bestAction;
    }
    return action;
    actionPerformed = false;

}
/*
int QLearning::assignReward(int prevAction, bool crash) {
  int reward;
  if (prevAction == 0) {  // if moved forward reward since we want to explore
    reward = 10;
  }
  if (prevAction == 1 || prevAction == 2) {  // if turn thats neutral, not bad but not exploring
    reward = 0;
  }
  if (crash)  // if crashed give big negative reward
    reward = -50;
  return reward;
}
*/
//                   no       right     centre      left centre+right left+right left+centre   all  
int R[8][3] = {{30, -1, -1}, {-1, -1, 30},{-1, 30, -1}, {-1,30,-1},{-1, -1, 30},{30,-1,-1},{-1, 30, -1},{-1,30,-1}} ;

int QLearning::assignReward(int prevAction, int prevState, bool crash) {
  int reward;
  if (crash) {
    // if crashed give big negative reward
    reward = -50;
  } else {
    reward = R[prevState][prevAction];
  }

  return reward;
}

int QLearning::maxFuture(int state) {
  int currentMax = -100000;  // initiate var to keep track of max

  for (int i = 0; i != 3; i++) {  // cycle through every action
    if (Q[state][i] > currentMax)  // if state action pair Q value greater than current Q value
      currentMax = Q[state][i];  // set new max Q value
  }
  return currentMax;
}

void QLearning::updateTable(int prevAction, int prevState, bool crash, int state) {
double reward = assignReward(prevAction, prevState, crash);  // get the reward based on previous action and if crash

  // Update Q value using equation. Based on old Q value, reward, max Q value of new state, and constants alpha, gamma
  if (crash == true) {  // if crash, the new state doesn't count, it crashed
    Q[prevState][prevAction] = Q[prevState][prevAction] + alpha * (reward - Q[prevState][prevAction]);
  } else {  // no crash, full equation used
  Q[prevState][prevAction] = Q[prevState][prevAction]+ alpha * (reward + (gamma * maxFuture(state))- Q[prevState][prevAction]);
  }
  QLearning::printTable();
}

void QLearning::printTable() {
  std::ofstream QTable("qtable.txt");
  for (int i=0; i<8; i++){
    QTable << Q[i][0] << "  " << Q[i][1] << "  " << Q[i][2] << "\n";
  }
  QTable.close();

}

int firstRun = 0;  // Can't update Q table on first run, no previous data to go off of
int prevState = 0;
int prevAction =0;

void QLearning::train_callback() {
    int state = findState(left, centre, right);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "current state '%d'", state);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "firstRun '%d'", firstRun);
    if (firstRun==0) {
      // Decide action based on current state. Picks either random action or uses table to decide
        int action = decideAction(state);  // decide action based on state
       // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "action", action);
        performAction(action);
        crash = ((scan_data_[0]<0.4 || scan_data_[1]<0.2)|| scan_data_[2]<0.2);
        if (actionPerformed){
            prevState = state;    // set prevstate = state before next iteration
            prevAction = action;
        }
        
        firstRun = 1;
    } else if(firstRun>0){
        if(crash){
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "prevState : '%d', prevAction : '%d' ", prevState, prevAction);
          state = findState(left, centre, right);
          updateTable(prevAction, prevState, crash, state);
          episode++;
          epsilon = epsilon*0.98;
          firstRun = 0;
          //reset world and reset similution
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "EPISODE '%d' ENDED", episode);
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Simulation reset DONE! ");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Epsilon : '%f' ", epsilon);
          resetWorld->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
          resetSimulation->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        } else {
          state = findState(left, centre, right);
          int action = decideAction(state);
          performAction(action);
          crash = ((scan_data_[0]<0.4 || scan_data_[1]<0.2)|| scan_data_[2]<0.2);
          if (actionPerformed){
              prevState = state;    // set prevstate = state before next iteration
              prevAction = action;
          }

          state = findState(left, centre, right);
          updateTable(prevAction, prevState, crash, state);

        }
    }
}

void QLearning::performAction(int action) {
    switch (action)
    {
    case 0:
        // go forward
        update_cmd_vel(LINEAR_VELOCITY, 0.0);
        actionPerformed = true;
        break;
    case 1:
        // turn right in place

        update_cmd_vel(-1*LINEAR_VELOCITY/3, -1 * ANGULAR_VELOCITY);
        actionPerformed = true;
        break;
    case 2:
        // turn left in place
        update_cmd_vel(-1*LINEAR_VELOCITY/3, ANGULAR_VELOCITY);
        actionPerformed = true;
        break;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QLearning>());
  rclcpp::shutdown();

  return 0;
}