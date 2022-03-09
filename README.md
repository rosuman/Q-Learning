# Q-Learning

This is the implementiaon of Reinforcement learning, particularly Q learning to train the two wheeled robot to nevigate in the environment with obstacles.
ROS2 foxy and gazebo is used for this project.

The robot used is [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) burger model. The goal of the robot is to learn the appropeiate actions based on its state to avoid the obstacles.

Once the robot is trained to avoid the obstacles, the final Q Table is used to make a hybrid go-to-goal and obstacle-avoidance algorithm to nevigate into the environment to reach the goal by avoiding obstacles.

To run the code, make sure turtlebot3 package and all its dependencies are installed. This can be done by going [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). Select 'foxy' and run the appropriate commands. Or watch [this](https://www.youtube.com/watch?v=8w3xhG1GPdo) how to do.

* From the home directory <code>name@ubuntu:~$</code> clone the ropository:
<code>git clone https://github.com/rosuman/Q-Learning.git </code>

<code> cd Q-Learning </code>

* Build the package:
 
  <code>colcon build --symlink-install</code>

  <code> . install/local_setup.bash </code>
  
  <code> export TURTLEBOT3_MODEL=burger </code>

* Now launch the training environment in gazebo:
  
  <code> ros2 launch turtlebot3_gazebo training.launch.py </code>

* Next run the q_learning node:
 
    for that open the new terminal and type
    
  <code> . install/local_setup.bash </code>
  
  <code> export TURTLEBOT3_MODEL=burger </code>
  
  <code> ros2 run turtlebot3_gazebo q_learning </code>

* After training quit the q_learning node as well as training environment of gazebo and launch demo environment along with run the run_demo node:

From the first terminal 

  <code> ros2 launch turtlebot3_gazebo demo.launch.py </code>
  
From second terminal
  
  <code> ros2 run turtlebot3_gazebo run_demo </code>
 


