# Q-Learning

This is the implementiaon of Reinforcement learning, particularly Q learning to train the two wheeled robot to nevigate in the environment with obstacles.
ROS2 foxy and gazebo is used for this project.

The robot used is [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) burger model. The goal of the robot is to learn the appropeiate actions based on its state to avoid the obstacles.

Once the robot is trained to avoid the obstacles, the final Q Table is used to make a hybrid go-to-goal and obstacle-avoidance algorithm to nevigate into the environment to reach the goal by avoiding obstacles.

To run the code, make sure turtlebot3 package and all its dependencies are installed. This can be done by going [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). Select 'foxy' and run the appropriate commands. Or watch [this](https://www.youtube.com/watch?v=8w3xhG1GPdo) how to do.

