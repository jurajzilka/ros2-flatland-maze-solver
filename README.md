# ROS2 Flatland robot tutorial, using SLAM and NAV2

## Prerequisites
Ensure that you have ROS2 Humble installed. If not, please follow the official guide.

## SLAM Toolbox and Nav2 Stack Setup

### Updating .bashrc

1. Install the RMW Cyclone DDS implementation (in terminal):
  ```
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  ```
2. After successful installation, open the `.bashrc` file:
  ```
  gedit .bashrc
  ```
3. Add the following line to `.bashrc` (around line 119):
  ```
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
4. Save and close the file.

### Nav2 Installation
1. Install Nav2:
  ```   
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
  ```
### SLAM Toolbox Installation and Setup
1. Install the SLAM Toolbox:
  ```
  sudo apt install ros-humble-slam-toolbox
  ```
2. Navigate to `/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml`.
3. Change the parameters in the yaml file as follows:
  ```
  base_frame: base_link
  use_scan_matching: false
  use_scan_barycenter: false
  ```
4. Since the file is read-only, open it in the command line at `/opt/ros/humble/share/slam_toolbox/config` and change the parameters:
  ```
  sudo nano mapper_params_online_async.yaml
  ```
5. Save and close the file.

## Workspace and Flatland Setup for First Run

1. Open a terminal where you want your workspace folder.
2. Clone the repository:
  ```
  git clone https://github.com/jurajzilka/ros2-flatland-maze-solver
  ```
3. Change directory to the cloned repository:
  ```
  cd ros2-flatland-maze-solver/
  ```
4. Build the workspace:
  ```
  colcon build
  ```
5. Source the setup file:
  ```
  source install/setup.bash
  ```
NOTE: You have to source the setup file everytime you open a new terminal!

## SLAM Mapping

1. We will launch the robot, SLAM Toolbox, and controller, then save the map.

- First terminal: 
  ```
  ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml
  ```
- Second terminal: 
  ```
  ros2 launch nav2_bringup navigation_launch.py
  ```
- Third terminal: 
  ```
  ros2 launch slam_toolbox online_async_launch.py
  ```
- Fourth terminal: 
  ```
  ros2 run flatland_quick_start_ros2 custom_robot_controller.py
  ```

2. Wait for the robot to create the map. Then, in a new command line (opened in the ROS workspace), type:
  ```
  ros2 run nav2_map_server map_saver_cli -f src/flatland_quick_start_ros2/flatland_worlds/maze/scanned_map
  ```
3. If successful, you should see `scanned_map.pgm` and `scanned_map.yaml` in `src/flatland_quick_start_ros2/flatland_worlds/maze`.
4. Close all terminals with Ctrl+C.

## Navigation

1. Now that the map is known, you can make the robot move to any part of the maze.
- First terminal: Build and start the robot.
  ```
  colcon build
  ros2 launch flatland_quick_start_ros2 flatland_rviz.launch.xml
  ```
- Second terminal: Start Nav2.
  ```
  ros2 launch nav2_bringup navigation_launch.py
  ```
- Third terminal: Load the map.
  ```
  ros2 run nav2_util lifecycle_bringup map_server
  ```
- Fourth terminal: Run the second instance of Rviz.
  ```
  ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
  ```
- In this instance of Rviz, you can use the goal position to see the robot moving.

## Conclusion

Congratulations! You've successfully set up and navigated a Flatland robot using ROS2 Humble, SLAM Toolbox, and Nav2. This tutorial guided you through the installation of necessary components, configuration of the environment, mapping a maze using SLAM, and finally navigating the robot within that space.

### Next Steps
- Experiment with different parameters in the SLAM Toolbox and Nav2 to see how they affect the robot's navigation and mapping capabilities.
- Try designing your own mazes and see how the robot navigates through them.
- Explore additional ROS2 packages and tools to further enhance your robot's capabilities.

### Feedback
Your feedback is invaluable in improving this tutorial. If you have suggestions, questions, or comments, please feel free to reach out or contribute to the repository.

Thank you for following this tutorial, and happy robot programming!

