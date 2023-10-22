# ros-agent
This is a co-simulation project of AVP (Automated Valet Parking) based on CARLA and Autoware. Based on ROS2, the ros-agent retrieve sensor data from CARLA and transmit it to Autoware, then Autoware calculates the control command to pass it to CARLA to execute. 

![demo](./Docs/AVP2_short.gif) 

---
## Requirements
- Ubuntu20.04
- CPU Intel i7 9th or higher
- RAM 32GB or more
- GPU 8GB or more
## Installation
The overall installation includes four parts:
- CARLA installation
- ROS2 installation
- Autoware installation
- ros-agent bridge installation
### CARLA
Download the specified [CARLA version](http://gofile.me/6MDrn/UOvykgikS) with SUSTC_Parkinglot Map. The newest version is recommended.([BaiduCloud](https://pan.baidu.com/share/init?surl=iLcAsa1yJSYNcisP1ymfow&pwd=b5au) | [GoogleDrive](https://drive.google.com/file/d/1kHZM-gMMSq9b4UvzKXK5PkzfXLoXZmkx/view))

### ROS2
This project is based on ROS2 galactic. In order to install galactic, please follow the official [tutorial](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).
### Autoware
Autoware is the world's leading open-source software project for AD(Autonomous Driving).Autoware includes the necessary functions to drive an autonomous vehicles from localization and object detection to route planning and control.This project use Autoware galactic as the AD stack. 
1. Clone Autoware repository
```shell
mkdir ~/ros-agent
cd ~/ros-agent
git clone https://github.com/ROS-Agent/autoware.git -b feature/bev
```
2. Install dependencies using Ansible. You can also install dependecies manually, following this [website](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/).
```shell
cd ~/ros-agent/autoware
./setup-dev-env.sh
```
3. Create the src directory and clone repositories into it.
```shell
mkdir src
vcs import src < autoware.repos
```
4. Install dependent ROS packages.
```shell
rosdep update --include-eol-distros
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
5. Build the workspace.
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
### ROS-Agent bridge
The ROS-Agent brige is the data transmission bridge, which retrieves sensor data from CARLA to pass it to Autoware and receives Autoware control command to pass it to CARLA.
1. Clone repositories and Maps.
```shell
mkdir -p ~/ros-agent/bridge
cd ~/ros-agent/bridge
git clone https://github.com/ROS-Agent/op_bridge.git -b feature/bev
git clone https://github.com/ROS-Agent/op_agent.git -b AVP
git clone https://github.com/ROS-Agent/scenario_runner.git
git clone https://github.com/ROS-Agent/Maps.git -b feature/bev
```
2. Replace sensor config and launch files
```shell
cd ~/ros-agent/bridge/Maps
sudo mv gnss.launch.xml ~/ros-agent/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml
sudo mv sensor_kit_calibration.yaml ~/ros-agent/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensor_kit_calibration.yaml
sudo mv sensors_calibration.yaml ~/ros-agent/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensors_calibration.yaml
```
3. Set environment variables
```shell
gedit ~/.bashrc
# Add the following line to the bottom of the file.
export CARLA_ROOT=/path/to/CARLA/server/root/folder
export SCENARIO_RUNNER_ROOT=~/ros-agent/bridge/scenario_runner
export LEADERBOARD_ROOT=~/ros-agent/bridge/op_bridge
export TEAM_CODE_ROOT=~/ros-agent/bridge/op_agent
# Save the file and reset the terminal.
```

## Start the Simulation
1. Open one terminal
```sh
cd $CARLA_ROOT
./CarlaUE4.sh
```
2. Open another terminal
```sh
cd ~/ros-agent/autoware
source ./install/setup.bash
cd ~/ros-agent/bridge/op_bridge/op_scripts
./run_exploration_mode_ros2.sh
# open another terminal
cd ~/ros-agent/bridge/op_bridge/op_bridge
python3 pub_fake_occgrid.py
# open another terminal
cd ~/ros-agent/bridge/op_bridge/op_bridge
./reset_OccGrid_origin.sh
```
3. Set Vehilce goal pose
- (1) Click the `2D Goal Pose` button in the toolbar
- (2) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose. If done correctly, you will see a planned path from initial pose to goal pose
![](Docs/set_goal_pose.png)
4. Engage the vehicle by shell script.
```
cd ~/ros-agent/bridge/op_bridge/op_bridge
./engage_sh.sh
```
5. Enjoy!!



## ROS-Agent Architecture
This is a ROS-Agent Architecture which describes data flow of this project.
![](./Docs/ROS-Agent-Architectue.png)
### How to create PCD Map
There are many ways to create PCD Map using CARLA simulation sensor data. In this project, we use the following two ways.
1. PCL Recorder. This is a CARLA official ROS Package to allows you to create point cloud maps from CARLA maps. Fore more information, please check this [webpage](https://carla.readthedocs.io/projects/ros-bridge/en/latest/pcl_recorder/).
2. LIO-SAM(SLAM). This method uses CARLA sensor data(LiDAR, IMU, GNSS) to SLAM. [Here](https://github.com/TixiaoShan/LIO-SAM/tree/ros2) is a real-time lidar-inertial odometry SLAM package. 

For more information about Map Creation, please check the Autoware Official website tutorial [guides](https://autowarefoundation.github.io/autoware-documentation/galactic/how-to-guides/creating-maps-for-autoware/).



### How to create HD Map
There are also many ways to create HD Map, but in order to satisfy Autoware's HD Map formap(extended `Lanelet2`), we use the official HD Map Tools [Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/). Here are some useful information for your reference:
1. Vector Map Builder [official tutorial](https://docs.web.auto/en/user-manuals/vector-map-builder/introduction).
2. Vector Map Builder short [video tutorial](https://www.bilibili.com/video/BV1Ku411f7xi/?spm_id_from=333.337.search-card.all.click&vd_source=a6a9cc0d52649040755e83284281ceb9).
3. Map Toolbox for Autoware, [a unity package tool](https://github.com/autocore-ai/MapToolbox).
4. [ASSURE mapping tools](https://github.com/hatem-darweesh/assuremappingtools). A desktop based tool for viewing, editing and saving road network maps for autonomous vehicle platforms such as Autoware.
5. OSM map editor [JOSM](https://learnosm.org/zh_CN/beginner/start-osm/).
6. [Autoware Lanelet2](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md) format details.


> **Tips**
> `Vector Map Builder` only supports ascii pcd map and do not support binary pcd map. You can use the `pcl_converter` command to complete the conversion between the two formats. 
```
pcl_converter -f ascii source.pcd dest.pcd
# if you have some problems, please check whether you have installed `pcl-tools`.
sudo apt install pcl-tools
```

## ROS-Agent Bridge topic interface
### Lidar
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/carla_pointcloud`                                   | [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud2.html)     | 3D Lidar sensor data|
### Camera
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/sensing/camera/traffic_light/camera_info`                                   | [sensor_msgs/msg/CameraInfo](https://docs.ros2.org/galactic/api/sensor_msgs/msg/CameraInfo.html)     | This message defines meta information for a camera.|
|`/sensing/camera/traffic_light/image_raw`|[sensor_msgs/msg/Image](https://docs.ros2.org/galactic/api/sensor_msgs/msg/Image.html)| This message contains an uncompressed image |
### IMU
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/sensing/imu/tamagawa/imu_raw`                                   | [sensor_msgs/msg/Imu](https://docs.ros2.org/galactic/api/sensor_msgs/msg/Imu.html)     | This is a message to hold data from an IMU (Inertial Measurement Unit)|
### GNSS
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/sensing/gnss/ublox/nav_sat_fix`                                   | [sensor_msgs/msg/NavSatFix](https://docs.ros2.org/galactic/api/sensor_msgs/msg/NavSatFix.html)     | Navigation Satellite fix for any Global Navigation Satellite System |
### Vehicle Velocity
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/vehicle/status/velocity_status`                                   | [autoware_auto_vehicle_msgs/msg/VelocityReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/VelocityReport.idl)     | Vechilce realtime velocity data |
### Vehicle SteerAngle
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/vehicle/status/steering_status`                                   | [autoware_auto_vehicle_msgs/msg/SteeringReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/SteeringReport.idl)     | Vechilce realtime steering angle data |
### Vehicle Control Mode
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/vehicle/status/control_mode`                                   | [autoware_auto_vehicle_msgs/msg/ControlModeReport](https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_vehicle_msgs/msg/ControlModeReport.idl)     | Contorl mode includes NO_COMMAND\AUTONOMOUS\AUTONOMOUS_STEER_ONLY\AUTONOMOUS_VELOCITY_ONLY\MANUAL\DISENGAGED\NOT_READY |
### Control_command
| Topic name                                      | Topic type                                       | Description                                |
| -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| `/control/command/control_cmd`                                   | [autoware_auto_control_msgs/msg/AckermannControlCommand](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_control_msgs/msg/AckermannControlCommand.idl)     | Lateral and longitudinal control message for Ackermann-style platforms |


