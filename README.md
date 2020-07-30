airsim_ros packages
===================

AirSim ROS packages: modified ros wrapper for airsim, and some vslam related tools. (Mainly focused on Car SimMode)

Package overview
----------------

* `airsim_ros_pkgs`: Modified ros wrapper for airsim
* `airsim_car_teleop`: A keyboard teleop ros node to control the simulating Car in airsim
* `settings`: Vslam simulation tests setting files
* `simu_tools`: Simulation related tools for handling the trajectory ground truth

Features
--------

* Campared with the original [ROS node](https://github.com/microsoft/AirSim/pull/2743), the stereo images timestamps sync problem is solved([ref](https://github.com/xuhao1/airsim_ros_pkgs)). Now the timestamp difference between a left and a right image is at max 0.003 second (ubuntu 16.04, 32G, ros-kinetic, UE 4.24, no GPU). 

```bash
### A ROS node for the Car SimMode, according to the original repo of microsoft/AirSim:
roslaunch airsim_ros_pkgs airsim_car_with_joy_control.launch

### A self-written ROS node for the Car SimMode (No longer maintained, since microsoft/AirSim supports Car ROS node now)
roslaunch airsim_ros_pkgs airsim_car_node.launch
```

* If you do not have a joy control hardware, here is a keyboard control node for the Car simulated with the ROS node above. Usage:

```bash
roslaunch airsim_car_teleop airsim_car_teleop_key.launch
```

* Simulation tools are provided to convert the car trajectory ground truth rostopic to a txt file (TUM), for evaluation postprocessing purpose. 

```bash
### Either use the pose recorder node to record the ground truth during simulation: 
roslaunch simu_tools car_pose_recorder output_filename rostopic_name

### Or use the python script /simu_tools/scripts/groundtruth_extractor.py to extract the ground truth from a rosbag. 
./groundtruth_extractor.py
```