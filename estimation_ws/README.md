### Preliminaries
- Install ROS Kinetic
- Install ardrone_autonomy package

``` bash
sudo apt install ros-kinetic-ardrone-autonomy
```
- We can access IMU and camera data by launching the driver with ardrone.launch
- Make sure to be on the drone's network and your IP address in the launch file is set properly.

- Launch `roscore`.
- Launch `roslaunch ardrone.launch`
- To read IMU data, `rostopic echo /ardrone/imu`
- To read magnetometer data, `rostopic echo /ardrone/mag`
- To read front camera images, `rosrun image_view image_view image:=/ardrone/front_camera/raw_image`

### Using this workspace
- Install dependency `iDynTree` from `devel` branch. Follow the installation instructions at https://github.com/robotology/idyntree/tree/devel
- Add the iDynTree install folder to `CMAKE_PREFIX_PATH`

``` bash
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/path/where/you/installed/iDynTree
```

- Make the `Params.cfg` file existing in `attitude_estimation_ws/src/imu_orientation_estimator/cfg` folder executable.
``` bash
sudo chmod a+x Params.cfg
```
- To build packages, run `catkin_make` in `attitude_estimation_ws` folder.
- To avoid sourcing the files everytime, add the `attitude_estimation_ws/src` to `ROS_PACKAGE_PATH`
``` bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/path/to/coding_playground/attitude_estimation_ws/src
```

- Run `dynamic_reconfigure` to change parameters of the filter online.
``` bash
rosrun rqt_reconfigure rqt_reconfigure
```

- Run `rqt_pose_view` through `rqt` and subscribe to the topics `/ground_truth/state`, `/mahony/state` and `/qekf/state` to visualize the orientation
