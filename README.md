# mcl_ros

## About mcl_ros

mcl_ros is a ROS package for mobile robot localization with 2D LiDAR. To implement localization, Monte Carlo localization (MCL) is used.

I confirmed that mcl_ros works on Ubuntu 18.04 and 20.04.





## How to install

You need to install ROS environment first. Then, install ros_mcl as

~~~
$ git clone https://github.com/NaokiAkai/mcl_ros.git
$ cd mcl_ros
$ catkin_make
~~~





## How to run

You need to publish sensor_msgs::LaserScan, nav_msgs::Odometry, and nav_msgs::OccupancyGrid messages. Default topic names are /scan, /odom, and /map.

In addition, you need to set static transform publisher between the base link to laser sensor frames. Default frame names are /base_link and /laser.

Then, run mcl_ros as

~~~
$ cd mcl_ros
$ source devel/setup.bash
$ roslaunch mcl_ros mcl.launch
~~~





## Parameter descriptions

There is a launch file under mcl_ros/src/mcl_ros/launch/ directory, named mcl.launch. The parameter descriptions can be seen in the launch file.





## Characteristics

### Robust localization

mcl_ros contains the localization method presented in [1]. The localization method simultaneously estimates the robot pose and sensor measurement classes. Here, two sensor measurements classes are considered; known and unknown, that is, mapped and unmapped obstacles. Because the method can simultaneously estimate the unknown measurement while localization, the localization robustness to unknown obstacles can be increased.

To use the localization method, measurement_model_type is needed to be set to 2.

~~~
$ roslaunch mcl_ros mcl.launch measurement_model_type:=2
~~~



### Adding random particles in resampling

Adding random particles in resampling is a simple method to improve localization robustness. mcl_ros uses this method.

To use the method, use_augmented_mcl or add_random_particles_in_resampling is needed to be set to true.

~~~
$ roslaunch mcl_ros mcl.launch use_augmented_mcl:=true
~~~

or

~~~
$ roslaunch mcl_ros mcl.launch add_random_particles_in_resampling:=true
~~~

If use_augmented_mcl is true, mcl_ros works as augmented MCL [2]. In my experience, I really recommend to set add_random_particles_in_resampling to true, instead of use_augmented_mcl.

### 

### Rejecting unknown scan

In [2], a method to reject scans that might measure unknown obstacles is presented. To reject the unknown scan, the beam model [2] is utilized. The unknown scan rejection method is implemented in mcl_ros.

To use the unknown scan rejection, reject_unknown_scan is needed to be set to true.

~~~
$ roslaunch mcl_ros mcl.launch reject_unknown_scan:=true
~~~

Note that the unknown scans are automatically detected if the method presented in [1] is used since it simultaneously estimate the unknown class measurements.



## 

## Reference

[1] Naoki Akai, Luis Yoichi Morales, and Hiroshi Murase. "Mobile robot localization considering class of sensor observations," In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3159-3166, 2018. 

[2] Sebastian Thrun, Wolfram Burgard, and Dieter Fox. "Probabilistic robotics," *MIT Press*, 2005.





## License

Mozilla Public License Version 2.0
