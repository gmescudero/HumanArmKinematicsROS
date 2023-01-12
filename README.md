# HumanArmKinematicsROS
Package to contain Human Arm Kinematics library into a ROS pack. 

The pack requires some sumbmodules so use ```git clone --recurse-submodules``` to clone the repository

## Open Zen library for LPMS IMU sensors
The OpenZen library is required for some functionality in the package. To install it use the following command:
```
apt install ros-<ros_version>-openzen-sensor
```
For more details go to https://bitbucket.org/lpresearch/openzenros/src/master/