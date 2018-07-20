# Realtime test examples

This package contains PointCloud2 message publish and subscribe. this example used to test ROS2 big data connunication performance.


## build
```
cd ros2_ws/src
git clone https://github.com/yechun1/rttest_sample.git
cd ros2_ws
ament build --isolated
```

## run test
```
source install_isolated/local_setup.bash
# terminal 1
ros2 run rttest_example publisher
# terminal 2
ros2 topic echo /rttest_sample
```

