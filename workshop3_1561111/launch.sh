source ~/.bashrc
rm -rf build devel
catkin_make
source devel/setup.bash
roslaunch jetson_camera everything.launch
