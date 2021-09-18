cd ~/catkin_ws/src

#gmappingのセットアップ
git clone https://github.com/ros-perception/slam_gmapping 

git clone https://github.com/ros-perception/openslam_gmapping.git

catkin build

#navigationのセットアップ
sudo apt install libbullet-dev libsdl-image1.2-dev libsdl-dev

cd ~/catkin_ws/src

git clone -b melodic-devel https://github.com/ros-planning/navigation.git

git clone -b ros1 https://github.com/ros-planning/navigation_msgs.git

git clone -b melodic-devel https://github.com/ros/geometry2.git

catkin build

