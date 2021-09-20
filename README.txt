cd ~/catkin_ws/src

# 依存パッケージのインストール
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-navigation

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

