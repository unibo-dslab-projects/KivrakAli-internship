source /opt/ros/humble/setup.zsh
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.zsh
source $HOME/unitree_ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
