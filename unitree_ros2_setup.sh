#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
# source $HOME/unitree_ros2/src/cyclonedds_ws/install/setup.bash
source $HOME/unitree_ros2/install/local_setup.bash
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
#                            <NetworkInterface name="eth0" priority="default" multicast="default" />
#                        </Interfaces></General></Domain></CycloneDDS>'
