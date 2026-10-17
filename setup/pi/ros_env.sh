# ROS environment on the Pi, sourced by every session that runs nodes
source /opt/ros/jazzy/setup.bash
source ~/ros2_drivers_ws/install/setup.bash
source ~/drift-mpc-ackermann/ros2_ws/install/setup.bash

# Fast DDS over UDP only. A node that dies without a clean shutdown leaves its shared memory
# port behind, and a publisher still matched to it blocks up to ~0.35 s (docs/localization.md)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# faster discovery of new nodes (fastdds.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/drift-mpc-ackermann/setup/pi/fastdds.xml

# acados for building ackermann_nmpc (docs/nmpc.md); the node finds its libraries by RPATH
export ACADOS_SOURCE_DIR=$HOME/acados
