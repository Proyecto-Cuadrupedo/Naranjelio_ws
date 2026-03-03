export NARANJELIO_HOME=$PWD
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro naranjelio.urdf.xacro)" -p use_sim_time:=true
