alias _fan='sudo sh -c "echo 125 > /sys/devices/pwm-fan/target_pwm"'

alias _rsviewer='~/librealsense/build/tools/realsense-viewer/realsense-viewer'
alias _rsrec='~/librealsense/build/tools/recorder/rs-recorder'
alias _samples='cd ~/workspace_ezemskov/collision-avoidance-library/build/samples/'

_setup_ros() 
{
   source /home/evgenium/ros2_foxy/install/setup.bash 2>/dev/null
   source /home/evgenium/nav2_foxy/install/setup.bash 2>/dev/null
   source /home/evgenium/workspace_ezemskov/cam_ws/install/setup.bash 2>/dev/null
   source /home/evgenium/realsense_ros/install/local_setup.bash 2>/dev/null
   source /opt/ros/foxy/share/gazebo_ros_pkgs/local_setup.bash

   export PYTHONPATH="$PYTHONPATH:/home/evgenium/workspace_ezemskov/cam_ws/src/rcam_ros2/rcam_ros2/rcam_ros2/"
}

alias _rcam='ros2 launch rcam_ros2 rcam.launch.py'
alias _nav='ros2 launch rcam_ros2 navigation.launch.py'
alias _static_odom='ros2 run rcam_ros2 static_odom'
alias _chassis='ros2 run rcam_ros2 chassis'
alias _sim='ros2 launch rcam_sim world.launch.py'

alias _subscriber_cpp='~/workspace_ezemskov/george/rcam_subscriber_cpp/build/rcam_subscriber_cpp'