source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/developer/robocross.dune/ros2_ws/install/setup.bash
export DISPLAY=:0
export USER=bmstu
export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;32m\]\w\[\033[00m\]\$ '
alias cb='cd /home/developer/robocross.dune/ros2_ws && colcon build --symlink-install && source /home/developer/robocross.dune/ros2_ws/install/setup.bash'
alias tl='ros2 topic list'
alias cw='cd /home/developer/robocross.dune/ros2_ws'
alias sim='ros2 launch simulation sim.launch.py'

export ROS_DOMAIN_ID=8 #231
export QT_QPA_PLATFORM=xcb
alias start="ros2 launch main rsp.launch.py"
export DISPLAY=:0
alias nav="ros2 launch main navigation.launch.py"

alias pause='ros2 topic pub --once /goal_status std_msgs/msg/String "{data: \"pause\"}"' 
alias stop='ros2 topic pub --once /goal_status std_msgs/msg/String "{data: \"stop\"}"'
alias move='ros2 topic pub --once /goal_status std_msgs/msg/String "{data: \"moving\"}"'

alias joy='ros2 launch main joy.launch.py'
