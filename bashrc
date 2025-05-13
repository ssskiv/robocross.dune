source /opt/ros/${ROS_DISTRO}/setup.bash
source /bmstu/ros2_ws/install/setup.bash
export DISPLAY=:0
export PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;32m\]\w\[\033[00m\]\$ '
alias cb='cd ~/ros2_ws && colcon build --symlink-install'
alias tl='ros2 topic list'
alias cw='cd ~/ros2_ws'
alias slm='ros2 launch main sim.launch.py'