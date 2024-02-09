For use this software, you will need to create a workspace,
and then copy this folders into the src folder and build.

Then install the following packages:  
  sudo apt install ros-noetic-teleop-twist-joy  
  sudo apt-get install ros-kinetic-hector-slam  
  sudo apt install python3-serial  
  
And for launching all the modules: roslaunch roboclaw_node launch_all.launch  
  
If you are facing these problems: AttributeError: Roboclaw instance has no attribute '_port' or Unable to read version  
try:  
  sudo adduser <USER> dialout  
or:  
  sudo chmod 666 /dev/ttyACM0  
  sudo chmod 666 /dev/ttyACM1  
  
If it keeps failing try BasicMicroMotion Studio software (Windows) to check the addresses and configurations.  
  
If you want to change parameters, you can change it the launch_all.launch file (ws/src/roboclaw_ros/roboclaw_node/launch/)  
  
DO NOT FORGET TO SOURCE THE BASH: source devel/setup.bash every time you open a new bash  
