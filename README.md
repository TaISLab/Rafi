## Software Setup Instructions

1. **Create Workspace:**
   - Create a ROS workspace for the software (ROS NOETIC).

2. **Copy Folders:**
   - Copy the provided folders into the `src` folder of the workspace.

3. **Build:**
   - Build the workspace.

4. **Install Packages:**
   - Install the following packages:
     ```bash
     sudo apt install ros-noetic-teleop-twist-joy
     sudo apt-get install ros-noetic-hector-slam
     sudo apt install python3-serial
     ```
5. **Activate LiDAR:**
   - The LiDAR folder is based on this blog https://archit0994.wixsite.com/architshah/post/manage-your-blog-from-your-live-site. So in order to activate the port you should follow the next steps after building it:
  
      - Add the ydlidar environment variable to the ~/.bashrc file and make it effective.

      - Add a device alias /dev/ydlidar to the X4 serial port.

6. **Launch Modules:**
   - Run roscore in a new bash:
     ```bash
     roscore
     ```
   - And in another bash launch all the modules using the command:
     ```bash
     roslaunch roboclaw_node launch_all.launch
     ```
     - Always remember to source the bash by running:
     ```bash
     source devel/setup.bash
     ```
     every time you open a new bash session.


7. **Troubleshooting:**
   - If encountering errors like `AttributeError: Roboclaw instance has no attribute '_port'` or `Unable to read version`, try:
     ```bash
     sudo adduser <USER> dialout
     ```
     or
     ```bash
     sudo chmod 666 /dev/ttyACM0
     sudo chmod 666 /dev/ttyACM1
     ```

8. **Further Troubleshooting:**
   - If issues persist, consider using BasicMicroMotion Studio software (Windows) to verify addresses and configurations.

9. **Parameter Configuration:**
   - To change parameters, modify the `launch_all.launch` file located at `ws/src/roboclaw_ros/roboclaw_node/launch/`.
