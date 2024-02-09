## Software Setup Instructions

1. **Create Workspace:**
   - Create a workspace for the software.

2. **Copy Folders:**
   - Copy the provided folders into the `src` folder of the workspace.

3. **Build:**
   - Build the workspace.

4. **Install Packages:**
   - Install the following packages:
     ```bash
     sudo apt install ros-noetic-teleop-twist-joy
     sudo apt-get install ros-kinetic-hector-slam
     sudo apt install python3-serial
     ```

5. **Launch Modules:**
   - Launch all the modules using the command:
     ```bash
     roslaunch roboclaw_node launch_all.launch
     ```

6. **Troubleshooting:**
   - If encountering errors like `AttributeError: Roboclaw instance has no attribute '_port'` or `Unable to read version`, try:
     ```bash
     sudo adduser <USER> dialout
     ```
     or
     ```bash
     sudo chmod 666 /dev/ttyACM0
     sudo chmod 666 /dev/ttyACM1
     ```

7. **Further Troubleshooting:**
   - If issues persist, consider using BasicMicroMotion Studio software (Windows) to verify addresses and configurations.

8. **Parameter Configuration:**
   - To change parameters, modify the `launch_all.launch` file located at `ws/src/roboclaw_ros/roboclaw_node/launch/`.

9. **Source Bash:**
   - Always remember to source the bash by running:
     ```bash
     source devel/setup.bash
     ```
     every time you open a new bash session.
