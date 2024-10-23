# README

## How to Launch the Gazebo Simulation

1. Create a ROS 2 workspace if you haven't done so already.
   ```bash
   mkdir -p dev_ws/src
   ```
2. Clone this repo into the `src` folder
   ```bash
   cd dev_ws/src
   git clone git@github.com:packbionics/assistive_brace.git
   ```
3. Navigate to the root of the workspace folder `dev_ws` and build the `walker_description` package
   ```bash
   cd ..
   colcon build --packages-select walker_description
   ```
4. Source the workspace
   ```bash
   . install/setup.bash
   ``` 
6. Run the `launch_walker.launch.py` launch file
   ```bash
   ros2 launch walker_description launch_walker.launch.py
   ```
   **Note** If you wish to run the simulation with GUI toggled on, run the command with the argument `gui:=true`
   ```bash
   ros2 launch walker_description launch_walker.launch.py gui:=true
   ```
