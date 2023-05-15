# AIIM Navigation Box

## Setting up the repository
### Cloning
```bash
git clone ssh://git@ssh.bitbucket.cms.navinfo.cloud:7999/aiim_ros/navigation_box.git
cd navigation_box
git submodule update --init --recursive
```

### Building
```bash
cd WORKSPACE_ROOT
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Sourcing
```bash
cd WORKSPACE_ROOT
source install/setup.bash
```

## Usage
### Basic Usage
To run the navigation box with default settings:
```bash
ros2 launch navigation_box navigation_box.launch.py
```
### Run Simulation
To run the navigation box with with simulated vehicle dynamics and manual control to record or replay a trajectory:
```bash
ros2 launch navigation_box simulation_motion.launch.py
```

### Advanced Usage
#### NavigationBox Launch file arguments
There are some arguments that can be modified to change the behaviour of the system.
These are listed below with their default values:
* `use_rosbag:=False` - Use data from rosbag as input
* `use_pcap_streamer:=False` - Use pcap streamer data as input
* `replay_trajectory:=False` - Replay the given trajectory file.
* `record_trajectory:=False` - Record to the given trajectory file.
* `visualize:=False` - This toggles between visualization using rviz2 or not.
* `use_autoware_planner:=False` - This toggles between autoware and aiim planners
* `use_mqtt_interface:=False` - Runs the MQTT interface node
There are also some conditional mandatory arguments:
* `rosbag_path` - The location of the rosbag path to be played
    * Only required when `use_rosbag` is set to `True`
    * Rosbag path is the path to the directory containing the db3 file
    * The Rosbag cannot be in the `/data` drive on nides machines as that will give errors.

Due to the nature of documentation, it is possible that this information will be deprecated at some point.
So, to get the latest list of possible arguments, you can also run:
```bash
ros2 launch navigation_box navigation_box.launch.py --show-args
```

#### Simulation arguments
The simulation scenario has a more simple setup, with parameters as follow:
* `replay_trajectory:=False` - Replay the given trajectory file, manual control is disabled when replay is enabled.
* `record_trajectory:=False` - Record to the given trajectory file, PurePursuit is disabled when recording is enabled.
* `visualize:=False` - This toggles between visualization using rviz2 or not.

Due to the nature of documentation, it is possible that this information will be deprecated at some point.
So, to get the latest list of possible arguments, you can also run:
```bash
ros2 launch navigation_box simulation_motion.launch.py --show-args
```

#### File paths
TODO(tijs.vandersmagt): Move these to command line arguments?
There are some file paths defined, which have to be adjusted for every machine and scenario.

* `src/navigation_box/navigation_box/navigation_box.py`
    * `trajectory_path`: Location trajectory file for recording/replaying
        * TODO(tijs.vandersmagt): This cannot be moved to a command line argument, as it currently is implemented as an ExecuteProcess. It is possible to replace this by a real node, which then takes the path as a parameter. This would require additional work, although the design of the node would be quite simple.
* `src/navigation_box/params/mmp_upstairs.yaml`
    * `lidar_intrinsics_path`: Location of the intrinsics file for the LiDAR to be used.
        * Ouster has different config files for different resolutions and types
        * This is likely to change with [MMS-2086].
    * `source_paths`: Location of the LiDAR
        * Can be either a path to a file: `/path/to/file.pcap`
        * Or a combination of ip/port: `192.168.1.200:404`
    * `pcd_dir`: Location of the point cloud pcd files
        * These can be copied to a different location, but the paths in the `grid_information.csv` file have to be changed accordingly!

#### Tuning
When working with the system, it might be nice not to have to build the system every time you make a small change to the parameters.
This is especially useful when trying to tune a system.

This can be done by changing the files in the `install` directory itself.
Note: These files will be overwritten whenever the workspace is built again.

The navigation box launch and param files can be found in:
`install/navigation_box/share/navigation_box`

## RVIZ
We use rviz2 to visualize the current state of the system.
The standard config file is contained within the repository.

If you want to make temporary changes, you can just press ctrl+s and it will save it locally.
However, this will be overwritten whenever you build the workspace again.
So, if you want to save these changes permanently, you will have to copy the contents of the temporary file in the install directory, to the main config file which is tracked by git.

The temporary config file in the install directory can be found at:
`install/navigation_box/share/navigation_box/rviz/navigation_box.rviz`

The main config file is located in:
`src/navigation_box/rviz/navigation_box.rviz`
