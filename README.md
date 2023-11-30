# Working with Gazebo

ENPM808X - Abhishekh Reddy Munnangi, 119399002

## Get started

### Environment and dependencies

#### Operating system, ROS installation and additional software

- Ubuntu Jammy (22.04)
- ROS2 Humble Hawksbill (Even base installation is sufficient)
- Git

#### Package dependencies

- `rclcpp` - ROS2 C++ Client Library
- `turtlebot3*` - Collection of packages for simulating Turtlebot 3 in Gazebo
- `std_msgs` - Standard Messages Library
- `geometry_msgs` - Geometry Messages Library
- `sensor_msgs` - Sensor Messages Library
- `ros2launch` - ROS2 Launch Library for Launch file support

#### Additional notes and considerations

- The following section assumes that you have an existing ROS2 workspace. If not,
see [how to create one](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#create-a-new-directory).

### Cloning the repository

The root directory of this repository are contents of a ROS2 package. Hence,
this needs to be cloned to a folder in the `src/` directory of a ROS2 project
workspace.

```console
<Your ROS2 Workspace>/
├── build/
├── install/
├── log/
└── src/
    └── gazebo_exercise/ <-- Repository clones to this folder
        └── <Repository contents>
```

<p align="center">Workspace directory tree with this repository contents in it</p>

Run this command in the `src/` directory of your ROS2 workspace

```bash
git clone https://github.com/armgits/808x-gazebo-exercise.git gazebo_exercise
```

>**Before the next step:** For the first time, ensure that the package
> dependencies are installed. Run these commands from the **root workspace directory**.

```bash
rosdep init && rosdep update
```

```bash
rosdep install --from-paths src -y --ignore-src
```

Build the package in the **root directory** of your ROS2 workspace.

```bash
colcon build --packages-select gazebo_exercise
```

Source the freshly built package

```bash
source install/setup.bash
```

### Launching the Gazebo Simulation

THe launch file in this package relies on the existing launch scripts from the
Turtlebot 3 package, so this step is fairly straightforward.

```bash
ros2 launch gazebo_exercise exercise_launch.py
```

For more information on the argumnets, run

```bash
ros2 launch gazebo_exercise exercise_launch.py --show-args
```

>**Note:** There could be a lot more arguments (including a previously existing rosbag record
>argument) in addition to the rosbag record argument from the Turtlebot 3 lanch
>file.

### Recording Bag File

Appending the `record_bag:=True` parameter to the launch command enables recording
all the message activity to a rosbag in the current working directory (Directory
in the terminal where the launch command is executed). This option is disabled
by default.

```bash
ros2 launch gazebo_exercise exercise_launch.py record_bag:=True
```

Recording stops when the Gazebo simulation is terminated.

#### Inspecting the Bag File

In the directory the previously discussed launch command is executed and the bag
file is saved to run:

```bash
ros2 bag info output_bag/
```

### Playing the Bag ile

In a similar fashion as the previous step:

```bash
ros2 bag play output_bag/
```

>Note: Gazebo should not be running in this step.
