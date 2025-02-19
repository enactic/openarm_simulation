## OpenArm Inverse Kinematics with MoveIt2 (ROS2)


1. [Install ROS 2](https://docs.ros.org/en/jazzy/Installation.html)
> We recommend using ROS2 Jazzy with Ubuntu Linux 24.04

2. [Install MoveIt2 from source](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html). The debian packages are often outdated, which may cause issues.

3. [Source the ROS2 install and create a colcon workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#build-the-workspace)

```sh
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ws_openarm/src
```

4. Clone the [openarm_ros2 repository](https://github.com/reazon-research/openarm_ros2) into the workspace:
   
```sh
cd ~/ws_openarm/src
git clone git@github.com:reazon-research/openarm_ros2.git
```

5. [Install dependencies with rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)
```sh
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
rosdep install --from-paths src -y --ignore-src
```

6. Build the packages
```sh
sudo apt install python3-colcon-common-extensions
cd ~/ws_openarm
colcon build
```

7. **Open a new terminal**, then source the `setup.bash` file in each workspace before launching the demo
```sh
source ~/ws_moveit/install/setup.bash
source ~/ws_openarm/install/setup.bash
ros2 launch openarm_moveit_config demo.launch.py
```

7. Drag the interactive marker to set a goal position, and press `Plan & Execute` to view the movement.
