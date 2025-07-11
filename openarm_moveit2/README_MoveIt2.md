## OpenArm Inverse Kinematics with MoveIt2 (ROS2)


1. [Install ROS 2](https://docs.ros.org/en/humble/Installation.html)
> We recommend using ROS2 Humble with Ubuntu Linux 22.04

Make sure to also install [ros-dev-tools](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#install-development-tools-optional)


2. Install MoveIt2

a. ROS2 Humble: Install from debian packages

```sh
sudo apt update
sudo apt install -y ros-humble-moveit
```

b. [ROS2 Jazzy: Install and build MoveIt2 from source](https://moveit.ai/install-moveit2/source/). The prebuilt debian packages are sometimes outdated, which may cause issues. 

It may be necessary to run a modified version of the build command if your system has less than 64 GB of RAM:
```sh
cd $COLCON_WS
rm -rf build log install
colcon build --mixin release --parallel-workers 2
```

After building, rename the workspace:
```sh
mv ~/ws_moveit2 ~/ws_moveit
```

3. [Source the ROS2 install and create a colcon workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#build-the-workspace)

```sh
source /opt/ros/humble/setup.bash
mkdir -p ~/ws_openarm/src
```

4. Clone the [openarm_ros2 repository](https://github.com/enactic/openarm_ros2) into the workspace:
   
```sh
cd ~/ws_openarm/src
git clone https://github.com/enactic/openarm_ros2.git
```

5. [Install dependencies with rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)
```sh
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
rosdep install --from-paths ~/ws_openarm/src -y -r --ignore-src
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

8. Drag the interactive marker to set a goal position, and press `Plan & Execute` to view the movement.
