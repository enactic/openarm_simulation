## OpenArm Simulation with MuJoCo

https://github.com/user-attachments/assets/5452adfd-1496-4264-bd3d-56e6716dfa6f

1. [Download MuJoCo](https://github.com/google-deepmind/mujoco/releases) and launch a simulation window.

For example, on x86 Linux:
```sh
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
cd mujoco-3.2.7/bin
./mujoco-3.2.7/bin/simulate
```

2. [Download the OpenArm MJCF and assets](https://github.com/enactic/openarm_mjcf/tree/master) and drag the `*.xml` file into the simulation window.
```sh
git clone git@github.com:enactic/openarm_mjcf.git
```
