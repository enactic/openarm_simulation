# OpenArm Simulation

Examples of OpenArm simulation for inverse kinematics and physics simulation.



https://github.com/user-attachments/assets/7bbe3548-b397-4221-9124-2c797e5500b1



## Getting started

1. Clone the repository:
```sh 
git clone --recurse-submodules https://github.com/enactic/openarm_simulation.git
```
2. Run a supported simulator:

- [MuJoCo](./openarm_mujoco/README_MuJoCo.md)
- [Genesis](./openarm_genesis/README_Genesis.md)
- [Moveit2](./openarm_moveit2/README_MoveIt2.md)

## Updating Submodules (openarm_mjcf)

```sh
git submodule update --init --recursive
```

## To Do:

- Real2Sim
  - [ ] Teleop simulation robot with real arms

- Simulators:
  - [ ] Gazebo with MoveIt2
  
- Demos:
  - [ ] TD-MPC2 (MuJoCo)
  - [ ] PPO (All)
  - [ ] ManiSkill
