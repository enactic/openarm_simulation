## OpenArm Simulation with Genesis


https://github.com/user-attachments/assets/4eaf1445-3274-4b60-ab9d-4c2eec123964


1. [Install conda](https://conda-forge.org/download/)

2. Change to the openarm_genesis directory:
```sh
cd openarm_genesis
```

3. Set up a conda virtual environment
```sh
conda env create -f environment.yaml
conda activate openarm_genesis
```

4. If using Mac OS, [upgrade PyOpenGL](https://github.com/Genesis-Embodied-AI/Genesis/issues/24)
```sh
pip install -U PyOpenGL
```

5. Run the demo script to verify genesis is working with OpenArm. The first run will take some time to compile.
   
```sh
python3 start_genesis.py
```



## Compatibility

- Genesis [does not support mimic joints (URDF) or tendons (MJCF)](https://github.com/Genesis-Embodied-AI/Genesis/issues/129), meaning each actuator must be controlled separately.
- The latest versions of genesis-world are often unstable. In this repo, we use genesis-world==0.2.1
- Genesis cannot be used with [uv](https://docs.astral.sh/uv/) due to [an issue with OpenGL].(https://github.com/Genesis-Embodied-AI/Genesis/issues/11)
