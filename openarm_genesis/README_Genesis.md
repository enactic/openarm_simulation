## OpenArm Simulation with Genesis

1. [Install conda](https://conda-forge.org/download/)

2. Set up a conda virtual environment (or run ./setup_env.sh)

```sh
conda env create -f environment.yaml
conda activate openarm_genesis
```
3. Run the demo script to verify genesis is working with OpenArm. The first run will take some time to compile.
   
```sh
python3 start_genesis.py
```

## Compatibility

- The latest versions of genesis-world are often unstable. In this repo, we use genesis-world==0.2.0
- Taichi is only supported up to Python 3.11.

- Genesis cannot be used with [uv](https://docs.astral.sh/uv/) due to [an issue with OpenGL](https://github.com/Genesis-Embodied-AI/Genesis/issues/11)
