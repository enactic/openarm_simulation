import os
import numpy as np
import genesis as gs
import pathlib

os.environ["PYOPENGL_PLATFORM"] = "glx"

KP = [85.0, 50.0, 30.0, 30.0, 27.5, 35.0, 35.0, 100.0]
KV = [18.5, 12.0, 8.0, 10.0, 3.0, 3.0, 3.0, 20.5]
JOINT_NAMES = ["rev1", "rev2", "rev3", "rev4", "rev5", "rev6", "rev7", "slider_left"]
FORCE_LOWER = [-87, -87, -87, -87, -12, -12, -12, -12]
FORCE_UPPER = [87, 87, 87, 87, 12, 12, 12, 12]
ZERO_POS = [0.0] * len(JOINT_NAMES)
assert len(KP) == len(KV) == len(JOINT_NAMES) == len(FORCE_LOWER) == len(FORCE_UPPER), (
    f"All constants must be length {len(JOINT_NAMES)}"
)

USE_GPU = False


def main() -> None:
    print("Setting up genesis")
    gs.init(backend=gs.gpu if USE_GPU else gs.cpu, logging_level="debug")
    scene: gs.Scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            substeps=10,
            gravity=(0, 0, -9.81),
        ),
    )

    scene.add_entity(gs.morphs.Plane())
    assets_path = str(pathlib.Path(__file__).resolve().parent.parent / "assets/openarm_grip.urdf")

    POS_ORIGIN = (0, 0, 0)  # m
    BASE_EULER = (0, 0, 0)  # degrees
    HORIZONTAL_EULER = (0, 90, 90)  # degrees

    openarm: gs.engine.entities.rigid_entity.rigid_entity.RigidEntity = scene.add_entity(
        gs.morphs.URDF(
            file=assets_path,
            pos=POS_ORIGIN,
            euler=BASE_EULER,
            fixed=True,
        ),
    )
    scene.build()

    dofs_idx = [openarm.get_joint(name).dof_idx_local for name in JOINT_NAMES]

    openarm.set_dofs_kp(kp=KP, dofs_idx_local=dofs_idx)
    openarm.set_dofs_kv(kv=KV, dofs_idx_local=dofs_idx)
    openarm.set_dofs_force_range(lower=FORCE_LOWER, upper=FORCE_UPPER, dofs_idx_local=dofs_idx)
    openarm.set_dofs_position(ZERO_POS, dofs_idx)
    openarm.control_dofs_position(ZERO_POS, dofs_idx)

    scene.step()
    gs.tools.run_in_another_thread(fn=run_sim, args=(scene, openarm, dofs_idx))
    scene.viewer.start()


def run_sim(scene: gs.Scene, openarm, dofs_idx):
    while True:
        positions = openarm.get_dofs_position()
        velocities = openarm.get_dofs_velocity()
        forces = openarm.get_dofs_force()

        control_forces = openarm.get_dofs_control_force()

        # print(f"Positions: {positions}")
        # print(f"Velocities: {velocities}")
        # print(f"Forces: {forces}")
        # print(f"Control Forces: {control_forces}")

        scene.step()
    scene.viewer.stop()


if __name__ == "__main__":
    main()
