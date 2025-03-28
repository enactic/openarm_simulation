import os
import torch
import genesis as gs
import pathlib
from enum import Enum

os.environ["PYOPENGL_PLATFORM"] = "glx"

print(f"Genesis version {gs.__version__}")
KP = [85.0, 50.0, 30.0, 30.0, 27.5, 35.0, 35.0, 100.0, 100.0] * 2
KV = [18.5, 12.0, 8.0, 10.0, 3.0, 3.0, 3.0, 20.5, 20.5] * 2
# mimic joints/tendons are not supported by genesis
JOINT_NAMES = [
    "left_rev1",
    "left_rev2",
    "left_rev3",
    "left_rev4",
    "left_rev5",
    "left_rev6",
    "left_rev7",
    "left_left_pris1",
    "left_right_pris2",
    "right_rev1",
    "right_rev2",
    "right_rev3",
    "right_rev4",
    "right_rev5",
    "right_rev6",
    "right_rev7",
    "right_left_pris1",
    "right_right_pris2",
]
FORCE_LOWER = [-87, -87, -87, -87, -12, -12, -12, -12, -12] * 2
FORCE_UPPER = [87, 87, 87, 87, 12, 12, 12, 12, 12] * 2
ZERO_POS = [0.0] * len(JOINT_NAMES)
assert len(KP) == len(KV) == len(JOINT_NAMES) == len(FORCE_LOWER) == len(FORCE_UPPER), (
    f"All constants must be length {len(JOINT_NAMES)}"
)

USE_GPU = False


class RobotModelType(Enum):
    MJCF = 1
    URDF = 2


ROBOT_MODEL = RobotModelType.MJCF


def main() -> None:
    print("Setting up genesis")
    gs.init(
        backend=gs.gpu if USE_GPU else gs.cpu,
        # logging_level="debug",
    )
    scene: gs.Scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            substeps=10,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.5, 0.0, 2.5),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
        ),
        show_viewer=True,
    )

    scene.add_entity(gs.morphs.Plane())
    assets_path = pathlib.Path(__file__).resolve().parent.parent / "assets/genesis"
    mjcf_str = str(assets_path / "openarm_bimanual.mjcf.xml")

    POS_ORIGIN = (0, 0, 0)  # m
    BASE_EULER = (0, 0, 0)  # degrees
    HORIZONTAL_EULER = (0, 90, 90)  # degrees

    openarm = scene.add_entity(
        gs.morphs.MJCF(
            file=mjcf_str,
            pos=POS_ORIGIN,
            euler=BASE_EULER,
            collision=False,  # box collision not supported for MJCF
            convexify=False,
        ),
        vis_mode="visual",
    )

    scene.build()

    dofs_idx = [openarm.get_joint(name).dof_idx_local for name in JOINT_NAMES]

    openarm.set_dofs_kp(kp=KP, dofs_idx_local=dofs_idx)
    openarm.set_dofs_kv(kv=KV, dofs_idx_local=dofs_idx)
    openarm.set_dofs_force_range(lower=FORCE_LOWER, upper=FORCE_UPPER, dofs_idx_local=dofs_idx)
    openarm.set_dofs_position(ZERO_POS, dofs_idx)
    openarm.control_dofs_position(ZERO_POS, dofs_idx)

    scene.step()

    if gs.platform == "Linux":
        run_sim(scene=scene, openarm=openarm, dofs_idx=dofs_idx)
    else:
        gs.tools.run_in_another_thread(fn=run_sim, args=(scene, openarm, dofs_idx))
        scene.viewer.start()


def run_sim(scene: gs.Scene, openarm, dofs_idx):
    COMMAND_STEPS = 100
    increment = COMMAND_STEPS * 0.01  # 1 %
    curr_step = 0
    LOWER_LIMIT, UPPER_LIMIT = openarm.get_dofs_limit()
    EXAMPLE_INDEX = 0
    GRIPPER_INDEX_LEFT = 7
    GRIPPER_INDEX_RIGHT = 8

    command_pos = torch.zeros_like(openarm.get_dofs_position())
    while True:
        positions = openarm.get_dofs_position()
        velocities = openarm.get_dofs_velocity()
        forces = openarm.get_dofs_force()
        control_forces = openarm.get_dofs_control_force()

        print(f"Positions: {positions}")
        # print(f"Velocities: {velocities}")
        # print(f"Forces: {forces}")
        # print(f"Control Forces: {control_forces}")

        curr_step += increment

        command_pos[EXAMPLE_INDEX] = (curr_step / COMMAND_STEPS) * (
            LOWER_LIMIT[EXAMPLE_INDEX] if curr_step < 0 else UPPER_LIMIT[EXAMPLE_INDEX]
        )
        command_pos[GRIPPER_INDEX_LEFT] = (
            abs(curr_step / COMMAND_STEPS) * LOWER_LIMIT[GRIPPER_INDEX_LEFT]
        )  # negative range of motion only
        # tendons and mimic joints are unsupported
        command_pos[GRIPPER_INDEX_RIGHT] = (
            abs(curr_step / COMMAND_STEPS) * LOWER_LIMIT[GRIPPER_INDEX_RIGHT]
        )  # negative range of motion only

        if not torch.all((LOWER_LIMIT <= command_pos) & (command_pos <= UPPER_LIMIT)):
            increment *= -1.0
            print("Control positions out of bounds, ignoring")
        else:
            print(f"Sent command {command_pos}")
            openarm.control_dofs_position(command_pos)

        scene.step()


if __name__ == "__main__":
    main()
