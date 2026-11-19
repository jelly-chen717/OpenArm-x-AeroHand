"""Interactive V1 model viewer; no motor transport and no trajectory playback."""
import mujoco.viewer
from openarm_app.kinematics import OpenArmIK


def main():
    arm = OpenArmIK()
    arm.setup.model.vis.quality.shadowsize = 2048
    mujoco.viewer.launch(arm.setup.model, arm.setup.data)


if __name__ == "__main__":
    main()
