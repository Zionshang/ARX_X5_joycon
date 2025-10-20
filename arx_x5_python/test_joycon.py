from bimanual import SingleArm
from typing import Dict, Any
import numpy as np
import time
import os
import sys

JOY_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if JOY_ROOT not in sys.path:
    sys.path.append(JOY_ROOT)

from joycon.joyconrobotics import JoyconRobotics
from joycon.visualizer import PoseVisualizer

arm_config: Dict[str, Any] = {
    "can_port": "can0",
    "type": 0,
}


def run_joycon_control():
    single_arm = SingleArm(arm_config)

    joy = JoyconRobotics(
        device="right",
        translation_frame="local",
        direction_reverse=[1, 1, 1],
        euler_reverse=[-1, -1, 1],
        offset_position_m=[0, 0, 0],
        limit_dof=True,
        glimit=[[0.0, -0.5, -0.5, -1.3, -1.3, -1.3], [0.5, 0.5, 0.5, 1.3, 1.3, 1.3]],
        gripper_limit=[0.0, 0.8],
    )

    viz = PoseVisualizer(
        axis_len=0.3,
        world_axis_len=0.2,
        window_title="Joycon Pose",
        orientation_format="euler",
    )

    while True:
        # pose: [x,y,z, roll, pitch, yaw]
        pose, gripper, control_button = joy.get_control()

        if control_button == 10:  # 同时按住 R+ZR 退出
            break

        # 发送末端位姿
        single_arm.set_ee_pose_xyzrpy(xyzrpy=np.asarray(pose, dtype=float))

        # 发送夹爪
        single_arm.set_catch_pos(pos=float(gripper))

        viz.update(pose)
        time.sleep(0.01)

    joy.disconnect()
    viz.close()


if __name__ == "__main__":
    run_joycon_control()
