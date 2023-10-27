import os

import numpy as np
import sapien


class Franka(sapien.Widget):
    def __init__(self):
        pass

    def load(self, scene: sapien.Scene):
        loader = scene.create_urdf_loader()
        loader.set_material(0.3, 0.3, 0.0)

        loader.set_link_material("panda_leftfinger", 1.0, 1.0, 0.0)
        loader.set_link_material("panda_rightfinger", 1.0, 1.0, 0.0)
        loader.set_link_patch_radius("panda_leftfinger", 0.05)
        loader.set_link_patch_radius("panda_rightfinger", 0.05)
        loader.set_link_min_patch_radius("panda_leftfinger", 0.05)
        loader.set_link_min_patch_radius("panda_rightfinger", 0.05)

        path = os.path.join(os.path.dirname(__file__), "panda.urdf")
        self.robot = loader.load(path)
        for link in self.robot.links:
            link.disable_gravity = True

        self.arm_joints = self.robot.active_joints[:7]
        self.gripper_joints = self.robot.active_joints[7:]

        self.set_arm_pd([200] * 7, [100] * 7, [36] * 7)
        self.set_gripper_pd(1e4, 1e3, 0.3)

        self.robot.set_qpos(
            [
                0,
                0.19634954084936207,
                0.0,
                -2.617993877991494,
                0.0,
                2.941592653589793,
                0.7853981633974483,
                0.04,
                0.04,
            ]
        )
        self.robot.set_qvel([0] * 9)
        self.set_arm_target(
            [
                0,
                0.19634954084936207,
                0.0,
                -2.617993877991494,
                0.0,
                2.941592653589793,
                0.7853981633974483,
            ]
        )

        l2 = self.robot.active_joints[-1].child_link
        l1 = self.robot.active_joints[-2].child_link
        l0 = l1.parent

        self.robot.create_fixed_tendon(
            [l0, l1, l2], [0, 1, -1], [0, 1, -1], stiffness=1e10
        )

    def set_arm_pd(self, ps, ds, limits):
        for j, p, d, l in zip(self.arm_joints, ps, ds, limits):
            j.set_drive_property(p, d, l, "acceleration")

    def set_gripper_pd(self, p, d, limit):
        for j in self.gripper_joints:
            j.set_drive_property(p, d, limit, "acceleration")

    def set_gripper_target(self, target):
        for j in self.gripper_joints:
            j.set_drive_target(target)

    def set_arm_target(self, target):
        for t, j in zip(target, self.arm_joints):
            j.set_drive_target(t)

    def set_arm_target_velocity(self, target):
        for t, j in zip(target, self.arm_joints):
            j.set_drive_velocity_target(t)

    def unload(self, scene: sapien.Scene):
        scene.remove_articulation(self.robot)
