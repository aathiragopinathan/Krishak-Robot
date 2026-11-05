import tempfile
import unittest
from pathlib import Path

import numpy as np

from krishak.cli import synthetic_field
from krishak.controller import Controller, State
from krishak.robotics import Arm, GroundMapper, SeedArm, SimRobot
from krishak.vision import PlantClassifier, detect


class PerceptionTests(unittest.TestCase):
    def test_fixture_and_empty_image(self):
        plants, _ = detect(synthetic_field())
        self.assertEqual(len(plants), 11)
        self.assertEqual(sum(p.weed for p in plants), 3)
        self.assertEqual(detect(np.zeros((100, 100, 3), np.uint8))[0], [])

    def test_classifier_roundtrip(self):
        x = np.array([[0.01, 1, 0.7, d] for d in (0.01, 0.02, 0.2, 0.3)])
        model = PlantClassifier.fit(x, [0, 0, 1, 1])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.json"
            model.save(path)
            loaded = PlantClassifier.load(path)
            np.testing.assert_array_equal(loaded.predict(x) >= 0.5, [False, False, True, True])

    def test_crop_exclusion_overrides_model(self):
        model = PlantClassifier(np.zeros(4), np.ones(4), [0, 0, 0, 0, 30])
        plants, _ = detect(synthetic_field(), model=model)
        self.assertEqual(sum(p.weed for p in plants), 3)


class RoboticsTests(unittest.TestCase):
    def test_projection(self):
        mapper = GroundMapper([[0.001, 0, 0.1], [0, 0.001, -0.2], [0, 0, 1]])
        np.testing.assert_allclose(mapper.project((100, 200)), [0.2, 0, 0])
        with self.assertRaises(ValueError):
            GroundMapper(np.zeros((3, 3)))

    def test_kinematics(self):
        arm = Arm()
        target = np.array([0.28, 0.03, 0])
        joints = arm.inverse_position(target)
        self.assertLess(np.linalg.norm(arm.forward(joints)[:3, 3] - target), 0.002)
        with self.assertRaises(ValueError):
            arm.inverse_position([4, 0, 0])

    def test_rover_interlock(self):
        robot = SimRobot()
        robot.drive(0.1, 0)
        np.testing.assert_allclose(robot.wheel_speeds, np.full(6, 1.25))
        with self.assertRaises(RuntimeError):
            robot.move_arm(Arm.home)
        with self.assertRaises(RuntimeError):
            robot.set_gripper(True)

    def test_reduced_arm_geometry_and_branches(self):
        arm = SeedArm()
        for target in ([0.28, 0.03, 0], [-0.2, 0.1, 0.15], [0.2, -0.16, 0.15]):
            joints = arm.inverse_position(target)
            np.testing.assert_allclose(arm.forward(joints)[:3, 3], target, atol=1e-9)
        for elbow in (-1.0, 1.0):
            seed = np.array([0.2, -0.3, elbow])
            target = arm.forward(seed)[:3, 3]
            np.testing.assert_allclose(arm.inverse_position(target, seed), seed, atol=1e-9)
        with self.assertRaises(ValueError):
            arm.inverse_position([2, 0, 0])

    def test_reduced_arm_uproot_sequence(self):
        robot = SimRobot(arm=SeedArm())
        controller = Controller(robot)
        controller.begin([[0.28, 0.03, 0]])
        observed = []
        for _ in range(20):
            controller.tick()
            observed.append((controller.state, robot.gripper_closed))
        self.assertEqual(controller.state, State.DONE)
        self.assertIn((State.LIFT, True), observed)
        self.assertIn((State.RELEASE, True), observed)
        self.assertIn((State.RETRACT, False), observed)
        self.assertEqual(robot.actions[0]['kind'], 'simulated_uproot_and_deposit')
        np.testing.assert_allclose(robot.joints, robot.arm.home)

    def test_emergency_stop_while_holding(self):
        robot = SimRobot(arm=SeedArm())
        controller = Controller(robot)
        controller.begin([[0.28, 0.03, 0]])
        for _ in range(4):
            controller.tick()
        self.assertTrue(robot.gripper_closed)
        joints = robot.joints.copy()
        controller.tick(emergency_stop=True)
        controller.tick()
        self.assertEqual(controller.state, State.FAULT)
        self.assertTrue(robot.gripper_closed)
        self.assertEqual(robot.actions, [])
        np.testing.assert_allclose(robot.joints, joints)

    def test_complete_loop_dedup_and_workspace(self):
        robot = SimRobot()
        robot.drive(0.1, 0)
        controller = Controller(robot)
        controller.begin([[0.28, 0.03, 0], [0.281, 0.03, 0], [5, 0, 0]])
        for _ in range(30):
            controller.tick()
        self.assertEqual(controller.state, State.DONE)
        self.assertEqual(len(robot.actions), 1)
        self.assertTrue(robot.stopped)
        np.testing.assert_allclose(robot.joints, Arm.home)

    def test_emergency_stop_is_latched(self):
        robot = SimRobot()
        controller = Controller(robot)
        controller.begin([[0.28, 0, 0]])
        controller.tick(emergency_stop=True)
        for _ in range(10):
            controller.tick()
        self.assertEqual(controller.state, State.FAULT)
        self.assertEqual(robot.actions, [])
        self.assertTrue(robot.stopped)

    def test_failed_ik_does_not_actuate(self):
        robot = SimRobot()
        def fail(*args):
            raise ValueError("test IK failure")
        robot.arm.inverse_position = fail
        controller = Controller(robot)
        controller.begin([[0.28, 0, 0]])
        controller.tick()
        controller.tick()
        self.assertEqual(controller.state, State.FAULT)
        self.assertEqual(robot.actions, [])

    def test_no_targets(self):
        controller = Controller(SimRobot())
        controller.begin([])
        controller.tick()
        controller.tick()
        self.assertEqual(controller.state, State.DONE)


if __name__ == "__main__":
    unittest.main()
