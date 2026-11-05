"""Illustrative geometry only: metres, radians, arm-base coordinates."""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class GroundMapper:
    homography: np.ndarray

    def __post_init__(self):
        self.homography = np.asarray(self.homography, float)
        if (self.homography.shape != (3, 3) or not np.isfinite(self.homography).all()
                or abs(np.linalg.det(self.homography)) < 1e-12):
            raise ValueError("Homography must be finite and invertible")

    def project(self, pixel):
        point = self.homography @ np.array([*pixel, 1.0])
        if not np.isfinite(point).all() or abs(point[2]) < 1e-9:
            raise ValueError("Pixel does not project to a finite ground point")
        return np.array([point[0] / point[2], point[1] / point[2], 0.0])


class Arm:
    # Standard DH: a, alpha, d. Example links, not measured Krishak geometry.
    dh = ((0, np.pi / 2, 0.12), (0.22, 0, 0), (0.18, 0, 0),
          (0, np.pi / 2, 0), (0, -np.pi / 2, 0.06), (0, 0, 0.06))
    home = np.array([0.0, 0.5, -1.0, 0.0, 0.5, 0.0])

    def forward(self, joints):
        if np.shape(joints) != (6,) or not np.isfinite(joints).all():
            raise ValueError("Expected six finite joint angles")
        transform = np.eye(4)
        for theta, (a, alpha, d) in zip(joints, self.dh):
            c, s, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
            transform = transform @ np.array([[c, -s * ca, s * sa, a * c],
                                              [s, c * ca, -c * sa, a * s],
                                              [0, sa, ca, d], [0, 0, 0, 1]])
        return transform

    def inverse_position(self, target, seed=None):
        """Damped least-squares position IK. Tool orientation is unconstrained."""
        target = np.asarray(target, float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Expected finite xyz target")
        joints = self.home.copy() if seed is None else np.array(seed, float)
        for _ in range(250):
            position = self.forward(joints)[:3, 3]
            error = target - position
            if np.linalg.norm(error) < 0.002:
                return joints
            jacobian = np.zeros((3, 6))
            for i in range(6):
                delta = np.zeros(6)
                delta[i] = 1e-5
                jacobian[:, i] = (self.forward(joints + delta)[:3, 3] - position) / 1e-5
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.001 * np.eye(3), error)
            joints = np.clip(joints + np.clip(step, -0.15, 0.15), -np.pi, np.pi)
        raise ValueError("IK did not converge within the example joint limits")


class SeedArm:
    """Paper-inspired base/shoulder/elbow model; gripper is the fourth DOF.

    Link dimensions are examples. Angles use a signed elbow convention, rather
    than the interior triangle angles shown in the manuscript figures.
    """
    home = np.array([0.0, 0.5, -1.0])

    def __init__(self, base_height=0.12, upper=0.22, forearm=0.18):
        if not np.isfinite([base_height, upper, forearm]).all() or min(base_height, upper, forearm) <= 0:
            raise ValueError("Link dimensions must be finite and positive")
        self.base_height, self.upper, self.forearm = base_height, upper, forearm

    def forward(self, joints):
        if np.shape(joints) != (3,) or not np.isfinite(joints).all():
            raise ValueError("Expected three finite positioning joint angles")
        yaw, shoulder, elbow = joints
        pitch = shoulder + elbow
        radius = self.upper * np.cos(shoulder) + self.forearm * np.cos(pitch)
        c, s, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
        transform = np.array([[c*cp, -s, -c*sp, radius*c],
                              [s*cp, c, -s*sp, radius*s],
                              [sp, 0, cp, self.base_height + self.upper*np.sin(shoulder)
                               + self.forearm*np.sin(pitch)], [0, 0, 0, 1]])
        return transform

    def inverse_position(self, target, seed=None):
        target = np.asarray(target, float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Expected finite xyz target")
        x, y, z = target
        radius, height = np.hypot(x, y), z - self.base_height
        cosine = (radius**2 + height**2 - self.upper**2 - self.forearm**2) / (2*self.upper*self.forearm)
        if abs(cosine) > 1 + 1e-10:
            raise ValueError("Target is outside the reduced arm workspace")
        candidates = []
        for sign in (-1, 1):
            elbow = sign * np.arccos(np.clip(cosine, -1, 1))
            shoulder = np.arctan2(height, radius) - np.arctan2(
                self.forearm*np.sin(elbow), self.upper + self.forearm*np.cos(elbow))
            q = np.array([np.arctan2(y, x), shoulder, elbow])
            if np.all(np.abs(q) <= np.pi):
                candidates.append(q)
        if not candidates:
            raise ValueError("No solution within example joint limits")
        reference = self.home if seed is None else np.asarray(seed, float)
        if reference.shape != (3,) or not np.isfinite(reference).all():
            raise ValueError("Invalid reduced arm seed")
        return min(candidates, key=lambda q: np.linalg.norm(q - reference))


@dataclass
class SimRobot:
    arm: Arm = field(default_factory=Arm)
    joints: np.ndarray = field(default_factory=lambda: Arm.home.copy())
    wheel_speeds: np.ndarray = field(default_factory=lambda: np.zeros(6))
    stopped: bool = True
    actions: list = field(default_factory=list)
    gripper_closed: bool = False

    def __post_init__(self):
        self.joints = self.arm.home.copy()

    def drive(self, linear, angular):
        if not np.isfinite([linear, angular]).all():
            raise ValueError("Invalid velocity")
        # Example skid-steer arrangement: left three, then right three wheels.
        left, right = (linear - angular * 0.4 / 2) / 0.08, (linear + angular * 0.4 / 2) / 0.08
        self.wheel_speeds = np.array([left] * 3 + [right] * 3)
        self.stopped = bool(np.allclose(self.wheel_speeds, 0))

    def stop(self):
        self.drive(0, 0)

    def move_arm(self, joints):
        if not self.stopped:
            raise RuntimeError("Arm movement requires a stopped rover")
        if np.shape(joints) != self.arm.home.shape or not np.isfinite(joints).all() or np.any(np.abs(joints) > np.pi):
            raise ValueError("Invalid arm command")
        self.joints = np.array(joints, copy=True)

    def set_gripper(self, closed):
        if not self.stopped:
            raise RuntimeError("Gripper movement requires a stopped rover")
        self.gripper_closed = bool(closed)

    def record_uproot(self, target):
        if not self.stopped or self.gripper_closed:
            raise RuntimeError("Completion requires a stopped rover and released gripper")
        self.actions.append({"kind": "simulated_uproot_and_deposit", "xyz_m": list(target)})
