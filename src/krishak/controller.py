"""Discrete task scheduler for one stationary camera observation."""
from enum import Enum
import numpy as np


class State(str, Enum):
    IDLE = "idle"
    STOP = "stop_rover"
    SELECT = "select_target"
    APPROACH = "approach"
    ACT = "grasp"
    LIFT = "lift"
    DEPOSIT = "move_to_basket"
    RELEASE = "release"
    RETRACT = "retract"
    DONE = "done"
    FAULT = "fault"


class Controller:
    def __init__(self, robot):
        self.robot = robot
        self.state = State.IDLE
        self.pending = []
        self.trace = []
        self.target = None
        self.plan = []
        self.error = None

    def transition(self, state, detail=""):
        self.trace.append({"step": len(self.trace), "from": self.state.value,
                           "to": state.value, "detail": detail})
        self.state = state

    def begin(self, targets):
        if self.state != State.IDLE:
            raise RuntimeError("Create a new controller for each observation")
        accepted = []
        for point in targets:
            p = np.asarray(point, float)
            if p.shape != (3,) or not np.isfinite(p).all():
                raise ValueError("Target must contain finite xyz coordinates")
            # Example working patch in front of the arm. Never clamp a target.
            if not (0.15 <= p[0] <= 0.38 and abs(p[1]) <= 0.18 and abs(p[2]) < 1e-6):
                self.trace.append({"step": len(self.trace), "from": "idle", "to": "idle",
                                   "detail": "rejected target outside working patch"})
                continue
            if not any(np.linalg.norm(p - other) < 0.025 for other in accepted):
                accepted.append(p)
        self.pending = sorted(accepted, key=lambda p: np.linalg.norm(p))
        self.transition(State.STOP)

    def tick(self, emergency_stop=False):
        if emergency_stop:
            self.robot.stop()
            self.error = "emergency stop requested"
            self.transition(State.FAULT, self.error)
            return
        if self.state in (State.IDLE, State.DONE, State.FAULT):
            return
        try:
            if self.state == State.STOP:
                self.robot.stop()
                self.transition(State.SELECT)
            elif self.state == State.SELECT:
                if not self.pending:
                    self.transition(State.DONE)
                    return
                self.target = self.pending.pop(0)
                approach = self.target + [0, 0, 0.08]
                # Solve all task poses before issuing a movement command.
                above = self.robot.arm.inverse_position(approach, self.robot.joints)
                contact = self.robot.arm.inverse_position(self.target, above)
                basket = self.robot.arm.inverse_position([0.20, -0.16, 0.15], above)
                self.plan = [above, contact, basket]
                self.transition(State.APPROACH)
            elif self.state == State.APPROACH:
                self.robot.set_gripper(False)
                self.robot.move_arm(self.plan[0])
                self.transition(State.ACT)
            elif self.state == State.ACT:
                self.robot.move_arm(self.plan[1])
                self.robot.set_gripper(True)
                self.transition(State.LIFT)
            elif self.state == State.LIFT:
                self.robot.move_arm(self.plan[0])
                self.transition(State.DEPOSIT)
            elif self.state == State.DEPOSIT:
                self.robot.move_arm(self.plan[2])
                self.transition(State.RELEASE)
            elif self.state == State.RELEASE:
                self.robot.set_gripper(False)
                self.robot.record_uproot(self.target)
                self.transition(State.RETRACT)
            elif self.state == State.RETRACT:
                self.robot.move_arm(self.robot.arm.home)
                self.transition(State.SELECT)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.robot.stop()
            self.error = str(exc)
            self.transition(State.FAULT, self.error)
