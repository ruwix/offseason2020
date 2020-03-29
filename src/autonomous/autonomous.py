from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from wpilib.geometry import Pose2d, Rotation2d
from components import chassis
from components import trajectorytracker
from autonomous import trajectories, waypoints
from physics import PhysicsEngine
from enum import Enum


class AutoMode(Enum):
    LAWFUL = 0
    STEAL = 1
    RENDEZVOUS = 2


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Ramsete"
    DEFAULT = True

    trajectorytracker: trajectorytracker.TrajectoryTracker
    chassis: chassis.Chassis

    def __init__(self):
        super().__init__()
        self.trajectories = trajectories.Trajectories()

    def setup(self):
        PhysicsEngine.setSimulationPose(waypoints.Waypoints.START_LAWFUL)

    def on_enable(self):
        super().on_enable()

    def on_disable(self):
        self.done()

    def genericTrajectoryState(self, initial_call, trajectory, next_state):
        if initial_call:
            self.trajectorytracker.setTrajectory(trajectory)
        self.trajectorytracker.track()
        if self.trajectorytracker.isFinished():
            self.next_state(next_state)

    @state(first=True)
    def pickMode(self, initial_call):
        mode = AutoMode.STEAL
        if mode == AutoMode.LAWFUL:
            PhysicsEngine.setSimulationPose(waypoints.Waypoints.START_LAWFUL)
            self.next_state("collectTrenchRunBalls")
        elif mode == AutoMode.STEAL:
            PhysicsEngine.setSimulationPose(waypoints.Waypoints.START_STEAL)
            self.next_state("stealTrenchRunBalls")
        elif mode == AutoMode.RENDEZVOUS:
            PhysicsEngine.setSimulationPose(waypoints.Waypoints.START_CENTER)
            self.next_state("collectTwoRendezvousBalls")
        else:
            self.next_state("success")

    @state()
    def collectTrenchRunBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call,
            self.trajectories.start_to_trench_run_lawful,
            "shootTrenchRunBalls",
        )

    @state()
    def shootTrenchRunBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call,
            self.trajectories.trench_run_lawful_to_shoot,
            "success",
        )

    @state()
    def collectTwoRendezvousBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call,
            self.trajectories.shoot_to_rendezvous_two_balls,
            "shootTwoRendezvousBalls",
        )

    @state()
    def shootTwoRendezvousBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call, self.trajectories.rendezvous_two_balls_to_shoot, "success"
        )

    @state()
    def stealTrenchRunBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call,
            self.trajectories.start_to_trench_run_steal,
            "shootStolenTrechRunBalls",
        )

    @state()
    def shootStolenTrechRunBalls(self, initial_call):
        self.genericTrajectoryState(
            initial_call, self.trajectories.trench_run_steal_to_shoot, "success"
        )

    @state()
    def success(self):
        self.done()

    def done(self):
        super().done()
        self.chassis.stop()
