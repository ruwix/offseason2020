from magicbot.state_machine import StateMachine, state
from wpilib import Timer
from wpilib.controller import RamseteController
from wpilib.trajectory import Trajectory

from auto import trajectories
from components import chassis


class TrajectoryTracker(StateMachine):

    chassis: chassis.Chassis
    BETA = 2.0
    ZETA = 0.7

    def __init__(self):
        self.trajectories = trajectories.Trajectories()
        self.ramsete = RamseteController(self.BETA, self.ZETA)
        self.desired_trajectory: Trajectory = None
        self.timer = Timer()

    def startToBall0(self):
        self.track(self.trajectories.start_to_first_ball)

    def track(self, trajectory):
        self.desired_trajectory = trajectory
        self.engage("trackTrajectory")

    @state(first=True)
    def trackTrajectory(self, initial_call):
        if initial_call:
            self.timer.start()
            self.timer.reset()
        cur_state = self.chassis.getPose()
        desired_state = self.desired_trajectory.sample(self.timer.get())
        desired_chassis = self.ramsete.calculate(cur_state, desired_state)
        self.chassis.setChassisVelocity(
            desired_chassis.vx, desired_chassis.vy, desired_chassis.omega
        )
        if self.desired_trajectory.totalTime() - self.timer.get() <= 0:
            self.next_state("success")

    @state()
    def success(self):
        self.chassis.stop()

    def done(self):
        super().done()
        self.chassis.stop()
