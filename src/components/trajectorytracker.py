from magicbot.state_machine import StateMachine, state
from wpilib import Timer
from wpilib.controller import RamseteController
from wpilib.trajectory import Trajectory

from autonomous import trajectories
from components import chassis


class TrajectoryTracker:

    chassis: chassis.Chassis
    BETA = 2.0
    ZETA = 0.7

    def __init__(self):
        self.trajectories: trajectories.Trajectories = trajectories.Trajectories()
        self.ramsete = RamseteController(self.BETA, self.ZETA)
        self.desired_trajectory: Trajectory = None
        self.timer = Timer()
        self.is_tracking = False

    def on_disable(self):
        self.is_tracking = False
        
    def setTrajectory(self, trajectory):
        self.timer.start()
        self.timer.reset()
        self.desired_trajectory = trajectory

    def track(self):
        self.is_tracking = True

    def stop(self):
        self.is_tracking = False

    def isFinished(self):
        if self.desired_trajectory == None:
            return True
        return self.timer.get() >= self.desired_trajectory.totalTime()

    def execute(self):
        if self.isFinished():
            self.is_tracking = False
            
        if self.is_tracking:
            cur_state = self.chassis.getPose()
            desired_state = self.desired_trajectory.sample(self.timer.get())
            desired_chassis = self.ramsete.calculate(cur_state, desired_state)
            self.chassis.setChassisVelocity(
                desired_chassis.vx, desired_chassis.vy, desired_chassis.omega
            )
