from magicbot.state_machine import AutonomousStateMachine, state

from components import chassis
from statemachines import trajectorytracker


class Autonomous(AutonomousStateMachine):

    MODE_NAME = "Ramsete"
    DEFAULT = True

    trajectorytracker: trajectorytracker.TrajectoryTracker
    chassis: chassis.Chassis

    def __init__(self):
        super().__init__()

    def setup(self):
        pass

    def on_enable(self):
        super().on_enable()

    @state(first=True)
    def trackTrajectory(self):
        self.trajectorytracker.startToBall0()

    def done(self):
        super().done()
        self.chassis.stop()
