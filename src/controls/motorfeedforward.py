from wpilib.controller import SimpleMotorFeedforwardMeters
from controls import motorstate


class MotorFeedforward:
    def __init__(self, ks, kv, ka):
        self.model = SimpleMotorFeedforwardMeters(ks, kv, ka)
        self.desired_velocity = 0
        self.prev_desired_velocity = 0
        self.desired_acceleration = 0

    def calculate(self, desired_velocity, dt):
        self.desired_velocity = desired_velocity
        self.desired_acceleration = (
            self.desired_velocity - self.prev_desired_velocity
        ) / dt
        self.prev_desired_velocity = self.desired_velocity
        return self.model.calculate(self.desired_velocity, self.desired_acceleration)
