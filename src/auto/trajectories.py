from wpilib.trajectory import TrajectoryConfig, TrajectoryGenerator, constraint

from auto import waypoints
from components import chassis
from utils import units


class Trajectories:
    MAX_VELOCITY = 12 * units.meters_per_foot
    MAX_ACCELERATION = 8 * units.meters_per_foot
    MAX_CENTRIPETAL_ACCELERATION = 8 * units.meters_per_foot
    MAX_VOLTAGE = 10

    def __init__(self):
        self.voltage_constraint = constraint.DifferentialDriveVoltageConstraint(
            chassis.Chassis.characterization_l,
            chassis.Chassis.kinematics,
            self.MAX_VOLTAGE,
        )

        self.fwd_config = TrajectoryConfig(self.MAX_VELOCITY, self.MAX_ACCELERATION)
        self.fwd_config.addConstraint(self.voltage_constraint)
        self.fwd_config.setReversed(False)
        self.fwd_config.setKinematics(chassis.Chassis.kinematics)

        self.rev_config = TrajectoryConfig(self.MAX_VELOCITY, self.MAX_ACCELERATION)
        self.rev_config.addConstraint(self.voltage_constraint)
        self.rev_config.setReversed(True)
        self.rev_config.setKinematics(chassis.Chassis.kinematics)

        self.start_to_first_ball = TrajectoryGenerator.generateTrajectory(
            (
                waypoints.Waypoints.START_CENTER,
                waypoints.Waypoints.BALL_0,
                waypoints.Waypoints.BALL_1,
                waypoints.Waypoints.BALL_2,
                waypoints.Waypoints.BALL_3,

            ),
            self.fwd_config,
        )
