from wpilib.trajectory import TrajectoryConfig, TrajectoryGenerator, constraint

from autonomous import waypoints
from components import chassis
from utils import units


class Trajectories:
    MAX_VELOCITY = 16 * units.meters_per_foot
    MAX_ACCELERATION = 8 * units.meters_per_foot
    MAX_CENTRIPETAL_ACCELERATION = 8 * units.meters_per_foot
    MAX_VOLTAGE = 10

    def __init__(self):
        self.voltage_constraint = constraint.DifferentialDriveVoltageConstraint(
            chassis.Chassis.feedforward_l.model,
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

        self.start_to_trench_run_lawful = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.START_LAWFUL, waypoints.Waypoints.TRENCH_RUN_LAWFUL,),
            self.rev_config,
        )

        self.trench_run_lawful_to_shoot = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.TRENCH_RUN_LAWFUL, waypoints.Waypoints.IDEAL_SHOOT,),
            self.fwd_config,
        )

        self.shoot_to_rendezvous_two_balls = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.START_CENTER, waypoints.Waypoints.RENDEZVOUS_TWO_BALLS,),
            self.rev_config,
        )

        self.rendezvous_two_balls_to_shoot = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.RENDEZVOUS_TWO_BALLS, waypoints.Waypoints.IDEAL_SHOOT,),
            self.fwd_config,
        )

        self.start_to_trench_run_steal = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.START_STEAL, waypoints.Waypoints.TRENCH_RUN_STEAL,),
            self.rev_config,
        )

        self.trench_run_steal_to_shoot = TrajectoryGenerator.generateTrajectory(
            (waypoints.Waypoints.TRENCH_RUN_STEAL, waypoints.Waypoints.IDEAL_SHOOT,),
            self.fwd_config,
        )