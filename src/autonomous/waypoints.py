from wpilib.geometry import Pose2d, Rotation2d

from utils import units


class Waypoints:
    MIN_FIELD_X = 0
    MAX_FIELD_X = 54 * units.meters_per_foot
    MIN_FIELD_Y = 0
    MAX_FIELD_Y = 27 * units.meters_per_foot

    INITATION_LINE_X = 10 * units.meters_per_foot

    OFFSET = 180

    START_LAWFUL = Pose2d(INITATION_LINE_X, 7.3, Rotation2d.fromDegrees(180))
    TRENCH_RUN_LAWFUL = Pose2d(8.3, 7.3, Rotation2d.fromDegrees(180))

    START_STEAL = Pose2d(INITATION_LINE_X, 0.7, Rotation2d.fromDegrees(180))
    TRENCH_RUN_STEAL = Pose2d(6.2, 0.7, Rotation2d.fromDegrees(180))

    START_CENTER = Pose2d(INITATION_LINE_X, 5.8, Rotation2d.fromDegrees(180))
    RENDEZVOUS_TWO_BALLS  = Pose2d(6.3, 5.5, Rotation2d.fromDegrees(120))

    IDEAL_SHOOT = Pose2d(INITATION_LINE_X + 2 * units.meters_per_foot, 5.8, Rotation2d.fromDegrees(180))

