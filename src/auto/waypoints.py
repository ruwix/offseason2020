from wpilib.geometry import Pose2d, Rotation2d

from utils import units


class Waypoints:
    MIN_FIELD_X = 0
    MAX_FIELD_X = 54 * units.meters_per_foot
    MIN_FIELD_Y = 0
    MAX_FIELD_Y = 27 * units.meters_per_foot

    INITATION_LINE_X = MAX_FIELD_X - 10 * units.meters_per_foot

    # START_CENTER = Pose2d(
    #     INITATION_LINE_X,
    #     10 * units.meters_per_foot,
    #     Rotation2d.fromDegrees(0),
    # )
    START_CENTER = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    BALL_0 = Pose2d(2, 2, Rotation2d.fromDegrees(90))
    BALL_1 = Pose2d(4, 4, Rotation2d.fromDegrees(0))
    BALL_2 = Pose2d(6, 2, Rotation2d.fromDegrees(-90))
    BALL_3 = Pose2d(8, 0, Rotation2d.fromDegrees(0))

    # BALL_0 = Pose2d(
    #     INITATION_LINE_X + 1,
    #     11 * units.meters_per_foot,
    #     Rotation2d.fromDegrees(0),
    # )
