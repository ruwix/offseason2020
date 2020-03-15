from enum import Enum

import hal
import numpy as np
import wpilib
from networktables import NetworkTables
from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.geometry import Pose2d, Rotation2d
from wpilib.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveOdometry,
    DifferentialDriveWheelSpeeds,
)

from utils import lazypigeonimu, lazytalonfx, units


class WheelState:
    def __init__(self, left=0, right=0):
        self.left = left
        self.right = right

    def __str__(self):
        return f"({self.left}, {self.right})"


class Chassis:

    # chassis physical constants
    BUMPER_WIDTH = 3.25 * units.meters_per_inch
    ROBOT_WIDTH = 30 * units.meters_per_inch + BUMPER_WIDTH
    ROBOT_LENGTH = 30 * units.meters_per_inch + BUMPER_WIDTH
    ROBOT_MASS = 100 * units.kilograms_per_pound

    TRACK_WIDTH = 24 * units.meters_per_inch
    TRACK_RADIUS = TRACK_WIDTH / 2

    WHEEL_DIAMETER = 6 * units.meters_per_inch
    WHEEL_RADIUS = WHEEL_DIAMETER / 2
    WHEEL_CIRCUMFERENCE = 2 * np.pi * WHEEL_RADIUS
    GEAR_RATIO = (48 / 14) * (50 / 16)  # 10.7142861

    # conversions
    RADIANS_PER_METER = (np.pi * 2 * GEAR_RATIO) / WHEEL_CIRCUMFERENCE
    METERS_PER_RADIAN = WHEEL_CIRCUMFERENCE / (np.pi * 2 * GEAR_RATIO)

    # motor config
    LEFT_INVERTED = False
    RIGHT_INVERTED = True

    # motor coefs
    KS = 0.149
    KV = 2.4
    KA = 0.234

    # velocity pidf gains
    VL_KP = 0.000363
    VL_KI = 0
    VL_KD = 0
    VL_KF = 0

    VR_KP = 0.000363
    VR_KI = 0
    VR_KD = 0
    VR_KF = 0

    # static objeccts
    characterization_l = SimpleMotorFeedforwardMeters(KS, KV, KA)
    characterization_r = SimpleMotorFeedforwardMeters(KS, KV, KA)
    kinematics = DifferentialDriveKinematics(TRACK_WIDTH)

    # required devices
    dm_l: lazytalonfx.LazyTalonFX
    dm_r: lazytalonfx.LazyTalonFX
    ds_l: lazytalonfx.LazyTalonFX
    ds_r: lazytalonfx.LazyTalonFX

    imu: lazypigeonimu.LazyPigeonIMU

    # constraints
    MAX_VELOCITY = 3

    class _Mode(Enum):
        Idle = 0
        PercentOutput = 1
        Velocity = 2

    def __init__(self):
        self.mode = self._Mode.Idle

        self.odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(0))

        self.desired_output = WheelState()
        self.desired_velocity = WheelState()
        self.desired_acceleration = WheelState()

        self.feedforward = WheelState()

        self.wheel_position = WheelState()
        self.prev_wheel_position = WheelState()
        self.wheel_velocity = WheelState()
        self.error_velocity = WheelState()

        self.nt = NetworkTables.getTable(f"/components/chassis")

    def setup(self):
        self.dm_l.setInverted(self.LEFT_INVERTED)
        self.dm_r.setInverted(self.RIGHT_INVERTED)

        if wpilib.RobotBase.isSimulation():
            hal.simulation.SimDeviceSim("Talon FX[0]").getBoolean("Inverted?").set(
                self.LEFT_INVERTED
            )
            hal.simulation.SimDeviceSim("Talon FX[1]").getBoolean("Inverted?").set(
                self.RIGHT_INVERTED
            )

        self.dm_l.setRadiansPerUnit(self.RADIANS_PER_METER)
        self.dm_r.setRadiansPerUnit(self.RADIANS_PER_METER)

        self.dm_l.setPIDF(
            0, self.VL_KP, self.VL_KI, self.VL_KD, self.VL_KF,
        )
        self.dm_r.setPIDF(
            0, self.VR_KP, self.VR_KI, self.VR_KD, self.VR_KF,
        )

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def setOutput(self, output_l: float, output_r: float) -> None:
        self.mode = self._Mode.PercentOutput
        self.desired_output.left = output_l
        self.desired_output.right = output_r

    def setWheelVelocity(self, velocity_l: float, velocity_r: float) -> None:
        self.mode = self._Mode.Velocity
        self.desired_velocity.left = velocity_l
        self.desired_velocity.right = velocity_r

    def setChassisVelocity(
        self, velocity_x: float, velocity_y: float, velocity_omega
    ) -> None:
        state = ChassisSpeeds(velocity_x, velocity_y, velocity_omega)
        velocity = self.kinematics.toWheelSpeeds(state)
        self.setWheelVelocity(velocity.left, velocity.right)

    def stop(self) -> None:
        """Stop all motor output."""
        self.mode = self._Mode.Idle
        if wpilib.RobotBase.isSimulation():
            self._setSimulationOutput(0, 0)
            self._setSimulationOutput(1, 0)

    def ntPutLeftRight(self, key, value):
        self.nt.putNumber(f"{key}_left", value.left)
        self.nt.putNumber(f"{key}_right", value.right)

    def updateNetworkTables(self):
        """Update network table values related to component."""
        self.ntPutLeftRight("wheel_position", self.wheel_position)
        self.ntPutLeftRight("wheel_velocity", self.wheel_velocity)
        self.ntPutLeftRight("desired_output", self.desired_output)
        self.ntPutLeftRight("desired_velocity", self.desired_velocity)
        self.ntPutLeftRight("error_velocity", self.error_velocity)
        self.ntPutLeftRight("feedforward", self.feedforward)

    def getHeading(self):
        if wpilib.RobotBase.isSimulation():
            return (
                hal.simulation.SimDeviceSim("Field2D").getDouble("rot").get()
                * units.radians_per_degree
            )
        else:
            return self.imu.getYaw()

    def getPose(self):
        x = hal.simulation.SimDeviceSim("Field2D").getDouble("x").get()
        y = hal.simulation.SimDeviceSim("Field2D").getDouble("y").get()
        rot = hal.simulation.SimDeviceSim("Field2D").getDouble("rot").get()* units.radians_per_degree

        return Pose2d(x,y,Rotation2d(rot)) # self.odometry.getPose()

    def _setSimulationOutput(self, id, output):
        hal.simulation.SimDeviceSim(f"Talon FX[{id}]").getDouble("Motor Output").set(
            output
        )

    def _getSimulationPosition(self, id):
        return (
            hal.simulation.SimDeviceSim(f"Custom Talon FX[{id}]")
            .getDouble("Position")
            .get()
        )

    def execute(self):
        dt = 0.02

        self.wheel_position.left = self.dm_l.getPosition()
        self.wheel_position.right = self.dm_r.getPosition()

        if wpilib.RobotBase.isSimulation():
            self.wheel_position.left = self._getSimulationPosition(0)
            self.wheel_position.right = -self._getSimulationPosition(1)

        self.odometry.update(
            Rotation2d(self.getHeading()),
            self.wheel_position.left,
            self.wheel_position.right,
        )

        self.wheel_velocity.left = (
            self.wheel_position.left - self.prev_wheel_position.left
        ) / dt
        self.wheel_velocity.right = (
            self.wheel_position.right - self.prev_wheel_position.right
        ) / dt

        self.error_velocity.left = self.desired_velocity.left - self.wheel_velocity.left
        self.error_velocity.right = (
            self.desired_velocity.right - self.wheel_velocity.right
        )

        if self.mode == self._Mode.Idle:
            self.dm_l.setOutput(0)
            self.dm_r.setOutput(0)
        elif self.mode == self._Mode.PercentOutput:
            self.dm_l.setOutput(self.desired_output.left)
            self.dm_r.setOutput(self.desired_output.right)

            if wpilib.RobotBase.isSimulation():
                self._setSimulationOutput(
                    0, self.desired_output.left * self.MAX_VELOCITY
                )
                self._setSimulationOutput(
                    1, self.desired_output.right * self.MAX_VELOCITY
                )

        elif self.mode == self._Mode.Velocity:
            self.desired_acceleration.left = (
                self.desired_velocity.left - self.wheel_velocity.left
            ) / dt
            self.desired_acceleration.right = (
                self.desired_velocity.right - self.wheel_velocity.right
            ) / dt

            self.feedforward.left = (
                self.characterization_l.calculate(
                    self.desired_velocity.left, self.desired_acceleration.left
                )
                / 12
            )
            self.feedforward.right = (
                self.characterization_r.calculate(
                    self.desired_velocity.right, self.desired_acceleration.right
                )
                / 12
            )

            self.dm_l.setVelocity(
                self.desired_velocity.left, self.feedforward.left,
            )
            self.dm_r.setVelocity(
                self.desired_velocity.right, self.feedforward.right,
            )
            if wpilib.RobotBase.isSimulation():
                self._setSimulationOutput(0, self.desired_velocity.left)
                self._setSimulationOutput(1, self.desired_velocity.right)

        self.prev_wheel_position.left = self.wheel_position.left
        self.prev_wheel_position.right = self.wheel_position.right

        self.updateNetworkTables()
