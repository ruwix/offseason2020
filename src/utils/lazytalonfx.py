import logging

import ctre
import numpy as np


class LazyTalonFX(ctre.WPI_TalonFX):
    """A wraper for the ctre.WPI_TalonFX to simplfy configuration and getting/setting values."""

    TIMEOUT = 10

    ControlMode = ctre.ControlMode
    DemandType = ctre.DemandType
    StatusFrame = ctre.StatusFrameEnhanced
    NeutralMode = ctre.NeutralMode

    CPR = 2048

    def __init__(self, id: int):
        super().__init__(id)
        self.no_closed_loop_warning = f"Talon {id} not in closed loop mode"

        self.counts_per_unit = self.CPR / (2 * np.pi)
        self.units_per_count = 2 * np.pi / self.CPR

    def setRadiansPerUnit(self, rads_per_unit):
        self.counts_per_unit = rads_per_unit * (self.CPR / (2 * np.pi))
        self.units_per_count = 1 / self.counts_per_unit

    def setSupplyCurrentLimit(self, current_limit, trigger_current, trigger_time):
        limits = ctre.SupplyCurrentLimitConfiguration(
            True, current_limit, trigger_current, trigger_time
        )
        self.configSupplyCurrentLimit(limits, self.TIMEOUT)

    def setStatorCurrentLimit(self, current_limit, trigger_current, trigger_time):
        limits = ctre.StatorCurrentLimitConfiguration(
            True, current_limit, trigger_current, trigger_time
        )
        self.configStatorCurrentLimit(limits, self.TIMEOUT)

    def setPIDF(self, slot: int, kp: float, ki: float, kd: float, kf: float) -> None:
        """Initialize the PIDF controller."""
        self.selectProfileSlot(slot, self.TIMEOUT)
        self.config_kP(slot, kp, self.TIMEOUT)
        self.config_kI(slot, ki, self.TIMEOUT)
        self.config_kD(slot, kd, self.TIMEOUT)
        self.config_kF(slot, kf, self.TIMEOUT)

    def setIZone(self, slot: int, izone: float) -> None:
        """Set the izone of the PIDF controller."""
        self.config_IntegralZone(slot, int(izone * self.counts_per_unit), self.TIMEOUT)

    def setBrakeMode(self):
        self.setNeutralMode(self.NeutralMode.Brake)

    def setCoastMode(self):
        self.setNeutralMode(self.NeutralMode.Coast)

    def setMotionMagicConfig(self, vel: float, accel: float) -> None:
        self.configMotionCruiseVelocity(
            int(vel * self.counts_per_unit / 10), self.TIMEOUT
        )
        self.configMotionAcceleration(
            int(accel * self.counts_per_unit / 10), self.TIMEOUT
        )

    def setOutput(self, signal: float, max_signal: float = 1) -> None:
        """Set the percent output of the motor."""
        signal = np.clip(signal, -max_signal, max_signal)
        self.set(self.ControlMode.PercentOutput, signal)

    def setPosition(self, pos: float) -> None:
        """Set the position of the motor."""
        self.set(self.ControlMode.Position, pos * self.counts_per_unit)

    def setVelocity(self, vel: float, ff: float = 0) -> None:
        """Set the velocity of the motor."""
        self.set(
            self.ControlMode.Velocity,
            vel * self.counts_per_unit / 10,
            self.DemandType.ArbitraryFeedForward,
            ff,
        )

    def setMotionMagicPosition(self, pos: float) -> None:
        """Set the position of the motor using motion magic."""
        self.set(self.ControlMode.MotionMagic, pos * self.counts_per_unit)

    def zero(self, pos: int = 0) -> None:
        """Zero the encoder if it exists."""
        self.setSelectedSensorPosition(pos * self.counts_per_unit, 0, self.TIMEOUT)

    def getPosition(self) -> int:
        """Get the encoder position if it exists."""
        return self.getSelectedSensorPosition(0) * self.units_per_count

    def getVelocity(self) -> int:
        """Get the encoder velocity if it exists."""
        return self.getSelectedSensorVelocity(0) * self.units_per_count * 10

    def getError(self) -> int:
        """Get the closed loop error if in closed loop mode."""
        if self._isClosedLoop():
            return self.getClosedLoopError(0)
        else:
            logging.warning(self.no_closed_loop_warning)
            return 0

    def getTarget(self) -> int:
        """Get the closed loop target if in closed loop mode."""
        if self._isClosedLoop():
            return self.getClosedLoopTarget(0)
        else:
            logging.warning(self.no_closed_loop_warning)
            return 0

    def _isClosedLoop(self) -> bool:
        return self.getControlMode() in (
            self.ControlMode.Velocity,
            self.ControlMode.Position,
            self.ControlMode.MotionMagic,
        )
