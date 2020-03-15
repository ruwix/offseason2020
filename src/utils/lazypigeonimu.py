import ctre

from utils import units


class LazyPigeonIMU(ctre.PigeonIMU):
    """A wrapper for the PigeonIMU."""

    def __init__(self, master: ctre.BaseTalon):
        super().__init__(master)

    def getYaw(self) -> float:
        return self.getYawPitchRoll()[1][0]

    def getYawInRange(self) -> float:
        return units.angle_range(self.getHeading())
