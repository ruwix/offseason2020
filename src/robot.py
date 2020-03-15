#!/usr/bin/env python3

import ctre
import numpy as np
import wpilib
from magicbot import MagicRobot

from auto import trajectories
from components.chassis import Chassis
from statemachines import trajectorytracker
from utils import lazypigeonimu, lazytalonfx


class Robot(MagicRobot):

    DS_R_ID = 0
    DM_R_ID = 1
    DS_L_ID = 2
    DM_L_ID = 3

    ACTUATOR_ID = 5

    chassis: Chassis
    trajectorytracker: trajectorytracker.TrajectoryTracker

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        self.ds_r = lazytalonfx.LazyTalonFX(self.DS_R_ID)
        self.dm_r = lazytalonfx.LazyTalonFX(self.DM_R_ID)

        self.ds_l = lazytalonfx.LazyTalonFX(self.DS_L_ID)
        self.dm_l = lazytalonfx.LazyTalonFX(self.DM_L_ID)

        self.dm_l.follow(self.ds_l)
        self.dm_r.follow(self.ds_r)

        self.actuator = ctre.WPI_TalonSRX(self.ACTUATOR_ID)

        self.imu = lazypigeonimu.LazyPigeonIMU(self.actuator)

        self.driver = wpilib.Joystick(0)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
           actions"""
        try:
            # if self.driver.getRawButton(1):
            #     speed = -self.driver.getY()
            #     rotation = self.driver.getZ()
            # output_l = speed + rotation
            # output_r = speed - rotation
            output_l = 0.3
            output_r = 0.3
            self.chassis.setOutput(output_l, output_r)
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(Robot)
