#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import ntcore
import wpilib
from wpilib import SmartDashboard
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
import navx

import constants
import subsystems.swervemodule as swervemodule


class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(
            wpimath.units.inchesToMeters(-12.25), wpimath.units.inchesToMeters(12.25)
        )
        self.frontRightLocation = wpimath.geometry.Translation2d(
            wpimath.units.inchesToMeters(12.25), wpimath.units.inchesToMeters(12.25)
        )
        self.backLeftLocation = wpimath.geometry.Translation2d(
            wpimath.units.inchesToMeters(-12.25), wpimath.units.inchesToMeters(-12.25)
        )
        self.backRightLocation = wpimath.geometry.Translation2d(
            wpimath.units.inchesToMeters(12.25), wpimath.units.inchesToMeters(-12.25)
        )

        self.frontLeft = swervemodule.SwerveModule(25, 21, 0)
        self.frontRight = swervemodule.SwerveModule(28, 24, math.pi / 2)
        self.backLeft = swervemodule.SwerveModule(26, 22, 3 * math.pi / 2)
        self.backRight = swervemodule.SwerveModule(27, 23, math.pi)

        self.gyro = navx.AHRS.create_spi()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def updatePIDConfig(self) -> None:
        self.frontLeft.updatePIDConfig()
        self.frontRight.updatePIDConfig()
        self.backLeft.updatePIDConfig()
        self.backRight.updatePIDConfig()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, constants.kMaxSpeed
        )

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def resetOdometry(self, pose: wpimath.geometry.Pose2d) -> None:
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.odometry.getPose()

    def setAllState(self, state: wpimath.kinematics.SwerveModuleState) -> None:
        self.frontLeft.setDesiredState(state)
        self.frontRight.setDesiredState(state)
        self.backLeft.setDesiredState(state)
        self.backRight.setDesiredState(state)

    def getError(self) -> list[tuple[float, float]]:
        return [
            self.frontLeft.getError(),
            self.frontRight.getError(),
            self.backLeft.getError(),
            self.backRight.getError(),
        ]
    
    def updateTelemetry(self) -> None:
        SmartDashboard.putNumberArray("swerve/state", [
            self.frontLeft.getState().angle.radians() - math.pi / 2 - 0,  self.frontLeft.getState().speed,
            self.frontRight.getState().angle.radians() - math.pi / 2 - math.pi / 2,  self.frontRight.getState().speed,
            self.backLeft.getState().angle.radians() - math.pi / 2 - 3 * math.pi / 2,  self.backLeft.getState().speed,
            self.backRight.getState().angle.radians() - math.pi / 2 - math.pi, self.backRight.getState().speed,
        ])

        SmartDashboard.putNumberArray("swerve/rawOut", [
            self.frontLeft.driveMotor.getOutputCurrent(),
            self.frontRight.driveMotor.getOutputCurrent(),
            self.backLeft.driveMotor.getOutputCurrent(),
            self.backRight.driveMotor.getOutputCurrent(),
        ])

        SmartDashboard.putNumber("gyro", self.gyro.getRotation2d().radians())