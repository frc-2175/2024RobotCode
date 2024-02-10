#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
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

        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/frontLeft/speed", swerveModuleStates[0].speed)
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/frontLeft/angle", swerveModuleStates[0].angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/frontRight/speed", swerveModuleStates[1].speed)
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/frontRight/angle", swerveModuleStates[1].angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/backLeft/speed", swerveModuleStates[2].speed)
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/backLeft/angle", swerveModuleStates[2].angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/backRight/speed", swerveModuleStates[3].speed)
        wpilib.SmartDashboard.putNumber("swerveModuleFAKE/backRight/angle", swerveModuleStates[3].angle.degrees())

        wpilib.SmartDashboard.putNumber("swerveModuleStates/frontLeft/speed", self.frontLeft.getState().speed)
        wpilib.SmartDashboard.putNumber("swerveModuleStates/frontLeft/angle", self.frontLeft.getState().angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleStates/frontRight/speed", self.frontRight.getState().speed)
        wpilib.SmartDashboard.putNumber("swerveModuleStates/frontRight/angle", self.frontRight.getState().angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleStates/backLeft/speed", self.backLeft.getState().speed)
        wpilib.SmartDashboard.putNumber("swerveModuleStates/backLeft/angle", self.backLeft.getState().angle.degrees())
        wpilib.SmartDashboard.putNumber("swerveModuleStates/backRight/speed", self.backRight.getState().speed)
        wpilib.SmartDashboard.putNumber("swerveModuleStates/backRight/angle", self.backRight.getState().angle.degrees())

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

    def test(self, runDrive, runSteer):
        self.frontLeft.test(runDrive, runSteer)
        self.frontRight.test(runDrive, runSteer)
        self.backLeft.test(runDrive, runSteer)
        self.backRight.test(runDrive, runSteer)
