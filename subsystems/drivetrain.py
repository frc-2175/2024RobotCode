#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import ntcore
import wpilib
from wpilib import SmartDashboard
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
import navx

import constants
import subsystems.swervemodule as swervemodule

from utils.swerveheading import SwerveHeadingController



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

        self.frontLeft = swervemodule.SwerveModule(25, 21, math.pi / 2)
        self.frontRight = swervemodule.SwerveModule(28, 24, math.pi)
        self.backLeft = swervemodule.SwerveModule(26, 22, 0)
        self.backRight = swervemodule.SwerveModule(27, 23, 3 * math.pi / 2)

        self.gyro = navx.AHRS.create_spi()

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            wpimath.geometry.Pose2d(),
        )

        self.headingController = SwerveHeadingController(self.gyro)

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
        angle: wpimath.geometry.Rotation2d | None = None
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if angle is not None:
            self.headingController.setGoal(angle)

        rot = self.headingController.update(xSpeed, ySpeed, rot)

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

    def setPose(self, pose: wpimath.geometry.Pose2d) -> None:
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose
        )

    def updateVision(self, poses: list[tuple[wpimath.geometry.Pose2d, float]]) -> None:
        for (pose, timestamp) in poses:
            self.odometry.addVisionMeasurement(pose, timestamp)

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
        return self.odometry.getEstimatedPosition()


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
            self.frontLeft.getState().angle.radians() - self.frontLeft.angularOffset,  self.frontLeft.getState().speed,
            self.frontRight.getState().angle.radians() - self.frontRight.angularOffset,  self.frontRight.getState().speed,
            self.backLeft.getState().angle.radians() - self.backLeft.angularOffset,  self.backLeft.getState().speed,
            self.backRight.getState().angle.radians() - self.backRight.angularOffset, self.backRight.getState().speed,
        ])
        
        SmartDashboard.putNumberArray("swerve/target", [
            self.frontLeft.targetedState.angle.radians() - self.frontLeft.angularOffset,  self.frontLeft.targetedState.speed,
            self.frontRight.targetedState.angle.radians() - self.frontRight.angularOffset,  self.frontRight.targetedState.speed,
            self.backLeft.targetedState.angle.radians() - self.backLeft.angularOffset,  self.backLeft.targetedState.speed,
            self.backRight.targetedState.angle.radians() - self.backRight.angularOffset, self.backRight.targetedState.speed,
        ])

        SmartDashboard.putNumberArray("swerve/current", [
            self.frontLeft.driveMotor.getOutputCurrent(),
            self.frontRight.driveMotor.getOutputCurrent(),
            self.backLeft.driveMotor.getOutputCurrent(),
            self.backRight.driveMotor.getOutputCurrent(),
        ])

        SmartDashboard.putNumber("gyro", self.gyro.getRotation2d().radians())