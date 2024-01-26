#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import wpilib
import wpilib.drive
from wpilib.shuffleboard import Shuffleboard

import wpimath
import wpimath.filter
import wpimath.controller
import wpimath.geometry
import wpimath.units
import wpimath.kinematics

import constants
import drivetrain
import swervemodule


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.swerve = drivetrain.Drivetrain()

        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.field = wpilib.Field2d()

        self.drivetrainTab = Shuffleboard.getTab("Drivetrain")

        # self.swerve.gyro.setAngleAdjustment(0)

    def robotPeriodic(self) -> None:
        swervemodule.driveP = wpilib.SmartDashboard.getNumber(
            "driveP", swervemodule.driveP
        )
        swervemodule.driveI = wpilib.SmartDashboard.getNumber(
            "driveI", swervemodule.driveI
        )
        swervemodule.driveD = wpilib.SmartDashboard.getNumber(
            "driveD", swervemodule.driveD
        )
        swervemodule.driveFF = wpilib.SmartDashboard.getNumber(
            "driveFF", swervemodule.driveFF
        )
        swervemodule.driveOutputMin = wpilib.SmartDashboard.getNumber(
            "driveOutputMin", swervemodule.driveOutputMin
        )
        swervemodule.driveOutputMax = wpilib.SmartDashboard.getNumber(
            "driveOutputMax", swervemodule.driveOutputMax
        )

        swervemodule.steerP = wpilib.SmartDashboard.getNumber(
            "steerP", swervemodule.steerP
        )
        swervemodule.steerI = wpilib.SmartDashboard.getNumber(
            "steerI", swervemodule.steerI
        )
        swervemodule.steerD = wpilib.SmartDashboard.getNumber(
            "steerD", swervemodule.steerD
        )
        swervemodule.steerFF = wpilib.SmartDashboard.getNumber(
            "steerFF", swervemodule.steerFF
        )
        swervemodule.steerOutputMin = wpilib.SmartDashboard.getNumber(
            "steerOutputMin", swervemodule.steerOutputMin
        )
        swervemodule.steerOutputMax = wpilib.SmartDashboard.getNumber(
            "steerOutputMax", swervemodule.steerOutputMax
        )

        wpilib.SmartDashboard.putData(self.field)

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.field.setRobotPose(self.swerve.getPose())
        if self.leftStick.getRawButtonPressed(8):
            self.swerve.gyro.reset()

        if self.leftStick.getRawButton(2):  # down
            self.swerve.setAllState(
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(180)
                )
            )
        elif self.leftStick.getRawButton(3):  # up
            self.swerve.setAllState(
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(0)
                )
            )
        elif self.leftStick.getRawButton(4):  # left
            self.swerve.setAllState(
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(270)
                )
            )
        elif self.leftStick.getRawButton(5):  # right
            self.swerve.setAllState(
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(90)
                )
            )
        else:
            self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.leftStick.getX(), 0.02)
            )
            * constants.kMaxSpeed
        )

        # we invert the Y axis of the joysticks
        ySpeed = (
            self.yspeedLimiter.calculate(
                wpimath.applyDeadband(-self.leftStick.getY(), 0.02)
            )
            * constants.kMaxSpeed
        )
        
        rot = (
            self.rotLimiter.calculate(
                wpimath.applyDeadband(self.rightStick.getX(), 0.02)
            )
            * constants.kMaxAngularSpeed
        )

        wpilib.SmartDashboard.putNumber("X Speed", xSpeed)
        wpilib.SmartDashboard.putNumber("Y Speed", ySpeed)
        wpilib.SmartDashboard.putNumber("Rotation Speed", rot)

        line = self.field.getObject("moveVec")

        asdfjiko = wpimath.geometry.Translation2d(xSpeed, ySpeed).rotateBy(self.swerve.getPose().rotation())
        line.setPose(self.field.getRobotPose() + wpimath.geometry.Transform2d(asdfjiko, asdfjiko.angle()))

        
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
