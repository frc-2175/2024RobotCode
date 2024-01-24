#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive
import wpilib.shuffleboard

import wpimath
import wpimath.filter
import wpimath.controller
import wpimath.geometry
import wpimath.units
import wpimath.kinematics

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

        # self.swerve.gyro.setAngleAdjustment(0)

    def robotPeriodic(self) -> None:
        swervemodule.driveP = wpilib.SmartDashboard.getNumber("driveP", swervemodule.driveP)
        swervemodule.driveI = wpilib.SmartDashboard.getNumber("driveI", swervemodule.driveI)
        swervemodule.driveD = wpilib.SmartDashboard.getNumber("driveD", swervemodule.driveD)
        swervemodule.driveFF = wpilib.SmartDashboard.getNumber("driveFF", swervemodule.driveFF)
        swervemodule.driveOutputMin = wpilib.SmartDashboard.getNumber("driveOutputMin", swervemodule.driveOutputMin)
        swervemodule.driveOutputMax = wpilib.SmartDashboard.getNumber("driveOutputMax", swervemodule.driveOutputMax)

        swervemodule.steerP = wpilib.SmartDashboard.getNumber("steerP", swervemodule.steerP)
        swervemodule.steerI = wpilib.SmartDashboard.getNumber("steerI", swervemodule.steerI)
        swervemodule.steerD = wpilib.SmartDashboard.getNumber("steerD", swervemodule.steerD)
        swervemodule.steerFF = wpilib.SmartDashboard.getNumber("steerFF", swervemodule.steerFF)
        swervemodule.steerOutputMin = wpilib.SmartDashboard.getNumber("steerOutputMin", swervemodule.steerOutputMin)
        swervemodule.steerOutputMax = wpilib.SmartDashboard.getNumber("steerOutputMax", swervemodule.steerOutputMax)

        # self.swerve.updatePIDConfig()

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)
        self.swerve.updateOdometry()

    def teleopPeriodic(self) -> None:
        self.driveWithJoystick(True)

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.leftStick.getX(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(-self.leftStick.getY(), 0.02)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            self.rotLimiter.calculate(
                wpimath.applyDeadband(self.rightStick.getX(), 0.02)
            )
            * drivetrain.kMaxAngularSpeed
        )

        if self.leftStick.getRawButtonPressed(8):
            self.swerve.gyro.reset()
        
        if self.leftStick.getRawButton(2): # down
            self.swerve.setAllState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(180)))
        elif self.leftStick.getRawButton(3): # up
            self.swerve.setAllState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(0)))
        elif self.leftStick.getRawButton(4): # left
            self.swerve.setAllState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(270)))
        elif self.leftStick.getRawButton(5): # right
            self.swerve.setAllState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d.fromDegrees(90)))
        else:
            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())