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
import arm
import shooter
import swerveheading


def squareInput(input: float, power: float = 2) -> float:
    sign = 0
    if input > 0:
        sign = 1
    elif input < 0:
        sign = -1

    return sign * abs(input) ** power


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.swerve = drivetrain.Drivetrain()

        self.arm = arm.Arm(30)

        self.shooter = shooter.Shooter(31, 32, 33)

        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.swerve.updatePIDConfig()

        self.headingController = swerveheading.SwerveHeadingController(self.swerve.gyro)

        self.swerve.gyro.reset()

    def robotPeriodic(self) -> None:
        # Log swerve module positions
        # wpilib.SmartDashboard.putNumberArray("swerve", [self.swerve.frontLeft.getPosition().angle.degrees(), 0.5])
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

        wpilib.SmartDashboard.putNumber("ArmAngle", self.arm.getArmAngle())
        wpilib.SmartDashboard.putNumber("ArmSpeed", self.arm.getArmSpeed())

        self.arm.periodic()

    def autonomousPeriodic(self) -> None:
        self.driveWithJoystick(False)

    def teleopPeriodic(self) -> None:
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

        if self.gamePad.getYButton():
            self.arm.setArmPreset("high")
            shooterPower = constants.kShooterPresets["high"]
        elif self.gamePad.getXButton():
            self.arm.setArmPreset("mid")
            shooterPower = constants.kShooterPresets["mid"]
        elif self.gamePad.getAButton():
            self.arm.setArmPreset("low")
            shooterPower = constants.kShooterPresets["low"]
        else:
            self.arm.setArmAngleDegrees(0)
            shooterPower = .5

        #self.arm.setArmSpeed(-self.gamePad.getY())
        
        intakeSpeed = 0.5
          
        if(self.gamePad.getRawAxis(2) > 0.5 ):
            self.shooter.SetShooterSpeedBoth(shooterPower)
        else:
            self.shooter.setShooterSpeedBoth(0)

        if(self.gamePad.getRawAxis(3) > 0.5):
            self.shooter.SetIntakeSpeed(-1)
        else:
            self.shooter.SetIntakeSpeed(-self.gamePad.getRawAxis(1))

        
        
        

    def testPeriodic(self) -> None:

        if (self.gamePad.getRawButton(3)):
            self.arm.setArmAngleDegrees(30)
        elif (self.gamePad.getRawButton(4)):
            self.arm.setArmAngleDegrees(90)
        elif (self.gamePad.getRawButton(1)):
            self.arm.setArmAngleDegrees(5)
        else:
            self.arm.setArmAngleDegrees(0)

        #self.arm.setArmSpeed(-self.gamePad.getY())

        shooterSpeed = 0.0

        if(self.gamePad.getRawAxis(3) > 0.5 ):
            shooterSpeed = 0.5
        
        self.shooter.SetShooterSpeedBoth(shooterSpeed)

        self.shooter.SetIntakeSpeed(-self.gamePad.getRawAxis(5))

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            self.xspeedLimiter.calculate(
                squareInput(wpimath.applyDeadband(self.leftStick.getX(), 0.1))
            )
            * constants.kMaxSpeed
        )

        # we invert the Y axis of the joysticks
        ySpeed = (
            self.yspeedLimiter.calculate(
                squareInput(wpimath.applyDeadband(-self.leftStick.getY(), 0.1))
            )
            * constants.kMaxSpeed
        )

        rot = (
            self.rotLimiter.calculate(
                squareInput(wpimath.applyDeadband(self.rightStick.getX(), 0.1))
            )
            * constants.kMaxAngularSpeed
        )

        rot = self.headingController.update(xSpeed, ySpeed, rot)

        wpilib.SmartDashboard.putNumber("X Speed", xSpeed)
        wpilib.SmartDashboard.putNumber("Y Speed", ySpeed)
        wpilib.SmartDashboard.putNumber("Rotation Speed", rot)

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
