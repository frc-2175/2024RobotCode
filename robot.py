#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.event

from wpilib import SmartDashboard

import rev

import wpimath
import wpimath.filter
import wpimath.geometry
import wpimath.kinematics
import wpimath.units

import constants

import subsystems.swervemodule as swervemodule
from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.shooter import Shooter

from utils.swerveheading import SwerveHeadingController, SwerveHeadingState
import utils.math

from wpilib import CameraServer

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        CameraServer.launch("camera.py")

        self.swerve = Drivetrain()

        self.arm = Arm(30, 35)

        self.shooter = Shooter(31, 32, 33)

        self.armButton = wpilib.DigitalInput(0)

        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.swerve.updatePIDConfig()

        self.headingController = SwerveHeadingController(self.swerve.gyro)

        self.swerve.gyro.reset()
        
        self.armButtonPastState = False
        
    def robotPeriodic(self) -> None:
        # swervemodule.driveP = wpilib.SmartDashboard.getNumber(
        #     "driveP", swervemodule.driveP
        # )
        # swervemodule.driveI = wpilib.SmartDashboard.getNumber(
        #     "driveI", swervemodule.driveI
        # )
        # swervemodule.driveD = wpilib.SmartDashboard.getNumber(
        #     "driveD", swervemodule.driveD
        # )
        # swervemodule.driveFF = wpilib.SmartDashboard.getNumber(
        #     "driveFF", swervemodule.driveFF
        # )
        # swervemodule.driveOutputMin = wpilib.SmartDashboard.getNumber(
        #     "driveOutputMin", swervemodule.driveOutputMin
        # )
        # swervemodule.driveOutputMax = wpilib.SmartDashboard.getNumber(
        #     "driveOutputMax", swervemodule.driveOutputMax
        # )

        # swervemodule.steerP = wpilib.SmartDashboard.getNumber(
        #     "steerP", swervemodule.steerP
        # )
        # swervemodule.steerI = wpilib.SmartDashboard.getNumber(
        #     "steerI", swervemodule.steerI
        # )
        # swervemodule.steerD = wpilib.SmartDashboard.getNumber(
        #     "steerD", swervemodule.steerD
        # )
        # swervemodule.steerFF = wpilib.SmartDashboard.getNumber(
        #     "steerFF", swervemodule.steerFF
        # )
        # swervemodule.steerOutputMin = wpilib.SmartDashboard.getNumber(
        #     "steerOutputMin", swervemodule.steerOutputMin
        # )
        # swervemodule.steerOutputMax = wpilib.SmartDashboard.getNumber(
        #     "steerOutputMax", swervemodule.steerOutputMax
        # )
        
        self.arm.updateTelemetry()
        self.swerve.updateTelemetry()
        self.shooter.updateTelemetry()

    def autonomousInit(self) -> None:
        self.autoTimer = wpilib.Timer()
        self.autoTimer.start()

    def autonomousPeriodic(self) -> None:
        self.arm.setArmPreset("low")
        self.shooter.setShooterSpeed(constants.kShooterPresets["low"])

        if self.autoTimer.hasElapsed(4.0):
            self.shooter.setIntakeSpeed(-0.8)

    def teleopPeriodic(self) -> None:
        if self.leftStick.getRawButtonPressed(8):
            self.swerve.gyro.reset()

        if self.leftStick.getRawButton(3) or self.rightStick.getRawButton(3):
            self.driveWithJoystick(False)
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
            self.arm.setArmPreset("intake")
            shooterPower = constants.kShooterPresets["intake"]
          
        if self.gamePad.getRightBumper() or self.gamePad.getAButton() or self.gamePad.getYButton():
            self.shooter.setShooterSpeed(shooterPower)
        else:
            self.shooter.setShooterSpeed(0)

        if self.gamePad.getLeftBumper():
            self.shooter.setIntakeSpeed(-0.8)
        else:
        # if self.gamePad.getBackButton > 0.5:
            self.shooter.setIntakeSpeed(wpimath.applyDeadband(-self.gamePad.getLeftY(), 0.1))
        # elif self.gamePad.getLeftTriggerAxis() > 0.5:
        #     self.shooter.setIntakeSpeed(-self.gamePad.getLeftTriggerAxis())
        # else:
        #     self.shooter.setIntakeSpeed(0)

        # if self.gamePad.getRightTriggerAxis() > 0.5:
        #     self.shooter.intakeNote()
            


    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            self.xspeedLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(self.leftStick.getX(), 0.1)
                )
            )
            * constants.kMaxSpeed
        )

        # we invert the Y axis of the joysticks
        ySpeed = (
            self.yspeedLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(-self.leftStick.getY(), 0.1)
                )
            )
            * constants.kMaxSpeed
        )
        
        rot = (
            self.rotLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(self.rightStick.getX(), 0.1)
                )
            )
            * constants.kMaxAngularSpeed
        )

        # if(self.leftStick.getRawButton(2)):
        #     ampHeight = wpimath.units.meters(5.49275)
        #     error = float(ampHeight) - float(self.swerve.getPose().y)
        
        #     ySpeed = error * 0.2

        #     rot = 90
            
        if(self.leftStick.getRawButton(1) or self.rightStick.getRawButton(1)):
            xSpeed /= 2
            ySpeed /= 2
            rot /= 2
        
        rot = self.headingController.update(xSpeed, ySpeed, rot)

        wpilib.SmartDashboard.putNumber("X Speed", xSpeed)
        wpilib.SmartDashboard.putNumber("Y Speed", ySpeed)
        wpilib.SmartDashboard.putNumber("Rotation Speed", rot)
        
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())
        
        
    def disabledPeriodic(self) -> None:
        SmartDashboard.putBoolean("arm/button", self.armButton.get())
        SmartDashboard.putBoolean("arm/buttonOld", self.armButtonPastState)

        if self.armButton.get() != self.armButtonPastState:
            self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kBrake if self.armButton.get() else rev.CANSparkMax.IdleMode.kCoast)
            self.armButtonPastState = self.armButton.get()

    def disabledExit(self) -> None:
        self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def testPeriodic(self) -> None:
        self.shooter.lowerMotor.set(1)
        self.shooter.upperMotor.set(1)
