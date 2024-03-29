#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# This is to help vscode
from typing import TYPE_CHECKING, Callable

from commands2 import Command, CommandScheduler
import wpilib
import wpilib.event

from wpilib import SmartDashboard
import wpilib.shuffleboard

import rev

import wpimath
import wpimath.filter
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
import wpimath.controller

import constants
from subsystems.drivetrain import Drivetrain
from subsystems.arm import Arm
from subsystems.shooter import Shooter
from subsystems.vision import Vision
from utils.coroutinecommand import RestartableCommand, commandify

import utils.math

from wpilib import CameraServer

from utils.gentools import doneable


from pathplannerlib.auto import PathPlannerAuto, AutoBuilder, PathPlannerPath, NamedCommands

from utils.swerveheading import SwerveHeadingState

field = wpilib.Field2d()

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        # CameraServer.launch("camera.py")

        # Command scheduler
        self.scheduler = CommandScheduler.getInstance()

        self.swerve = Drivetrain()
        self.arm = Arm(30, 35)
        self.shooter = Shooter(31, 32, 33)
        self.scheduler.registerSubsystem(
            self.swerve,
            self.arm,
            self.shooter,
        )

        # We call this registerNamedCommand wrapper to ensure statically that all
        # our PathPlanner event commands are restartable.
        def registerNamedCommand(name: str, cmd: Callable[[], Command]):
            NamedCommands.registerCommand(name, RestartableCommand(cmd))

        registerNamedCommand('shootArm', commandify(self.shootArm))
        registerNamedCommand("setArmIntake", commandify(self.setArmIntake))
        registerNamedCommand("setArmSpeaker", commandify(self.setArmSpeaker))
        registerNamedCommand("setArmAmp", commandify(self.setArmAmp))
        registerNamedCommand("intakeNote", commandify(self.intakeNote))
        registerNamedCommand("shootNote", commandify(self.shootNote))
        registerNamedCommand("stopIntake", commandify(self.stopIntake))

        self.swerve.setupPathPlanner()
        self.armButton = wpilib.DigitalInput(0)

        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.gamePad = wpilib.XboxController(2)

        # Slew rate limiters to make joystick inputs more gentle
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(8)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(8)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(12)

        self.swerve.updatePIDConfig()

        self.swerve.gyro.reset()
        
        self.armButtonPastState = False

        self.vision = Vision()

        self.autoPID = wpimath.controller.PIDController(1 / 2, 0, 0.05)

        # In order to play nice with PathPlanner's autos, which are Command-only,
        # we always store Commands in our auto chooser. However, since we want our
        # autos to be restartable, we use these wrappers to ensure that all our auto
        # options can be wrapped in RestartableCommand.
        self.autoChooser = wpilib.SendableChooser()
        def setDefaultAuto(name: str, cmd: Callable[[], Command]):
            self.autoChooser.setDefaultOption(name, RestartableCommand(cmd))
        def addAuto(name: str, cmd: Callable[[], Command]):
            self.autoChooser.addOption(name, RestartableCommand(cmd))

        setDefaultAuto("None", commandify(self.doNothingAuto))
        addAuto("Two Note", commandify(self.twoNoteAuto))
        addAuto("Two Note Driver Right", commandify(self.twoNoteDriverRightAuto))
        addAuto("aslkjaslkfd", lambda: PathPlannerAuto("New Auto"))
        addAuto("Auto Test", lambda: PathPlannerAuto("Two Note"))
        addAuto("Three Note", lambda: PathPlannerAuto("Three Note"))
        addAuto("Four Note", lambda: PathPlannerAuto("Four Note"))
        addAuto("Two Nothing", commandify(self.twoNothing))
        SmartDashboard.putData("Auto selection", self.autoChooser)

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

        field.getObject("origin").setPose(wpimath.geometry.Pose2d())
        poses = self.vision.update()
        self.swerve.updateVision(poses)
        self.swerve.updateOdometry()
        field.setRobotPose(self.swerve.getPose())

        self.arm.periodic2175()
        self.scheduler.run()

        self.arm.updateTelemetry()
        self.swerve.updateTelemetry()
        self.shooter.updateTelemetry()
        SmartDashboard.putData(field)

        # TODO: This should almost certainly move to teleopPeriodic.
        if self.leftStick.getRawButtonPressed(8):
            self.swerve.gyro.reset()

    def autonomousInit(self) -> None:
        self.scheduler.cancelAll()
        self.swerve.gyro.reset()
        self.swerve.headingController.setState(SwerveHeadingState.DISABLE)

        autoCommand = self.autoChooser.getSelected()
        self.scheduler.schedule(autoCommand)

    def autonomousPeriodic(self) -> None:
        # No code necessary. The CommandScheduler will continue to run the command
        # scheduled by autonomousInit.
        return

    def teleopInit(self) -> None:
        self.swerve.headingController.setState(SwerveHeadingState.OFF)
        self.scheduler.cancelAll()

        self.intakeLocked = False

    def teleopPeriodic(self) -> None:
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

        intakeSpeed = wpimath.applyDeadband(-self.gamePad.getLeftY(), 0.1)

        if self.shooter.noteDetected():
            if 0 < -self.gamePad.getLeftY():
                intakeSpeed = -self.gamePad.getLeftY()
            else:
                intakeSpeed = 0

        if self.gamePad.getLeftBumper() or self.gamePad.getRightBumper():
            self.shooter.setShooterSpeed(shooterPower)
            if self.shooter.isShooterAtTarget(shooterPower):
                intakeSpeed = -1
        elif self.gamePad.getAButton() or self.gamePad.getYButton() or self.gamePad.getXButton():
            self.shooter.setShooterSpeed(shooterPower)
        else:
            self.shooter.setShooterSpeed(0)

        self.shooter.setIntakeSpeed(intakeSpeed)


    def driveWithJoystick(self, fieldRelative: bool) -> None:
        xSpeed = (
            self.xspeedLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(-self.leftStick.getY(), 0.1)
                )
            )
            * constants.kMaxSpeed
        )

        # we invert the Y axis of the joysticks
        ySpeed = (
            self.yspeedLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(-self.leftStick.getX(), 0.1)
                )
            )
            * constants.kMaxSpeed
        )
        
        rot = (
            self.rotLimiter.calculate(
                utils.math.signedPower(
                    wpimath.applyDeadband(-self.rightStick.getX(), 0.1)
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

        wpilib.SmartDashboard.putNumber("X Speed", xSpeed)
        wpilib.SmartDashboard.putNumber("Y Speed", ySpeed)
        wpilib.SmartDashboard.putNumber("Rotation Speed", rot)
        
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())


    def disabledInit(self) -> None:
        self.scheduler.cancelAll()

        
    def disabledPeriodic(self) -> None:
        SmartDashboard.putBoolean("arm/button", self.armButton.get())
        SmartDashboard.putBoolean("arm/buttonOld", self.armButtonPastState)

        if self.armButton.get() != self.armButtonPastState:
            self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kBrake if self.armButton.get() else rev.CANSparkMax.IdleMode.kCoast)
            self.armButtonPastState = self.armButton.get()

    def disabledExit(self) -> None:
        self.arm.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

    def testPeriodic(self) -> None:
        self.shooter.lowerMotor.set(1)
        self.shooter.upperMotor.set(1)

    # --------------------------------------------------------------------
    # --------------------------------------------------------------------
    # --------------------------------------------------------------------

    @doneable
    def driveToPoint(self, point: wpimath.geometry.Pose2d):
        while True:
            yield
            

            direction = point.translation() - self.swerve.getPose().translation()
            error = direction.norm()

            if error < 0.1:
                self.swerve.drive(0, 0, 0, True, self.getPeriod())
                return

            pidOut = self.autoPID.calculate(-error) # Negated because normally you don't use error as your process variable
            
            clampMag = 0.25
            if pidOut > clampMag:
                pidOut = clampMag
            elif pidOut < 0:
                pidOut = 0

            normalized = (direction / direction.norm())
            outputX = normalized.X() * pidOut * constants.kMaxSpeed
            outputY = normalized.Y() * pidOut * constants.kMaxSpeed
            # print(direction, error, pidOut, (outputX, outputY))

            #TODO this should be field relative
            self.swerve.drive(outputX, outputY, 0, False, self.getPeriod(), point.rotation())

            # print(self.swerve.getPose().translation())

    @doneable
    def setArmSpeaker(self):
        print("going to speaker preset")
        self.arm.setArmPreset("low")
        self.shooter.setShooterPreset("low")
        yield

    @doneable    
    def setArmAmp(self):
        print("going to amp preset")
        self.arm.setArmPreset("high")
        self.shooter.setShooterPreset("high")
        yield

    @doneable
    def setArmIntake(self):
        print("going to intake preset")
        self.arm.setArmPreset("intake")
        self.shooter.setShooterSpeed(0)
        yield

    

    @doneable
    def doNothingAuto(self):
        print("Doing nothing...")
        yield from sleep(2)
        print("Still doing nothing...")
        yield from sleep(2)
        print("Done doing nothing :)")

    @doneable
    def twoNothing(self):
        nothingOne = self.doNothingAuto()
        nothingTwo = self.doNothingAuto()

        while not (nothingOne.done or nothingTwo.done):
            nothingOne.resume()
            nothingTwo.resume()
            yield

    @doneable
    def oneNoteAuto(self):
        yield from self.shootNote()

    @doneable
    def intakeNote(self):
        print("intaking note")
        self.arm.setArmPreset("intake")
        self.shooter.setIntakeSpeed(-0.5)
        while not self.shooter.noteDetected():
            yield
        self.shooter.setIntakeSpeed(0)
        print("done intaking note")

    @doneable
    def twoNoteAuto(self):
        yield from self.shootNote()
        yield from sleep(1)
        # # TODO: real poses
        print("Driving to note")
        intakeInst = self.intakeNote()
        driveToPointOne = self.driveToPoint(wpimath.geometry.Pose2d(2, 0, wpimath.geometry.Rotation2d()))

        while not (driveToPointOne.done):
            intakeInst.resume()
            driveToPointOne.resume()

        self.shooter.setIntakeSpeed(0)
        print("Driving back to speaker")
        yield from self.driveToPoint(wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d()))

        print("Adjusting note")
        self.shooter.setIntakeSpeed(0.2)
        yield from sleep(0.25)
        self.shooter.setIntakeSpeed(0)

        yield from self.shootNote()

    @doneable
    def reverseIntake(self):
        self.shooter.setIntakeSpeed(0.2)
        yield from sleep(0.25)

    @doneable
    def stopIntake(self):
        self.shooter.setIntakeSpeed(0)
        yield

    @doneable
    def twoNoteDriverRightAuto(self):
        yield from self.shootNote()
        yield from sleep(1)
        # # TODO: real poses
        print("Driving to note")

        yield from self.driveToPoint(wpimath.geometry.Pose2d(1.068, 1.331, wpimath.geometry.Rotation2d.fromDegrees(60)))
        self.shooter.setIntakeSpeed(0)
        print("Driving back to speaker")
        yield from self.driveToPoint(wpimath.geometry.Pose2d(0, 0, wpimath.geometry.Rotation2d()))

        print("Adjusting note")
        self.shooter.setIntakeSpeed(0.2)
        yield from sleep(0.25)
        self.shooter.setIntakeSpeed(0)

        yield from self.shootNote()

    @doneable
    def shootArm(self):
        self.arm.setArmPreset("low")
        yield from sleep(1)

    @doneable
    def spinShooter(self):
        self.shooter.setShooterSpeed(constants.kShooterPresets["low"])
        while not self.shooter.atTarget():
            yield

    @doneable
    def finishShot(self):
        self.shooter.setIntakeSpeed(-0.8)
        yield from sleep(0.25)
        self.shooter.setIntakeSpeed(0)
        self.shooter.setShooterSpeed(0)
        self.arm.setArmPreset("intake")

    @doneable
    def shootNote(self):
        print("Shooting note")
        self.shooter.setIntakeSpeed(0)
        self.arm.setArmPreset("low")
        self.shooter.setShooterPreset("low")

        t = AutoTimer(3)
        while not (self.shooter.atTarget() and self.arm.atTarget()) and not t.expired():
            yield
        
        self.shooter.setIntakeSpeed(-0.8)
        yield from sleep(1)
        self.shooter.setIntakeSpeed(0)
        self.shooter.setShooterSpeed(0)

        self.arm.setArmPreset("intake")



    @doneable
    def firstShot(self):
        self.shooter.setShooterSpeed(constants.kShooterPresets["low"])
        yield from self.shootArm()
        while not self.shooter.atTarget():
            yield
        self.shooter.setIntakeSpeed(-0.8)
        yield from sleep(0.5)
        self.shooter.setShooterSpeed(0)
        self.arm.setArmPreset("intake") 

    @doneable
    def secondThing(self):
        self.shooter.setIntakeSpeed(0)
        self.shooter.setShooterSpeed(constants.kShooterPresets["low"])
        yield from self.shootArm()
        while not self.shooter.atTarget():
            yield
        
        self.shooter.setIntakeSpeed(-0.8)
        yield from sleep(0.5)
        self.shooter.setShooterSpeed(0)
        self.arm.setArmPreset("intake")        

    @doneable
    def prepareIntake2(self):
        print("preparing")
        self.arm.setArmPreset("intake")
        self.shooter.setShooterSpeed(-100)
        self.shooter.setIntakeSpeed(-0.8)
        yield
        print("done")

@doneable
def sleep(duration: float):
    t = wpilib.Timer()
    t.start()
    while not t.hasElapsed(duration):
        yield

class AutoTimer:
    def __init__(self, duration: float):
        self.duration = duration
        self.t = wpilib.Timer()
        self.t.start()
        
    def expired(self):
        if self.t.hasElapsed(self.duration):
            return True
        else:
            return False

    
