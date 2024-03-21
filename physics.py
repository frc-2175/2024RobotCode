#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import pyfrc
import wpilib
import wpilib.simulation

from wpilib import SmartDashboard
from wpimath import units
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose2d, Rotation2d
from pyfrc.physics.core import PhysicsInterface

import rev

from utils.math import clamp
import math
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot

import constants

class Arm:
    oldPosition: float
    def __init__(self, robot: "MyRobot") -> None:
        self.robot = robot
        self.armSim = wpilib.simulation.SingleJointedArmSim(
            gearbox=DCMotor.NEO(2),
            gearing=300,
            moi=3.25532,
            armLength=0.6731,
            minAngle=0,
            maxAngle=math.pi,
            simulateGravity=True,
            startingAngle=math.pi/2,
            # Add noise with a std-dev of 1 tick
            measurementStdDevs=[1/8192],
        )

        if type(robot.arm.angleEncoder) is not rev.SparkMaxAlternateEncoder:
            exit()
            
        self.encoderSim: rev.SparkMaxAlternateEncoder = robot.arm.angleEncoder 
        self.motorSim = robot.arm.angleMotor
        self.pid = PIDController(1 / math.radians(45), 0, 0.4)
        self.oldPosition = self.encoderSim.getPosition()

    def update(self, now: float, tm_diff: float) -> None:
        out = self.pid.calculate(self.encoderSim.getPosition(), self.robot.arm.targetAngle)
        vel = (self.encoderSim.getPosition() - self.oldPosition) / tm_diff


        voltage = wpilib.RobotController.getInputVoltage()

        # First, we set our "inputs" (voltages)
        self.armSim.setInput(
            0, clamp(
                out * voltage + self.robot.arm.feedforward.calculate(self.encoderSim.getPosition(), vel), 
                -voltage, 
                voltage
            )
        )

        # Next, we update it
        self.armSim.update(tm_diff)

        self.oldPosition = self.encoderSim.getPosition()

        # Finally, we set our simulated encoder's readings and simulated battery
        # voltage
        self.encoderSim.setPosition(self.armSim.getAngle())

    def getCurrentDraw(self) -> units.amperes:
        return self.armSim.getCurrentDraw()
    

class Shooter:
    def __init__(self, robot: "MyRobot") -> None:
        self.robot = robot
        self.upperFlywheel = wpilib.simulation.FlywheelSim(
            DCMotor.NEO(1), 1, 0.000339396, [2*math.pi/42]
        )
        self.upperEncoderSim = robot.shooter.upperEncoder 
        self.upperMotorSim = robot.shooter.upperMotor
        self.upperPID = PIDController(1 / math.radians(45), 0, 0.1)
        self.upperFF = 1.05/5500


    def update(self, now: float, tm_diff: float) -> None:
        out = self.upperPID.calculate(self.upperEncoderSim.getPosition(), self.robot.shooter.upperTarget)
        out = min(out, 1)   
        out = max(out, -1)


        # First, we set our "inputs" (voltages)
        self.upperFlywheel.setInput(
            0, out * wpilib.RobotController.getInputVoltage()
        )

        # Next, we update it
        self.upperFlywheel.update(tm_diff)

        # Finally, we set our simulated encoder's readings and simulated battery
        # voltage
        self.upperEncoderSim.setPosition(self.upperFlywheel.getAngularVelocity())

        # Update the mechanism arm angle based on the simulated arm angle
        # -> setAngle takes degrees, getAngle returns radians... >_>
        # self.arm.setAngle(math.degrees(self.upperFlywheel.getAngle()))

    def getCurrentDraw(self) -> units.amperes:
        return self.upperFlywheel.getCurrentDraw()
    

class Drive:
    def __init__(self, robot: "MyRobot") -> None:
        self.robot = robot


    def update(self, now: float, tm_diff: float) -> None:
        pose = self.robot.swerve.getPose()
        chassisSpeeds = self.robot.swerve.kinematics.toChassisSpeeds((
            self.robot.swerve.frontLeft.targetedState,
            self.robot.swerve.frontRight.targetedState,
            self.robot.swerve.backLeft.targetedState,
            self.robot.swerve.backRight.targetedState,
        ))
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, pose.rotation())
        newPose = Pose2d(
            pose.X() + chassisSpeeds.vx * tm_diff,
            pose.Y() + chassisSpeeds.vy * tm_diff,
            Rotation2d(pose.rotation().radians() + chassisSpeeds.omega*tm_diff))
        
        self.robot.swerve.resetOdometry(newPose)
        

    def getCurrentDraw(self) -> units.amperes:
        return 0

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.robot = robot

        self.arm = Arm(robot)
        self.drive = Drive(robot)
        self.physics_controller = physics_controller

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        self.arm.update(now, tm_diff)
        self.drive.update(now, tm_diff)
        # SimBattery estimates loaded battery voltage
        wpilib.simulation.RoboRioSim.setVInVoltage(
            wpilib.simulation.BatterySim.calculate([self.arm.getCurrentDraw()])
        )