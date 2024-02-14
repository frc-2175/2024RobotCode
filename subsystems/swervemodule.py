#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.units
import rev

import constants

driveP = 0.1
driveI = 0
driveD = 0
driveFF = 1 / constants.kMaxSpeed
driveOutputMin = -0.5
driveOutputMax = 0.5

steerP = 1
steerI = 0
steerD = 0
steerFF = 0
steerOutputMin = -0.5
steerOutputMax = 0.5

wpilib.SmartDashboard.putNumber("driveP", driveP)
wpilib.SmartDashboard.putNumber("driveI", driveI)
wpilib.SmartDashboard.putNumber("driveD", driveD)
wpilib.SmartDashboard.putNumber("driveFF", driveFF)
wpilib.SmartDashboard.putNumber("driveOutputMin", driveOutputMin)
wpilib.SmartDashboard.putNumber("driveOutputMax", driveOutputMax)

wpilib.SmartDashboard.putNumber("steerP", steerP)
wpilib.SmartDashboard.putNumber("steerI", steerI)
wpilib.SmartDashboard.putNumber("steerD", steerD)
wpilib.SmartDashboard.putNumber("steerFF", steerFF)
wpilib.SmartDashboard.putNumber("steerOutputMin", steerOutputMin)
wpilib.SmartDashboard.putNumber("steerOutputMax", steerOutputMax)

class SwerveModule:
    targetedState: wpimath.kinematics.SwerveModuleState

    def __init__(
        self,
        driveMotorId: int,
        steerMotorId: int,
        angularOffset: float,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, steer motor, drive encoder and steer encoder.

        :param driveMotorId:    CAN id for the drive motor.
        :param steerMotorId:    CAN id for the steer motor.
        """
        self.driveMotor = rev.CANSparkMax(driveMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.steerMotor = rev.CANSparkMax(steerMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.angularOffset = angularOffset

        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerEncoder = self.steerMotor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self.steerEncoder.setInverted(True)

        self.drivePIDController = self.driveMotor.getPIDController()
        self.steerPIDController = self.steerMotor.getPIDController()
        self.steerPIDController.setPositionPIDWrappingEnabled(True)
        self.steerPIDController.setPositionPIDWrappingMaxInput(math.pi)
        self.steerPIDController.setPositionPIDWrappingMinInput(-math.pi)

        # Set the distance traveled per rotation for the drive motor.
        self.driveEncoder.setPositionConversionFactor(
            math.pi * constants.kWheelDiameter / constants.kDriveMotorReduction
        )

        # 
        self.driveEncoder.setVelocityConversionFactor(
            (math.pi * constants.kWheelDiameter / constants.kDriveMotorReduction) / 60.0
        )

        # The Spark MAX gives us its position in rotations, so we need to convert it to radians.
        self.steerEncoder.setPositionConversionFactor(math.tau)

        # Set the steering PID controller to drive using position(angle) instead of velocity/
        self.steerPIDController.setReference(0, rev.CANSparkMax.ControlType.kPosition)

        self.targetedState = wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition()),
        )

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition()),
        )
    
    def updatePIDConfig(self) -> None:
        self.drivePIDController.setP(driveP)
        self.drivePIDController.setI(driveI)
        self.drivePIDController.setD(driveD)
        self.drivePIDController.setFF(driveFF)
        self.drivePIDController.setOutputRange(driveOutputMin, driveOutputMax)

        self.steerPIDController.setP(steerP)
        self.steerPIDController.setI(steerI)
        self.steerPIDController.setD(steerD)
        self.steerPIDController.setFF(steerFF)
        self.steerPIDController.setOutputRange(driveOutputMin, driveOutputMax)

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        desiredState.angle += wpimath.geometry.Rotation2d(self.angularOffset)

        encoderRotation = wpimath.geometry.Rotation2d(self.steerEncoder.getPosition())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        self.targetedState = state

        # Calculate the drive output from the drive PID controller.
        self.drivePIDController.setReference(
            state.speed,
            rev.CANSparkMax.ControlType.kVelocity,
        )

        # Calculate the steer motor output from the steer PID controller.
        self.steerPIDController.setReference(
            state.angle.radians(),
            rev.CANSparkMax.ControlType.kPosition,
        )
        
    def getError(self) -> tuple[float, float]:
        driveError = self.driveEncoder.getVelocity()

        steerError = self.targetedState.angle - wpimath.geometry.Rotation2d(self.steerEncoder.getPosition())

        return (driveError, steerError.radians())