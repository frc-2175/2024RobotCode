import math

import ntcore
import wpimath.filter
import wpimath.controller
import wpilib
from wpimath.units import inchesToMeters
from wpilib import SmartDashboard
from commands2 import Subsystem
from wpimath.geometry import Pose3d, Rotation3d, Rotation2d

import rev

import constants

class Arm(Subsystem):
    def __init__(self, angleMotorId, followerId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.angleMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.angleMotor.setInverted(False)
        self.followerMotor = rev.CANSparkMax(followerId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.followerMotor.follow(self.angleMotor, True)
        self.followerMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        self.feedforward = wpimath.controller.ArmFeedforward(0, 0.34, 5.85, 0.02)

        if wpilib.RobotBase.isSimulation():
            self.angleEncoder = self.angleMotor.getAlternateEncoder(rev.SparkMaxAlternateEncoder.Type.kQuadrature, 1024)
        else:
            self.angleEncoder = self.angleMotor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)

        self.anglePIDController = self.angleMotor.getPIDController()
        self.anglePIDController.setFeedbackDevice(self.angleEncoder)

        self.anglePIDController.setPositionPIDWrappingEnabled(True)
        self.anglePIDController.setPositionPIDWrappingMaxInput(math.pi)
        self.anglePIDController.setPositionPIDWrappingMinInput(-math.pi)

        #self.angleEncoder.setPositionConversionFactor(1 / 60 /  5 * math.tau)
        self.angleEncoder.setPositionConversionFactor(math.tau)
        
        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, False)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, math.radians(90)) 

        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, False)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, math.radians(0)) 

        self.anglePIDController.setFF(0)
        self.anglePIDController.setP(1 / math.radians(45))
        self.anglePIDController.setI(0)
        self.anglePIDController.setD(1)

        self.setArmAngleDegrees(0)

        self.angleMotor.setClosedLoopRampRate(1/2)

        # Mechanism2D stuff

        

        # Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
        self.mech2d = wpilib.Mechanism2d(2, 2)
        self.armPivot = self.mech2d.getRoot("ArmPivot", 1-inchesToMeters(8.797571), constants.kArmHeight)
        self.armMech = self.armPivot.appendLigament(
            "Arm",
            inchesToMeters(26.5),
            math.degrees(self.angleEncoder.getPosition()),
            6,
            wpilib.Color8Bit(wpilib.Color.kYellow),
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("arm/mechanism", self.mech2d)




        inst = ntcore.NetworkTableInstance.getDefault();
    
        self.componentTopic = inst.getStructTopic("SmartDashboard/arm/arm3d", Pose3d).publish()

    
    def getArmSpeed(self):
        return self.angleMotor.get()

    def getArmAngle(self):
        return self.angleEncoder.getPosition()
    
    def setArmAngleDegrees(self, angle:float):
        self.targetAngle = angle
        self.anglePIDController.setReference(self.targetAngle, rev.CANSparkMax.ControlType.kPosition)
    
    def setArmPreset(self, preset:str):
        """Set the arm to a preset angle (defined in constants.py)"""
        self.setArmAngleDegrees(constants.kArmPresets[preset])

    def setIdleMode(self, mode: rev.CANSparkMax.IdleMode):
        self.angleMotor.setIdleMode(mode)
        self.followerMotor.setIdleMode(mode)

    def atTarget(self):
        if self.targetAngle - math.radians(1) < self.getArmAngle() < self.targetAngle + math.radians(1):
            return True
        else:
            return

    def periodic2175(self):
        feedforward = self.feedforward.calculate(self.angleEncoder.getPosition(), self.angleEncoder.getVelocity())
        SmartDashboard.putNumber("arm/feedforward", feedforward)
        self.anglePIDController.setReference(self.targetAngle, rev.CANSparkMax.ControlType.kPosition, pidSlot=0, arbFeedforward=feedforward)

    def updateTelemetry(self):
        SmartDashboard.putNumber("arm/angle", self.getArmAngle())
        SmartDashboard.putNumber("arm/speed", self.getArmSpeed())
        SmartDashboard.putNumber("arm/target", self.targetAngle)
        self.armMech.setAngle(math.degrees(self.angleEncoder.getPosition()))
        self.componentTopic.set(Pose3d(inchesToMeters(-8.797571), 0, inchesToMeters(10.801276), Rotation3d(math.pi/2-self.getArmAngle(), 0, math.pi/2)))

