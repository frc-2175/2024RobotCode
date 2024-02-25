import rev
import math
import wpimath.filter
from wpilib import SmartDashboard
import constants

class Arm:
    def __init__(self, angleMotorId, followerId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.angleMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.followerMotor = rev.CANSparkMax(followerId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.followerMotor.follow(self.angleMotor, True)

        self.angleMotor.setInverted(False)

        self.angleEncoder = self.angleMotor.getAbsoluteEncoder(rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        self.anglePIDController = self.angleMotor.getPIDController()
        self.anglePIDController.setFeedbackDevice(self.angleEncoder)

        self.anglePIDController.setPositionPIDWrappingEnabled(True)
        self.anglePIDController.setPositionPIDWrappingMaxInput(-180)
        self.anglePIDController.setPositionPIDWrappingMinInput(180)

        #self.angleEncoder.setPositionConversionFactor(1 / 60 /  5 * 360)
        self.angleEncoder.setPositionConversionFactor(360)

        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, 90) 

        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, 2) 

        self.anglePIDController.setFF(0)
        self.anglePIDController.setP(1/45)
        self.anglePIDController.setI(0)
        self.anglePIDController.setD(0.01)

        self.setArmAngleDegrees(0)

        self.angleMotor.setClosedLoopRampRate(1/2)

    
    def getArmSpeed(self):
        return self.angleMotor.get()

    def getArmAngle(self):
        return self.angleEncoder.getPosition()
    
    def setArmAngleDegrees(self, angle:float):
        self.targetAngle = angle
        self.anglePIDController.setReference(angle, rev.CANSparkMax.ControlType.kPosition)
    
    def setArmPreset(self, preset:str):
        """Set the arm to a preset angle (defined in constants.py)"""
        self.setArmAngleDegrees(constants.kArmPresets[preset])

    def setIdleMode(self, mode: rev.CANSparkMax.IdleMode):
        self.angleMotor.setIdleMode(mode)

    def updateTelemetry(self):
        SmartDashboard.putNumber("arm/angle", self.getArmAngle())
        SmartDashboard.putNumber("arm/speed", self.getArmSpeed())
        SmartDashboard.putNumber("arm/target", self.targetAngle)
        SmartDashboard.putNumber("arm/measured", self.angleEncoder.getPosition())