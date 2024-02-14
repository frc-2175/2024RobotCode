import rev
import math
import wpimath.filter
from wpilib import SmartDashboard
import constants

class Arm:
    def __init__(self, angleMotorId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.angleMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        
        self.angleMotor.setInverted(False)

        self.angleEncoder = self.angleMotor.getEncoder()
        self.anglePIDController = self.angleMotor.getPIDController()

        self.angleEncoder.setPositionConversionFactor(1 / 60 /  5 * 360)
        
        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, 90) 

        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, 2) 

        self.anglePIDController.setFF(0)
        self.anglePIDController.setP(1/30)
        self.anglePIDController.setI(0)
        self.anglePIDController.setD(0)

        self.targetAngle = 0

        self.angleMotor.setClosedLoopRampRate(1)

    
    def getArmSpeed(self):
        return self.angleMotor.get()

    def getArmAngle(self):
        return self.angleEncoder.getPosition()
    
    def setArmAngleDegrees(self, angle:float):
        self.anglePIDController.setReference(angle, rev.CANSparkMax.ControlType.kPosition)
    
    def setArmPreset(self, preset:str):
        """Set the arm to a preset angle (defined in constants.py)"""
        self.anglePIDController.setReference(constants.kArmPresets[preset], rev.CANSparkMax.ControlType.kPosition)

    def updateTelemetry(self):
        SmartDashboard.putNumber("arm/angle", self.getArmAngle())
        SmartDashboard.putNumber("arm/speed", self.getArmSpeed())
        SmartDashboard.putNumber("arm/target", self.targetAngle)