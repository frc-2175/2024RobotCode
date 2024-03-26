from commands2 import Subsystem
import rev
import wpilib
from wpilib import SmartDashboard
import constants

class Shooter(Subsystem):
    upperkP = 0
    upperkI = 0
    upperkFF = 1.0/5500

    proximity = 0
    updateTimer = wpilib.Timer()
    color: wpilib.Color
    colorTimer = wpilib.Timer()

    intakeSensor = wpilib.AnalogInput(0)

    def __init__(self, lowerMotorId, upperMotorId, intakeMotorId) -> None:
        self.upperMotor = rev.CANSparkMax(upperMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.lowerMotor = rev.CANSparkMax(lowerMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.motorIntake.setInverted(True)

        self.upperMotor.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.lowerMotor.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)

        self.upperEncoder = self.upperMotor.getEncoder()
        self.lowerEncoder = self.lowerMotor.getEncoder()

        self.upperPIDController = self.upperMotor.getPIDController()
        self.lowerPIDController = self.lowerMotor.getPIDController()

        
        SmartDashboard.putNumber("shooter/lowerkI", 0.001)
        SmartDashboard.putNumber("shooter/upperkI", 0.001)

        self.upperMotor.disableVoltageCompensation()
        self.lowerMotor.disableVoltageCompensation()

        self.upperPIDController.setP(self.upperkP)
        self.lowerPIDController.setI(self.upperkI)
        self.upperPIDController.setFF(self.upperkFF)

        self.lowerPIDController.setP(0.00000)
        self.lowerPIDController.setD(0.01)
        self.lowerPIDController.setFF(0.95/5500)

        self.upperTarget = 0
        self.lowerTarget = 0

        # this is gonna be dumb
        self.updateTimer = wpilib.Timer()
        self.updateTimer.start()

        self.noteIntaked = False
    
    def setShooterSpeed(self, velocity):
        self.upperTarget = velocity
        self.lowerTarget = velocity
        self.upperPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)
        self.lowerPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def getShooterSpeed(self):
        return self.upperEncoder.getVelocity()
    
    def isShooterAtTarget(self, targetSpeed):
        if (targetSpeed - 100) < self.upperEncoder.getVelocity():
            return True
        else:
            return False

    def setIntakeSpeed(self, motorSpeed):
        self.motorIntake.set(motorSpeed * 0.5)

    def atTarget(self) -> bool:
        upperError = abs(self.upperEncoder.getVelocity() - self.upperTarget)
        lowerError = abs(self.lowerEncoder.getVelocity() - self.lowerTarget)
        return upperError < constants.kShooterTolerance and lowerError < constants.kShooterTolerance

    # def getRawColor(self):
    #     return self.colorSensor.getRawColor()
    
    # def detectColor(self, color):
       
    #    detectColor = wpilib._wpilib.Color(color) #Test to figure out the ring color

    #    self.colorMatch = rev.ColorMatch

    #    return self.colorMatch.matchColor(detectColor)



    # def getProximity(self):
    #     return self.colorSensor.getProximity()

    # def intakeNote(self):
    #     """Run intake until note is detected by the color sensor"""
    #     if self.colorSensor.getProximity() > constants.kShooterProximityThreshold:
    #         self.setIntakeSpeed(0)
    #         self.noteIntaked = True

    #     elif self.noteIntaked == False:
    #         self.setIntakeSpeed(-0.5)
    #     return
    
    # def resetNoteState(self):
    #     self.noteIntaked = False
    
    # def updateValues(self):
    #     if self.colorTimer.advanceIfElapsed(0.05):
    #         self.color = self.colorSensor.getColor()

    #     if self.colorTimer.advanceIfElapsed(0.012):
    #         self.proximity = self.colorSensor.getProximity()

    def noteDetected(self):
        if 1.5 < self.intakeSensor.getVoltage() < 2.5:
            return True
        else:
            return False

    def updateTelemetry(self):
        SmartDashboard.putNumber("shooter/upperSpeed", self.upperEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/lowerSpeed", self.lowerEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/upperTarget", self.upperTarget)
        SmartDashboard.putNumber("shooter/lowerTarget", self.lowerTarget)