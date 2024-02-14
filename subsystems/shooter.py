import rev
import wpilib
from wpilib import SmartDashboard
import constants

class Shooter:
    def __init__(self, lowerMotorId, upperMotorId, intakeMotorId) -> None:
        self.upperMotor = rev.CANSparkMax(upperMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.lowerMotor = rev.CANSparkMax(lowerMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        # self.colorSensor = rev.ColorSensorV3(wpilib.I2C.Port.kOnboard)

        #Addresses REVlib issue #55: https://github.com/robotpy/robotpy-rev/issues/55
        if wpilib.RobotBase.isSimulation():
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        else:
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushed)

        self.motorIntake.setInverted(True)

        self.upperMotor.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.lowerMotor.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)

        self.upperEncoder = self.upperMotor.getEncoder()
        self.lowerEncoder = self.lowerMotor.getEncoder()

        self.upperPIDController = self.upperMotor.getPIDController()
        self.lowerPIDController = self.lowerMotor.getPIDController()

        self.upperPIDController.setP(0.00001)
        self.upperPIDController.setFF(1/4000)

        self.lowerPIDController.setP(0.00001)
        self.lowerPIDController.setFF(1/4000)

        self.upperTarget = 0
        self.lowerTarget = 0
    
    def setShooterSpeed(self, velocity):
        self.upperTarget = velocity
        self.lowerTarget = velocity
        self.upperPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)
        self.lowerPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def setIntakeSpeed(self, motorSpeed):
        self.motorIntake.set(motorSpeed * 0.5)

    def getRawColor(self):
        # return self.colorSensor.getRawColor()
        return
    
    def getProximity(self):
        return # return self.colorSensor.getProximity()

    def intakeNote(self):
        # """Run intake until note is detected by the color sensor"""
        # if self.colorSensor.getProximity() > constants.kShooterProximityThreshold:
        #     self.setIntakeSpeed(0.5)
        # else:
        #     self.setIntakeSpeed(0)
        return

    def updateTelemetry(self):
        SmartDashboard.putNumber("shooter/upperSpeed", self.upperEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/lowerSpeed", self.lowerEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/upperTarget", self.upperEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/lowerTarget", self.lowerEncoder.getVelocity())