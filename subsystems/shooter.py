import rev
import wpilib
from wpilib import SmartDashboard
import constants

class Shooter:
    upperkP = 0
    upperkI = 0
    upperkFF = 1.05/5500

    proximity = 0
    updateTimer = wpilib.Timer()
    color: wpilib.Color
    colorTimer = wpilib.Timer()

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

        self.colorSensor = rev.ColorSensorV3(wpilib.I2C.Port.kMXP)
        self.colorSensor.configureProximitySensor(rev.ColorSensorV3.ProximityResolution.k8bit, rev.ColorSensorV3.ProximityMeasurementRate.k6ms)
        self.colorSensor.configureColorSensor(rev.ColorSensorV3.ColorResolution.k13bit, rev.ColorSensorV3.ColorMeasurementRate.k25ms)

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
        self.color = self.colorSensor.getColor()
        self.proximity = self.colorSensor.getProximity()
    
    def setShooterSpeed(self, velocity):
        self.upperTarget = velocity
        self.lowerTarget = velocity
        self.upperPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)
        self.lowerPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def setIntakeSpeed(self, motorSpeed):
        self.motorIntake.set(motorSpeed * 0.5)

    def getRawColor(self):
        return self.colorSensor.getRawColor()
    
    def getProximity(self):
        return self.colorSensor.getProximity()

    def intakeNote(self):
        """Run intake until note is detected by the color sensor"""
        if self.colorSensor.getProximity() > constants.kShooterProximityThreshold:
            self.setIntakeSpeed(0.5)
        else:
            self.setIntakeSpeed(0)
        return
    
    def updateValues(self):
        if self.colorTimer.advanceIfElapsed(0.05):
            self.color = self.colorSensor.getColor()

        if self.colorTimer.advanceIfElapsed(0.012):
            self.proximity = self.colorSensor.getProximity()

    def updateTelemetry(self):
        SmartDashboard.putNumber("shooter/upperSpeed", self.upperEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/lowerSpeed", self.lowerEncoder.getVelocity())
        SmartDashboard.putNumber("shooter/upperTarget", self.upperTarget)
        SmartDashboard.putNumber("shooter/lowerTarget", self.lowerTarget)
        SmartDashboard.putNumber("shooter/proximity", self.proximity)
        SmartDashboard.putNumberArray("shooter/color", [self.color.red, self.color.green, self.color.blue])
        # SmartDashboard.putNumberArray("shooter/colorSensor/rawColor", [color.red, color.green, color.blue])
        # SmartDashboard.putNumber("shooter/colorSensor/proximity", self.getProximity())