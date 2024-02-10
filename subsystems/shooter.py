import rev
import wpilib
import constants

class Shooter:
    def __init__(self, lowerShooterMotorId, upperShooterMotorId, intakeMotorId) -> None:
        self.shooterMotorLower = rev.CANSparkMax(lowerShooterMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterMotorUpper = rev.CANSparkMax(upperShooterMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.colorSensor = rev.ColorSensorV3(wpilib.I2C.Port.kOnboard)

        #Addresses REVlib issue #55: https://github.com/robotpy/robotpy-rev/issues/55
        if wpilib.RobotBase.isSimulation():
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        else:
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushed)

        self.motorIntake.setInverted(True)

        self.shooterMotorLower.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.shooterMotorUpper.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)

        self.shooterEncoder = self.shooterMotorLower.getEncoder()

        self.shooterUpperPIDController = self.shooterMotorUpper.getPIDController()
        self.shooterLowerPIDController = self.shooterMotorLower.getPIDController()

        self.shooterUpperPIDController.setFF(0)
    
    def setShooterSpeedBoth(self, motorSpeed):
        self.shooterMotorLower.set(motorSpeed)
        self.shooterMotorUpper.set(motorSpeed)

    def setShooterSpeedLower(self, motorSpeed):
        self.shooterMotorLower.set(motorSpeed)
    
    def setShooterSpeedUpper(self, motorSpeed):
        self.shooterMotorUpper.set(motorSpeed)
    
    def setShooterSpeed(self, velocity):
        self.shooterUpperPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)
        self.shooterLowerPIDController.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity)

    def setIntakeSpeed(self, motorSpeed):
        self.motorIntake.set(motorSpeed * 0.5)

    def intakeNote(self):
        """Run intake until note is detected by the color sensor"""
        if self.colorSensor.getProximity() > constants.kShooterProximityThreshold:
            self.setIntakeSpeed(0.5)
        else:
            self.setIntakeSpeed(0)

    def shootNote(self, velocity:int):
        """Gets shooter up to speed, then outtakes note into shooter and shoots"""

        # Check if shooter is within speed range
        if velocity - constants.kShooterSpeedRange < self.shooterEncoder.getVelocity() < velocity + constants.kShooterSpeedRange:
            # Outtake until note stops being detected
            if self.colorSensor.getProximity() > constants.kShooterProximityThreshold:
                self.setIntakeSpeed(0.5)
            else:
                self.setIntakeSpeed(0)
                self.setShooterSpeed(0)
        else:
            self.setShooterSpeed(velocity)