import rev
import wpilib

class Shooter:
    def __init__(self, lowerShooterMotorId, upperShooterMotorId, intakeMotorId) -> None:
        self.shooterMotorLower = rev.CANSparkMax(lowerShooterMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.shooterMotorUpper = rev.CANSparkMax(upperShooterMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        #Addresses REVlib issue #55: https://github.com/robotpy/robotpy-rev/issues/55
        if wpilib.RobotBase.isSimulation():
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        else:
            self.motorIntake = rev.CANSparkMax(intakeMotorId, rev.CANSparkLowLevel.MotorType.kBrushed)

        # TODO: Figure out how to set idle mode
        # self.shooterMotorLower.IdleMode(1)
        # self.shooterMotorUpper.IdleMode(1)
    
    def SetShooterSpeedBoth(self, motorSpeed):
        self.shooterMotorLower.set(motorSpeed)
        self.shooterMotorUpper.set(motorSpeed)

    def SetShooterSpeedLower(self, motorSpeed):
        self.shooterMotorLower.set(motorSpeed)
    
    def SetShooterSpeedUpper(self, motorSpeed):
        self.shooterMotorUpper.set(motorSpeed)

    def SetIntakeSpeed(self, motorSpeed):
        self.motorIntake.set(motorSpeed * 0.5)