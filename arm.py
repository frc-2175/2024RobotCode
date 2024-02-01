import rev

class Arm:
    def __init__(self, angleMotorId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

    def setArmSpeed(self, motorSpeed):
        self.angleMotor.set(motorSpeed)