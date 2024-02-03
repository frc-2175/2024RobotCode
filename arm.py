import rev

class Arm:
    def __init__(self, angleMotorId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        # TODO: this is not how you set idle mode
        # self.angleMotor.IdleMode(1)
        self.angleMotor.setInverted(True)

    def setArmSpeed(self, motorSpeed):
        self.angleMotor.set(motorSpeed)