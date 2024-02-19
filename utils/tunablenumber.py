from wpilib import SmartDashboard

class TunableNumber:
    key: str
    value: float

    def __init__(self, key: str, value: float):
        self.key = key
        self.value = value
        SmartDashboard.putNumber(key, value)


    def update(self) -> bool:
        newValue = SmartDashboard.getNumber(self.key, self.value)

        if newValue != self.value:
            self.value = newValue
            return True

        return False