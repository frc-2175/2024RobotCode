
from enum import Enum
import math
import navx
import wpimath.controller
import wpilib
from wpimath.geometry import Rotation2d

class SwerveHeadingState(Enum):
    OFF = 0
    MAINTAIN = 1

class SwerveHeadingController:
    state: SwerveHeadingState

    def __init__(self, gyro: navx.AHRS) -> None:
        self.gyro = gyro
        self.state = SwerveHeadingState.OFF
        self.shouldMaintain = False
        self.goal = self.gyro.getYaw()
        self.PID = wpimath.controller.PIDController(1 / 15, 0, 0)
        self.PID.enableContinuousInput(-180, 180)

    def update(self, xSpeed: float, ySpeed: float, rot: float) -> float:
        bot_turning = math.fabs(rot) > 0.1
        bot_translating = xSpeed != 0 or ySpeed != 0
        shouldChangeToMaintain = not bot_turning and bot_translating

        wpilib.SmartDashboard.putBoolean("Bot Turning", bot_turning)
        wpilib.SmartDashboard.putBoolean("Bot Translating", bot_translating)

        if shouldChangeToMaintain:
            self.state = SwerveHeadingState.MAINTAIN
            self.PID.setSetpoint(self.goal)
        else:
            self.state = SwerveHeadingState.OFF
            self.goal = self.gyro.getYaw()
        
        if self.state == SwerveHeadingState.OFF:
            return rot
        else:
            return self.PID.calculate(self.gyro.getYaw())
        
    def setGoal(self, goal: Rotation2d): # because navX is clockwise positive, we have to negate it
        self.goal = -goal.degrees()