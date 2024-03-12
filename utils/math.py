from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters
import constants
import math

def signedPower(input: float, power: float = 2) -> float:
    sign = 0
    if input > 0:
        sign = 1
    elif input < 0:
        sign = -1

    return sign * abs(input) ** power

def shotAngle(distance: float) -> float:
    targetPose = Translation2d(inchesToMeters(9.055), inchesToMeters(80.515))
    robotPose = Translation2d(distance, 0)
    pivotPose = robotPose + Translation2d(inchesToMeters(-8.797571), constants.kArmHeight)
    relative = targetPose - pivotPose
    armRadius = inchesToMeters(26.120528)

    pivotToTarget = math.fmod(relative.angle().radians(), math.pi)

    f2 = math.acos(armRadius * constants.kShooterOffset.cos()/relative.norm())

    return pivotToTarget - f2 - constants.kShooterOffset.radians()
