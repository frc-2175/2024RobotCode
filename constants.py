import math
from typing import Literal
import wpimath.units
import wpimath.geometry

kWheelDiameter = wpimath.units.inchesToMeters(3)
kTrackWidth = wpimath.units.inchesToMeters(24.5)
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

kDriveMotorReduction = (45 * 22) / (13 * 15)
kDriveMotorFreeSpeed = 5676 / 60 # 94.6 rev/s

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction # 4.46 m/s
kMaxSpeed = 0.8 * kMaxSpeedTheoretical # 2/3rds of max speed, poofs used 0.8

kMaxAngularSpeedTheoretical = kMaxSpeedTheoretical / math.hypot(kTrackWidth / 2, kTrackWidth / 2)
kMaxAngularSpeed = 0.5 * kMaxAngularSpeedTheoretical # rad/s, about 1/3rd of max speed, poofs used 0.5
kShooterTolerance = 100

Preset = Literal["intake"] | Literal["low"] | Literal["mid"] | Literal["high"]

kArmPresets = {
    "intake": math.radians(0),
    "low": math.radians(12),
    "mid": math.radians(27),
    "high": math.radians(90)
}

kArmHeight = wpimath.units.inchesToMeters(10.801276)
kArmReduction = (1 / 60) * (12 / 60)
kArmLength = wpimath.units.inchesToMeters(26.5)


kShooterPresets = {
    "intake": 3000,
    "low": 4000,
    "mid": 3800,
    "high": 1200,
}

kShooterProximityThreshold = 90
kShooterSpeedRange = 100
kShooterTolerance = 100
kShooterOffset = wpimath.geometry.Rotation2d.fromDegrees(30) # from CAD
