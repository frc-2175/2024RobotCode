import math
import wpimath.units

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

kShooterProximityThreshold = 90
kShooterSpeedRange = 100

kArmPresets = {
    "intake": 0,
    "low": 17,
    "mid": 60,
    "high": 90
}

kShooterPresets = {
    "intake": 3000,
    "low": 3000,
    "mid": 1200,
    "high": 1200,
}

kArmOffset = 235.6