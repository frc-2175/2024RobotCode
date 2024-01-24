import math
import wpimath.units

kWheelDiameter = wpimath.units.inchesToMeters(3)
kTrackWidth = wpimath.units.inchesToMeters(24.5)
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

kDriveMotorReduction = (45 * 22) / (13 * 15)
kDriveMotorFreeSpeed = 5676 / 60 # 94.6 rev/s

kMaxSpeedTheoretical = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / kDriveMotorReduction # 4.46 m/s
kMaxSpeed = 2 / 3 * kMaxSpeedTheoretical # 2/3rds of max speed, poofs used 0.8

kMaxAngularSpeedTheoretical = kMaxSpeedTheoretical / math.hypot(kTrackWidth / 2, kTrackWidth / 2)
kMaxAngularSpeed = math.pi # rad/s, about 1/3rd of max speed, poofs used 0.5