import rev
import math

class Arm:
    def __init__(self, angleMotorId) -> None:
        self.angleMotor = rev.CANSparkMax(angleMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        # TODO: this is not how you set idle mode
        # self.angleMotor.IdleMode(1)
        self.angleMotor.setInverted(True)

        self.angleEncoder = self.angleMotor.getEncoder()
        self.anglePIDController = self.angleMotor.getPIDController()

        #self.angleEncoder.setPositionConversionFactor(1 / 60 /  5 * 360)
        
        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kForward, self.rotFromDegrees(90)) 

        self.angleMotor.enableSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, True)
        self.angleMotor.setSoftLimit(rev.CANSparkBase.SoftLimitDirection.kReverse, self.rotFromDegrees(2)) 

        self.anglePIDController.setFF(0)
        self.anglePIDController.setP(1/30)
        self.anglePIDController.setI(0)
        self.anglePIDController.setD(0)

    
    def getArmSpeed(self):
        return self.angleMotor.get()

    def getArmAngle(self):
        return self.rotToDegrees(self.angleEncoder.getPosition())
    
    def setArmAngleDegrees(self, angle):
        self.anglePIDController.setReference(self.rotFromDegrees(angle), rev.CANSparkMax.ControlType.kPosition)

    def rotToDegrees(self, angle): 
        return angle / 60 / 5 * 360
    
    def rotFromDegrees(self, angle): 
        return angle * 60 * 5 / 360
    
    def periodic(self):
        self.anglePIDController.setFF(0)
    
    # def setArmAngle(self, angle):   
        