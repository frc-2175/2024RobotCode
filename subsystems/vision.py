import dataclasses
import math
import wpimath.units
import wpimath.geometry
from wpimath.geometry import Pose3d, Pose2d, Transform2d, Transform3d, Rotation3d, Quaternion, CoordinateSystem
from wpimath.units import inchesToMeters
import ntcore
import wpiutil.wpistruct
import robotpy_apriltag
import numpy as np

@wpiutil.wpistruct.make_wpistruct(name="tagdetection")
@dataclasses.dataclass
class TagDetection:
    id: int
    transform: Transform3d

field = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2024Crescendo)

class Vision:
    def __init__(self) -> None:


        inst = ntcore.NetworkTableInstance.getDefault();

        self.subscriber = inst.getStructArrayTopic("/jetson/detections", TagDetection).subscribe([])

        self.publisher = inst.getStructTopic("robot3d", Pose3d).publish()
        self.tag3d = inst.getStructTopic("tag3d", Pose3d).publish()
        self.publisher2 = inst.getStructTopic("foobar", Pose3d).publish()

        self.cameraPose = Transform3d(wpimath.geometry.Translation3d(inchesToMeters(-5), inchesToMeters(-12), inchesToMeters(10.15)), wpimath.geometry.Rotation3d(0, math.radians(-30), math.radians(180)))
    
    def update(self) -> list[tuple[Pose2d, float]]:
        values = self.subscriber.readQueue()

        poses: list[tuple[Pose2d, float]] = []

        for value in values:
            time = value.time / 1e6
            detection: TagDetection
            for detection in value.value:
                tagPose = field.getTagPose(detection.id)
                if not tagPose: continue
                
                
                detectionNWU = CoordinateSystem.convert(detection.transform, CoordinateSystem.EDN(), CoordinateSystem.NWU())
                nwuInverted = Transform3d(detectionNWU.translation(), Rotation3d(0, 0, math.pi) + detectionNWU.rotation())
                robotPose: Pose3d = tagPose.transformBy(nwuInverted.inverse()).transformBy(self.cameraPose.inverse())
                self.publisher.set(robotPose)
                self.tag3d.set(Pose3d(detectionNWU.translation(), Rotation3d(0, 0, math.pi) + detectionNWU.rotation()))
                self.publisher2.set(Pose3d(
                    detectionNWU.translation(),
                    detectionNWU.rotation(),
                ))
                poses.append((robotPose.toPose2d(), time))
            
        return poses