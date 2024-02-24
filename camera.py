from cscore import CameraServer

def main():
    CameraServer.enableLogging()

    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(640, 480)
    camera.setExposureManual(1)
    camera.setFPS(60)

    CameraServer.putVideo("rect", 640, 480)

    while True:
        CameraServer.waitForever()