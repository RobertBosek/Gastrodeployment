


class DeploymentControl:

    def __init__(self):
        self.mode = "standby"
        print("[DeploymentControl]: Initialized")
        self.loop()

    def loop(self):
        while True:
            if self.mode == "standby":
                self.shutdownBackend()
                self.playStandbyVideo()
            elif self.mode == "alert":
                self.showInfo()
                self.startupBackend()

    def showInfo(self):
        print("stop Video")
        print("show Infoscreen")

    def playStandbyVideo(self):
        print("PlayingEyecatcher Video")

    def startupBackend(self):
        print("exec launchfile")

    def shutdownBackend(self):
        print("shutting down backend ROS nodes")