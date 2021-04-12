class PathTarget:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class PathTargetList:
    def __init__(self):
        self.targets = []

    def addTarget(self, x, y, theta):
        self.targets.append(PathTarget(x,y, theta))

    def getTarget(self, index):
        return self.targets[index]

    def numTargets(self):
        return len(self.targets)
