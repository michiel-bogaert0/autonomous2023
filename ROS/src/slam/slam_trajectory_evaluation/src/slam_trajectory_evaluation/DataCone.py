from geometry_msgs.msg import Point


class DataCone:
    pos = Point(0, 0, 0)

    def __init__(self, amount):
        self.maxamount = amount
        self.classType = -1
        self.min = 10000
        self.max = 0
        self.distances = []
        self.posEval = []

    def AddItem(self, distance: float, pos: Point):
        if distance < self.min:
            self.min = distance
        elif distance > self.max:
            self.max = distance

        if len(self.distances) < self.maxamount:
            self.distances.append(distance)
            self.posEval.append(pos)

    def CheckPos(self, pos: Point) -> bool:
        for p in self.posEval:
            if abs(p.x - pos.x) < 0.01 and abs(p.y - pos.y) < 0.01:
                return True
        return False
