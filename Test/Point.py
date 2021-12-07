class Point:
    def __init__(self, x, y, is_covered=False, is_obstacle=False):
        self.x = x
        self.y = y
        self.is_covered = is_covered
        self.is_obstacle = is_obstacle

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_is_covered(self):
        return self.is_covered

    def get_is_obstacle(self):
        return self.is_obstacle

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_is_covered(self, is_covered):
        self.is_covered = is_covered

    def set_is_obstacle(self, is_obstacle):
        self.is_obstacle = is_obstacle

    def __repr__(self):
        return "Point(x, y)"

    def __str__(self):
        return "X= " + str(self.x) + " Y= " + str(self.y) + " IsCovered= " + str(self.is_covered) + " IsObstacle= " + str(self.is_obstacle)
