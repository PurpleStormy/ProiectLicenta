class Point:
    def __init__(self, x, y, degrees, rgb, is_covered=False):
        self.x = x
        self.y = y
        self.degrees = degrees
        self.rgb = rgb
        self.is_covered = is_covered

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_degrees(self):
        return self.degrees

    def get_rgb(self):
        return self.rgb

    def get_is_covered(self):
        return self.is_covered

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_degrees(self, degrees):
        self.degrees = degrees

    def set_rgb(self, rgb):
        self.rgb = rgb

    def set_is_covered(self, is_covered):
        self.is_covered = is_covered

    def __repr__(self):
        return "X= " + str(self.x) + " Y= " + str(self.y) + " Degrees= " + str(self.degrees) + " IsCovered= " + str(self.is_covered) + " RGB = " \
               + str(self.rgb) + "\n"

    def __str__(self):
        return "X= " + str(self.x) + " Y= " + str(self.y) + " Degrees= " + str(self.degrees) + " IsCovered= " + str(self.is_covered) + " RGB = " \
               + str(self.rgb) + "\n"
