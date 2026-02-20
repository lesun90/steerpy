# SteerPy author, 2026.

class Sensors:
    """
    LiDAR + convenience distances (meters):
      front, rear, left, right, front_left, front_right
      lidar: list[float], N beams in car frame (N is runtime-configurable),
             index 0 is front and indices increase CCW, -1 means no hit in range
    """

    def __init__(self):
        self.front = -1.0
        self.rear = -1.0
        self.left = -1.0
        self.right = -1.0
        self.front_left = -1.0
        self.front_right = -1.0
        self.lidar = []
