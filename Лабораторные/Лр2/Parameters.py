
class Parameters:

    # Robot motion parameters
    x = 0
    y = 0
    heading = 0
    velocity = 0
    angular_rate = 0


    def __init__(self, x, y, heading, velocity, angular_rate):
        self.x = x
        self.y = y
        self.heading = heading
        self.velocity = velocity
        self.angular_rate = angular_rate
