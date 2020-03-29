class MotorState:
    def __init__(self):
        self.position = 0
        self.prev_position = 0
        self.velocity = 0
        self.prev_velocity = 0
        self.acceleration = 0

    def update(self, position, dt):
        self.position = position
        self.velocity = (self.position - self.prev_position) / dt
        self.acceleration = (self.velocity - self.prev_velocity) / dt
        self.prev_velocity = self.velocity
        self.prev_position = self.position

    def putNT(self, nt, name):
        nt.putNumber(f"{name}_position", self.position)
        nt.putNumber(f"{name}_velocity", self.velocity)
        nt.putNumber(f"{name}_acceleration", self.acceleration)

