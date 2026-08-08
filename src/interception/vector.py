import math


class Vector:
    """2D vector in world units (metres, or metres/second for velocities)."""

    __slots__ = ("x", "y")

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    # --- construction ---------------------------------------------------
    @staticmethod
    def from_polar(r, theta):
        return Vector(r * math.cos(theta), r * math.sin(theta))

    def copy(self):
        return Vector(self.x, self.y)

    # --- arithmetic -----------------------------------------------------
    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __eq__(self, other):
        return isinstance(other, Vector) and self.x == other.x and self.y == other.y

    def __repr__(self):
        return f"Vector({self.x:.3f}, {self.y:.3f})"

    # --- queries --------------------------------------------------------
    def pair(self):
        return (self.x, self.y)

    def magnitude(self):
        return math.hypot(self.x, self.y)

    def magnitude_squared(self):
        return self.x * self.x + self.y * self.y

    def angle(self):
        return math.atan2(self.y, self.x)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def dist_to(self, other):
        return (self - other).magnitude()

    # --- transforms (all return new vectors) ----------------------------
    def update(self, x=None, y=None):
        if x is not None:
            self.x = float(x)
        if y is not None:
            self.y = float(y)

    def truncate(self, max_value):
        """Clamp magnitude to ``max_value``, preserving direction."""
        mag = self.magnitude()
        if mag > max_value:
            return self * (max_value / mag)
        return self

    def set_magnitude(self, magnitude):
        mag = self.magnitude()
        if mag == 0.0:
            return Vector(0.0, 0.0)
        return self * (magnitude / mag)

    def normalize(self):
        return self.set_magnitude(1.0)

    def perpendicular(self):
        return Vector(-self.y, self.x)

    def rotate(self, angle):
        cos_t = math.cos(angle)
        sin_t = math.sin(angle)
        return Vector(
            self.x * cos_t - self.y * sin_t,
            self.x * sin_t + self.y * cos_t,
        )
