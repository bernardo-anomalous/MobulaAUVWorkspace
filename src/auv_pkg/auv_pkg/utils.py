import math


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """Convert quaternion components to Euler angles in degrees.

    Parameters
    ----------
    x, y, z, w : float
        Quaternion components in the ``xyzw`` convention.

    Returns
    -------
    tuple
        The roll, pitch and yaw angles in degrees.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = -math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
