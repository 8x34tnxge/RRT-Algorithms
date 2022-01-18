import math
import random

def sampleUnitBall(num: int, dim=3):
    assert num > 0
    assert isinstance(num, int)

    samples = []

    for _ in range(num):
        if dim == 2:
            sample = get2D_sample()
        elif dim == 3:
            sample = get3D_sample()
        else:
            raise ValueError("Parameter \{dim\} only support 2 or 3")

        samples.append(sample)

    return samples

def get2D_sample():
    # let's set r, theta
    # 0 < r <= 1, 0 < theta < 2pi
    r = random.random()
    theta = random.random() * 2 * math.pi

    # then, x = r * cos(theta)
    # y = r * sin(theta)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    
    sample = (x, y)
    return sample

def get3D_sample():
    # let's set r, theta, phi
    # 0 < r <= 1, 0 < theta < 2pi, 0 < phi < pi
    r = random.random()
    theta = random.random() * 2 * math.pi
    phi = random.random() * math.pi

    # then, x = r * cos(theta) * sin(phi)
    # y = r * sin(theta) * sin(phi)
    # z = r * cos(phi)
    x = r * math.cos(theta) * math.sin(phi)
    y= r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(phi)

    sample = (x, y, z)
    return sample