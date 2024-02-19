import numpy as np
import qmt
from vqf import VQF

Ts = 0.01  # sampling rate
N = 2  # number of IMUs

vqfs = {id: VQF(0.01) for id in range(N)}


def process_data(data: dict):
    quats = {}
    for id in data:
        vqfs[id].update(*[value for key, value in data[id].items() if isinstance(value, np.ndarray)])
        quats[id] = vqfs[id].getQuat6D()
    return quats


def calculate_inclination(Q1, Q2):
    inv_Q2 = qmt.qinv(Q2)
    relative_quaternion = Q1 * inv_Q2
    heading, inclination = qmt.headingInclinationAngle(relative_quaternion)
    inclination = np.degrees(inclination)
    return inclination
