import asyncio

import numpy as np
import qmt
import time
from vqf import VQF
import matplotlib.pyplot as plt
Ts = 0.016666  # sampling rate
N = 2  # number of IMUs

vqfs = {id: VQF(Ts) for id in range(N)}
calls_per_second = 0
start_time = time.time()
inclination_angles = []
timestamps = []
def process_data(data: dict):
    quats = {}
    for id in data:
        vqfs[id].update(*[value for key, value in data[id].items() if isinstance(value, np.ndarray)])
        quats[id] = vqfs[id].getQuat6D()
    return quats


def calculate_inclination(Q1, Q2):
    inv_Q1 = qmt.qinv(Q1)
    relative_quaternion = Q2 * inv_Q1
    heading, inclination = qmt.headingInclinationAngle(relative_quaternion)
    inclination = np.degrees(inclination)
    return inclination

def always_process_data(sensor_dict):
    global calls_per_second
    global start_time
    t = time.time()
    while True:
        data = {}
        for id in range(N):
            queue_data = sensor_dict.get(id)
            data[id] = queue_data.get()
            #print(data[id])
        # sample data[id]
        # {'gyro': array([ 1.92319047, -1.38425756, -0.63783944]), 'acc': array([-0.02813759, -0.08818505,  9.81900501])}
        # {'gyro': array([ 0.21070607, -0.36701715, -0.39044639]), 'acc': array([-0.12048834, -0.07399426,  9.43940926])}
        quats = process_data(data)  #this gives me 2 quaternions one for each sensor
        #print(quats)    #{0: array([-0.23207212, -0.12451914, -0.50840576, -0.81985431]), 1: array([ 0.40295376, -0.76601632,  0.0574779 , -0.49753749])}
        inclination = calculate_inclination(quats[0], quats[1])
        #print(inclination)
        compute_time = time.time() - t
        still_wait = Ts - compute_time
        if still_wait <= 0:
            still_wait = 0

        time.sleep(still_wait)
        t = time.time()

        # Update call counter
        calls_per_second += 1
        current_time = time.time()
        if current_time - start_time >= 1:  # Check if 1 second has passed
            print("Calls per second:", calls_per_second)
            start_time = current_time
            calls_per_second = 0
