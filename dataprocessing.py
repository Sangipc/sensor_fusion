import asyncio

import numpy as np
import qmt
import time
from vqf import VQF
import matplotlib.pyplot as plt
Ts = 0.016666  # sampling rate
N = 2  # number of IMUs

vqfs = {id: VQF(Ts) for id in range(N)}
for vqf in vqfs.values():
    vqf.setTauAcc(1e-6)  # Set a very small value for accelerometer time constant
#     vqf.setTauMag(1.0)
# print("Values of vqfs dictionary:")
# for id, vqf in vqfs.items():
#     print(f"VQF Instance {id}:")
#     print("Parameters:", vqf.params)
#     print("Coefficients:", vqf.coeffs)
#     print("State:", vqf.state)
#     print()
calls_per_second = 0
start_time = time.time()

def process_data(data: dict):
    quats = {}
    for id in data:
        gyr = data[id]['gyro']
        acc = data[id]['acc']
        mag = data[id]['mag']
        vqfs[id].update(gyr, acc, mag)
        #vqfs[id].update(*[value for key, value in data[id].items() if isinstance(value, np.ndarray)])
        # print("Values of vqfs dictionary:")
        # # for id, vqf in vqfs.items():
        # print(f"VQF Instance {id}:")
        # print("Parameters:", vqfs[id].params)
        # print()
        # print("Coefficients:", vqfs[id].coeffs)
        # print()
        # print("State:", vqfs[id].state)
        # print()

        quats[id] = vqfs[id].getQuat6D()
    return quats

def calculate_inclination(Q1, Q2):
    #inv_Q2 = qmt.qinv(Q2)
    dictt = qmt.quatProject(Q1, [0, 0, 1])
    projAngle, resAngle, projQuat, resQuat_Q1 = dictt['projAngle'], dictt['resAngle'], dictt['projQuat'], dictt['resQuat']

    #print(resQuat_Q1)
    dictt = qmt.quatProject(Q2, [0, 0, 1])
    projAngle, resAngle, projQuat, resQuat_Q2 = dictt['projAngle'], dictt['resAngle'], dictt['projQuat'], dictt[
        'resQuat']

    #print(resQuat_Q2)

    #q2_heading, q2_incl = qmt.quatProject(Q2, [0, 0, 1])
    #print(Q1)
    #print(dictt)

    relative_quaternions21 = qmt.qmult(resQuat_Q1,qmt.qinv(resQuat_Q2))
    relative_quaternions12 = qmt.qmult(qmt.qinv(resQuat_Q2), resQuat_Q1)
    heading_1, inclination_1 = qmt.headingInclinationAngle(resQuat_Q1)
    heading_2, inclination_2 = qmt.headingInclinationAngle(resQuat_Q2)
    heading_3, inclination_21 = qmt.headingInclinationAngle(relative_quaternions21)
    #heading_3, inclination_12 = qmt.headingInclinationAngle(relative_quaternions12)
    inclination_1 = np.degrees(inclination_1)
    inclination_2 = np.degrees(inclination_2)
    inclination_21 = np.degrees(inclination_21)
    #inclination_12 = np.degrees(inclination_12)
    return inclination_1, inclination_2, inclination_21

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
        inclination_1, inclination_2, inclination_21= calculate_inclination(quats[0], quats[1])
        #print("Sensor 1",inclination_1)
        #print("Sensor 2", inclination_2)
        #print("inclination21", inclination_21)
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
