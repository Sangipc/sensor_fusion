import numpy as np
import pandas as pd
DELTA_T = 0.1
DATASET_NUM = 50
DATA_NUM = 9

params_axis = np.ones((4, 1))
params_pos = np.ones((6, 1))
j1, j2 = np.zeros(3), np.zeros(3)
o1, o2 = np.zeros(3), np.zeros(3)
g1, g2 = np.zeros(3), np.zeros(3)
g_dot1, g_dot2 = np.zeros(3), np.zeros(3)
a1, a2 = np.zeros(3), np.zeros(3)
imu_raw_data_1, imu_raw_data_2 = None, None
imu_raw_data_online1, imu_raw_data_online2 = None, None
prev_angle_gyr, prev_angle_acc_gyr = 0.0, 0.0
def get_raw_data():
    global imu_raw_data_1, imu_raw_data_2
    """
    Get data from two IMUs, extracting angular velocities.
    """
    imu_filename_1 = "data1.csv"
    imu_filename_2 = "data2.csv"

    imu_raw_data_1 = get_data(imu_filename_1, DATASET_NUM)
    imu_raw_data_2 = get_data(imu_filename_2, DATASET_NUM)
    # print("Data 1")
    # for i in range(50):
    #     for j in range(9):
    #         print(imu_raw_data_1[i][j], end=" ")
    #     print()
    # print("Data 2")
    # for i in range(50):
    #     for j in range(9):
    #         print(imu_raw_data_2[i][j], end=" ")
    #     print()


def get_data(filename, NUM):
    acc = []
    vel = []
    vel_dot = []
    raw_data = []
    path = "./data_logging/"
    path_filename = path + filename
    with open(path_filename, 'r') as file:
        lines = file.readlines()
        lines = lines[1:]

        for line in lines[:NUM]:
            values = line.split(',')
            values = [float(val) for val in values]

            time, acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, angle_x, angle_y, angle_z= values

            acc.append([acc_x, acc_y, acc_z])
            vel.append([vel_x, vel_y, vel_z])

            # Calculate vel_dot
            vel_dot_i = []
            for j in range(3):
                if len(vel) > 2:
                    vel_dot_val = (vel[-3][j] - 8 * vel[-2][j] + 8 * vel[-1][j] - vel[-0][j]) / 12 * DELTA_T
                else:
                    vel_dot_val = (8 * vel[-1][j] - vel[-0][j]) / 12 * DELTA_T
                vel_dot_i.append(vel_dot_val)
            vel_dot.append(vel_dot_i)
            row_data = []
            row_data.extend(acc[-1])
            row_data.extend(vel[-1])
            row_data.extend(vel_dot[-1])
            raw_data.append(row_data)
    return raw_data
def get_pos(input_data, params, output):
    """
    Calculate joint positions relative to two IMUs.
    """

    # Define 6 parameters to be solved
    o1x, o1y, o1z, o2x, o2y, o2z = params

    for i in range(input_data.shape[0]):
        # Angular acceleration calculation values
        acc_joint1_x = (
            input_data[i, 4] * (input_data[i, 3] * o1y - input_data[i, 4] * o1x)
            - input_data[i, 5] * (input_data[i, 5] * o1x - input_data[i, 3] * o1z)
            + (input_data[i, 7] * o1z - input_data[i, 8] * o1y)
        )
        acc_joint1_y = (
            input_data[i, 5] * (input_data[i, 4] * o1z - input_data[i, 5] * o1y)
            - input_data[i, 3] * (input_data[i, 3] * o1y - input_data[i, 4] * o1x)
            + (input_data[i, 8] * o1x - input_data[i, 6] * o1z)
        )
        acc_joint1_z = (
            input_data[i, 3] * (input_data[i, 5] * o1x - input_data[i, 3] * o1z)
            - input_data[i, 4] * (input_data[i, 4] * o1z - input_data[i, 5] * o1y)
            + (input_data[i, 6] * o1y - input_data[i, 7] * o1x)
        )

        acc_joint2_x = (
            input_data[i, 13] * (input_data[i, 12] * o2y - input_data[i, 13] * o2x)
            - input_data[i, 14] * (input_data[i, 14] * o2x - input_data[i, 12] * o2z)
            + (input_data[i, 16] * o2z - input_data[i, 17] * o2y)
        )
        acc_joint2_y = (
            input_data[i, 14] * (input_data[i, 13] * o2z - input_data[i, 14] * o2y)
            - input_data[i, 12] * (input_data[i, 12] * o2y - input_data[i, 13] * o2x)
            + (input_data[i, 17] * o2x - input_data[i, 15] * o2z)
        )
        acc_joint2_z = (
            input_data[i, 12] * (input_data[i, 14] * o2x - input_data[i, 12] * o2z)
            - input_data[i, 13] * (input_data[i, 13] * o2z - input_data[i, 14] * o2y)
            + (input_data[i, 15] * o2y - input_data[i, 16] * o2x)
        )

        # Objective function
        output[i, 0] = np.sqrt(
            (input_data[i, 0] - acc_joint1_x) ** 2
            + (input_data[i, 1] - acc_joint1_y) ** 2
            + (input_data[i, 2] - acc_joint1_z) ** 2
        ) - np.sqrt(
            (input_data[i, 9] - acc_joint2_x) ** 2
            + (input_data[i, 10] - acc_joint2_y) ** 2
            + (input_data[i, 11] - acc_joint2_z) ** 2
        )

def get_axis(input_data, params, output):
    """
    Calculate joint directions relative to two IMUs.
    """

    # Extracting parameters
    theta_1, theta_2, phi_1, phi_2 = params

    for i in range(input_data.shape[0]):
        # Objective model
        output[i, 0] = np.sqrt(
            (
                (input_data[i, 1] * np.sin(theta_1) - input_data[i, 2] * np.cos(phi_1) * np.sin(theta_1)) ** 2
            )
            + (
                (input_data[i, 2] * np.cos(phi_1) * np.cos(theta_1) - input_data[i, 0] * np.sin(phi_1)) ** 2
            )
            + (
                (input_data[i, 0] * np.cos(phi_1) * np.sin(theta_1) - input_data[i, 1] * np.cos(phi_1) * np.cos(theta_1)) ** 2
            )
        ) - np.sqrt(
            (
                (input_data[i, 4] * np.sin(theta_2) - input_data[i, 5] * np.cos(phi_2) * np.sin(theta_2)) ** 2
            )
            + (
                (input_data[i, 5] * np.cos(phi_2) * np.cos(theta_2) - input_data[i, 3] * np.sin(phi_2)) ** 2
            )
            + (
                (input_data[i, 3] * np.cos(phi_2) * np.sin(theta_2) - input_data[i, 4] * np.cos(phi_2) * np.cos(theta_2)) ** 2
            )
        )


ITER_STEP = 1e-6  # Define the iteration step

def get_jacobian(func, input_data, params, output):
    """
    Calculate the Jacobian matrix for Gauss-Newton method iteration.
    """
    m = input_data.shape[0]  # Number of data points
    n = len(params)  # Number of unknown parameters

    out0 = np.zeros((m, 1))
    out1 = np.zeros((m, 1))
    param0 = np.zeros((n, 1))
    param1 = np.zeros((n, 1))

    for j in range(n):
        param0 = params.reshape(-1, 1)
        param1 = params.reshape(-1, 1)
        param0[j] -= ITER_STEP
        param1[j] += ITER_STEP
        func(input_data, param0.flatten(), out0)
        func(input_data, param1.flatten(), out1)

        output[:, j:j+1] = (out1 - out0) / (2 * ITER_STEP)


ITER_CNT = 100  # Define the iteration count

def gauss_newton(func, input_data, output, params):
    """
    Gauss-Newton method implementation.
    """
    m = input_data.shape[0]  # Number of data points
    n = len(params)  # Number of parameters

    # Initialize matrices and vectors
    jmat = np.zeros((m, n))
    r = np.zeros((m, 1))
    tmp = np.zeros((m, 1))

    pre_mse = 0.0
    mse = 0.0

    for i in range(ITER_CNT):
        mse = 0.0
        func(input_data, params, tmp)
        r = output.reshape(-1, 1) - tmp
        get_jacobian(func, input_data, params, jmat)

        # Mean squared error
        mse = np.transpose(r) @ r
        mse /= m

        if np.abs(mse - pre_mse) < 1e-8:
            break
        pre_mse = mse

        # Regularization parameter
        lambda_reg = 0.1

        # Compute delta with regularization
        delta = np.linalg.inv(np.transpose(jmat) @ jmat + lambda_reg * np.identity(jmat.shape[1])) @ np.transpose(
            jmat) @ r
        print(f"i = {i}, mse = {mse}")
        params += np.squeeze(delta)

    print("params:", params)

def imu_joint_pos_data_fit():
    global imu_raw_data_1,imu_raw_data_2
    input_data = np.zeros((DATASET_NUM, 18))
    output_data = np.zeros((DATASET_NUM, 1))

    for i in range(DATASET_NUM):
        k = 0
        for j in range(9):
            input_data[i, k] = imu_raw_data_1[i][j]
            k += 1
        for j in range(9):
            input_data[i, k] = imu_raw_data_2[i][j]
            k += 1
        output_data[i, 0] = 0

    params_pos = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    gauss_newton(get_pos, input_data, output_data, params_pos)

    o1 = params_pos[:3]
    o2 = params_pos[3:]
    print("o1:", o1)
    print("o2:", o2)

    return 0


def imu_joint_axis_data_fit():
    print("Inside imu_joint_axis_data_fit")
    input_data = np.zeros((DATASET_NUM, 6))
    output_data = np.zeros((DATASET_NUM, 1))

    for i in range(DATASET_NUM):
        k = 0
        for j in range(3, 6):
            input_data[i, k] = imu_raw_data_1[i][j]
            k += 1
        for j in range(3, 6):
            input_data[i, k] = imu_raw_data_2[i][j]
            k += 1
        output_data[i, 0] = 0

    params_axis = np.array([0.5, 0.5, 0.5, 0.5])
    gauss_newton(get_axis, input_data, output_data, params_axis)

    j1 = np.array([np.cos(params_axis[2]) * np.cos(params_axis[0]),
                   np.cos(params_axis[2]) * np.sin(params_axis[0]),
                   np.sin(params_axis[2])])

    j2 = np.array([np.cos(params_axis[3]) * np.cos(params_axis[1]),
                   np.cos(params_axis[3]) * np.sin(params_axis[1]),
                   np.sin(params_axis[3])])

    return 0

def get_angle_acc(j1, j2, a1, a2, g1, g2, g_dot1, g_dot2, o1, o2):
    c = np.array([1, 0, 0])
    x1 = np.cross(j1, c)
    y1 = np.cross(j1, x1)
    x2 = np.cross(j2, c)
    y2 = np.cross(j2, x2)

    o1 = o1 - np.dot(o1, j1) * j1 / 2 - np.dot(o2, j2) * j1 / 2
    o2 = o2 - np.dot(o1, j1) * j2 / 2 - np.dot(o2, j2) * j2 / 2

    a1_dot = a1 - (np.cross(g1, np.cross(g1, o1)) + np.cross(g_dot1, o1))
    a2_dot = a2 - (np.cross(g2, np.cross(g2, o2)) + np.cross(g_dot2, o2))

    p1 = np.dot(a1_dot, x1)
    p2 = np.dot(a1_dot, y1)
    q1 = np.dot(a2_dot, x2)
    q2 = np.dot(a2_dot, y2)

    acc1 = np.array([p1, p2])
    acc2 = np.array([q1, q2])

    angle_acc = np.arccos(np.dot(acc1, acc2) / (np.linalg.norm(acc1) * np.linalg.norm(acc2)))

    return angle_acc

def test_angle():
    angle_acc = angle_gyr = angle_acc_gyr = 0.0
    sum_val = 0.0
    cnt = 0
    lambda_val = 0.01

    # Assuming imu_raw_data_online1 and imu_raw_data_online2 are pandas DataFrames from CSV files
    imu_raw_data_online1 = pd.read_csv("data_logging/data1.csv").values
    imu_raw_data_online2 = pd.read_csv("data_logging/data2.csv").values

    fp = open("data.txt", "w")
    if fp is None:
        print("File cannot open")
        exit(0)

    prev_angle_acc_gyr = prev_angle_gyr = 0.0

    for i in range(500):
        cnt += 1

        a1 = imu_raw_data_online1[i, 0:3]
        a2 = imu_raw_data_online2[i, 0:3]
        g1 = imu_raw_data_online1[i, 3:6]
        g2 = imu_raw_data_online2[i, 3:6]
        g_dot1 = imu_raw_data_online1[i, 6:9]
        g_dot2 = imu_raw_data_online2[i, 6:9]

        # Assuming j1, j2, o1, o2 are defined somewhere
        angle_acc = get_angle_acc(j1, j2, a1, a2, g1, g2, g_dot1, g_dot2, o1, o2)
        sum_val += np.dot(g1, j1) - np.dot(g2, j2)

        if cnt > 3:
            DELTA_T = 1  # Replace this with the actual value of DELTA_T
            angle_gyr = sum_val * DELTA_T

            angle_acc_gyr = lambda_val * angle_acc + (1 - lambda_val) * (prev_angle_acc_gyr + angle_gyr - prev_angle_gyr)
            print("angle:", angle_acc_gyr)
            cnt = 0

            fp.write(f"{angle_acc_gyr}\n")

        prev_angle_acc_gyr = angle_acc_gyr
        prev_angle_gyr = angle_gyr

    fp.close()
def main():
    get_raw_data()
    imu_joint_axis_data_fit()
    imu_joint_pos_data_fit()

    # Refers to an arbitrary point along the joint axis
    # Shift it as close as possible to the sensors by applying:
    global o1, o2, j1, j2  # Assuming o1, o2, j1, j2 are defined globally
    o1 = o1 - j1 * (np.dot(o1, j1) + np.dot(o2, j2)) / 2
    o2 = o2 - j2 * (np.dot(o1, j1) + np.dot(o2, j2)) / 2

    test_angle()

    return 0

if __name__ == "__main__":
    main()
