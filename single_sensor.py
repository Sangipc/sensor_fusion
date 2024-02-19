import asyncio
from functools import partial
import time
import multiprocessing
from vqf import VQF
import numpy as np
import bleak
import qmt
import ble_sensor
from ble_sensor import BleSensor, PayloadMode, DotData

sensors = []


class Scanner:
    def __init__(self):
        self._client = None

    @staticmethod
    async def scan_and_filter_xsens_dot():
        print("Scanning for")
        async with bleak.BleakScanner() as scanner:
            bledevices = await scanner.discover()

        if not bledevices:
            print("No BLE devices")

        xsens_dot_devices = [
            d for d in bledevices if d.name and "Movella DOT" in d.name
        ]

        if not xsens_dot_devices:
            print("No Xsens Dot Devices")
            return

        numOfDevices = len(xsens_dot_devices)
        print(f"{numOfDevices} of Xsens DOT devices found:")
        for i, d in enumerate(xsens_dot_devices):
            print(f"{i + 1}#: {d.name}, {d.address}")

        return xsens_dot_devices


def run_process(fn, *args):
    "Spawn a process and run the function `fn`"
    process = multiprocessing.Process(target=fn, args=args)
    process.start()
    return process


Ts = 0.01  # sampling rate
N = 2  # number of IMUs


async def read_measurement(queue, device):
    sensor_queue = multiprocessing.Manager().Queue()
    sensor = BleSensor(device.address, sensor_queue)
    sensors.append(sensor)

    print("start to connect:")
    await sensor.connect()
    print("connect finish.")

    await asyncio.sleep(2)  # wait for response
    print("Battery Status")
    batteryInfo = await sensor.getBatteryInfo()
    print(batteryInfo)
    await asyncio.sleep(2)  # wait for response

    print("set sensor output rate, only these values allowed: 1, 4, 10, 12, 15, 20, 30, 60, 120")
    await sensor.setOuputRate(30)

    print("Starting measurement reading")
    print(sensor)
    sensor.payloadType = ble_sensor.PayloadMode.rateQuantities
    sensor.queue = queue
    print("From for loop")
    await sensor.startMeasurement()

    print("\nNotifications enabled. Waiting for data...")
    await asyncio.sleep(0.2)  # heading reset
    print("Start Recording")

    # default is no recording
    sensor.fileName = sensor.create_csvfile()
    sensor.recordFlag = True

    # Run within the timeToRunInMinute time range.
    startTimeSeconds = int(round(time.time()))
    timeToRunInMinute = 0.5
    print(f"run the test for {timeToRunInMinute} minutes")
    while int(round(time.time())) - startTimeSeconds < timeToRunInMinute * 60:
        await asyncio.sleep(0.1)

    # stop measurement
    await sensor.stopMeasurement()
    await asyncio.sleep(0.5)


def always_read_imu(id, sensor_dict, devices):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def read_measurement_wrapper():
        await read_measurement(sensor_dict[id], devices[id])

    loop.run_until_complete(read_measurement_wrapper())


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
    # print("relative_quaternion", relative_quaternion)
    heading, inclination = qmt.headingInclinationAngle(relative_quaternion)
    inclination = np.degrees(inclination)
    return inclination


def always_process_data(sensor_dict):
    t = time.time()
    while True:
        data = {}
        for id in range(N):
            queue_data = sensor_dict.get(id)
            data[id] = queue_data.get()
            # print(data[id])

        quats = process_data(data)
        # print(quats)
        inclination = calculate_inclination(quats[0], quats[1])
        print(inclination)

        compute_time = time.time() - t
        still_wait = Ts - compute_time
        if still_wait <= 0:
            still_wait = 0

        time.sleep(still_wait)
        t = time.time()


def main():
    manager = multiprocessing.Manager()
    sensor_dict = manager.dict()

    devices = asyncio.run(Scanner.scan_and_filter_xsens_dot())
    processes = []
    for id in range(N):
        sensor_dict[id] = manager.Queue()
        process = run_process(always_read_imu, id, sensor_dict, devices)
        processes.append(process)

    process = run_process(always_process_data, sensor_dict)
    processes.append(process)

    for process in processes:
        process.join()


if __name__ == "__main__":
    main()
