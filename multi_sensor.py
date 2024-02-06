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

    async def scan_and_filter_xsens_dot():
        print("Scanning for")
        async with bleak.BleakScanner() as scanner:
            bledevices = await scanner.discover()

        if not bledevices:
            print("No BLE devices")

        xsens_dot_devices = []
        for i, d in enumerate(bledevices):
            if d.name and "Movella DOT" in d.name:
                xsens_dot_devices.append(d)

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

async def read_measurement(queue):
    devices = await Scanner.scan_and_filter_xsens_dot()

    if not devices:
        print("No Xsens DOT, considering turn them on or shake them from sleep mode")
        return

    for d in devices:
        with multiprocessing.Manager() as manager:
            sensor_queue = manager.Queue()
            sensor = BleSensor(d.address, sensor_queue)
            sensors.append(sensor)
    # print("Sensors from start_sensor",sensors)
    print("start to connect:")
    for s in sensors:
        await s.connect()
    print("connect finish.")

    await asyncio.sleep(2)  # wait for response
    print("Battery Status")
    for i in range(len(sensors)):
        batteryInfo = await sensors[i].getBatteryInfo()
        print(batteryInfo)
    await asyncio.sleep(2)  # wait for response

    print("set sensor output rate, only these values allowed: 1, 4, 10, 12, 15, 20, 30, 60, 120")
    for s in sensors:
        await s.setOuputRate(30)
    print("Starting measurement reading")
    print(sensors)
    for s in sensors:
        s.payloadType = ble_sensor.PayloadMode.rateQuantities
        s.queue = queue
        print("From for loop")
        await s.startMeasurement()

    print("\nNotifications enabled. Waiting for data...")
    # heading reset
    await asyncio.sleep(0.2)  # wait for response
    print("Start Recording")
    for s in sensors:
        # default is no recording
        s.fileName = s.create_csvfile()
        s.recordFlag = True

    # Run within the timeToRunInMinute time range.
    startTimeSeconds = int(round(time.time()))
    timeToRunInMinute = 0.5
    print(f"run the test for {timeToRunInMinute} minutes")
    while int(round(time.time())) - startTimeSeconds < timeToRunInMinute * 60:
        await asyncio.sleep(0.1)

    # stop measurement
    for s in sensors:
        await s.stopMeasurement()
    await asyncio.sleep(0.5)

def always_read_imu(id, sensor_dict) -> None:
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def read_measurement_wrapper():
        await read_measurement(sensor_dict[id])

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
    #print("relative_quaternion", relative_quaternion)
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
            #print(data[id])

        quats = process_data(data)
        #print(quats)
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

    processes = []
    for id in range(N):
        sensor_dict[id] = manager.Queue()
    for id in range(N):
        process = run_process(always_read_imu, id,sensor_dict)
        processes.append(process)
    #print("This is the queue", sensor_dict)
    process = run_process(always_process_data, sensor_dict)
    processes.append(process)

    for process in processes:
        process.join()

if __name__ == "__main__":

    main()