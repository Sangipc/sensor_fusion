import numpy as np
import asyncio
import bleak
import time
import sys
import qmt
#from qmt import jointAxisEstHingeOlsson
import ble_sensor
from ble_sensor import BleSensor
from sync_manager import SyncManager
class Scanner:
    def __init__(self):
        self._client = None

    async def scan_and_filter_xsens_dot():
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

async def read_sensor(queue):
    devices = await Scanner.scan_and_filter_xsens_dot()

    if not devices:
        print("No Xsens DOT, considering turn them on or shake them from sleep mode")
        return
    queue = queue
    sensors = []
    for d in devices:
        sensor = BleSensor(d.address)
        sensor.queue = queue
        sensors.append(sensor)
    root_sensor = sensors[0]
    #print(root_sensor)
    #print(root_sensor.address)

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
    syncManager = SyncManager(sensors)
    for s in sensors:
        s.syncManager = syncManager
    # await syncManager.startSyncing()

    for s in sensors:
        s.payloadType = ble_sensor.PayloadMode.rateQuantities
        await s.startMeasurement(root_sensor)
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

    # print("Power Off Sensors")
    # for s in sensors:
    #     await s.poweroffSensor()
    # await asyncio.sleep(2)  # wait for response

    print("exit program")
    await sys.exit(0)

# async def process_data(queue):
#     count = 0
#     # acc1 = np.empty((0, 3))
#     # gyro1 = np.empty((0,3))
#     # acc2 = np.empty((0, 3))
#     # gyro2 = np.empty((0,3))
#     while True:
#         sensor_data = await queue.get()
#         sens1 , sens2 = sensor_data
#         # print("sensor data", sensor_data)
#         # print("sens1 ", sens1)
#         # print("sens2 ", sens2)
#         if count == 0:
#             acc1 = np.array(sens1)
#             gyro1 = np.array(sens2)
#             # print(f"ACC1: {acc1}, gyro1: {gyro1}")
#         elif count == 1:
#             acc2 = np.array(sens1)
#             gyro2 = np.array(sens2)
#             # print(f"ACC2: {acc1}, gyro2: {gyro1}")
#         else:
#             if count % 6 == 0 and count != 0:  # Check for every 6th count
#                 # print("Counting inside 6: ", count)
#                 # print(np.max(acc1.shape))
#                 # print(np.max(acc2.shape))
#                 # print("shape", acc1.shape)
#                 # print("shape", acc2.shape)
#                 if acc1.shape[0] > 0 and acc2.shape[0] > 0:
#                     jhat1, jhat2 = qmt.jointAxisEstHingeOlsson(acc1, acc2, gyro1, gyro2)
#                     # Normalize the vectors to ensure they are unit vectors
#                     jhat1_normalized = jhat1 / np.linalg.norm(jhat1)
#                     jhat2_normalized = jhat2 / np.linalg.norm(jhat2)
#
#                     # Calculate the dot product of the normalized vectors
#                     dot_product = np.dot(jhat1_normalized.flatten(), jhat2_normalized.flatten())
#
#                     # Calculate the angle in radians using arccosine
#                     angle_radians = np.arccos(dot_product)
#
#                     # Convert the angle to degrees
#                     angle_degrees = np.degrees(angle_radians)
#
#                     print(f"The angle between the vectors is approximately: {angle_degrees} degrees")
#                     # Clear arrays for next calculation
#                     acc1 = np.empty((0, 3))
#                     gyro1 = np.empty((0, 3))
#                     acc2 = np.empty((0, 3))
#                     gyro2 = np.empty((0, 3))
#             if count % 2 == 0:
#
#                 acc1 = np.vstack((acc1, np.array(sens1)))
#                 gyro1 = np.vstack((gyro1, np.array(sens2)))
#                 # print(f"Successfully taken acc1, gyro1 for {count} iteration ")
#                 # print(f"ACC1: {acc1}, gyro1: {gyro1}")
#             else:
#
#                 acc2 = np.vstack((acc2, np.array(sens1)))
#                 gyro2 = np.vstack((gyro2, np.array(sens2)))
#                 # print(acc2.shape)
#                 # print(f"Successfully taken acc2, gyro2 for {count} iteration ")
#                 # print(f"ACC2: {acc1}, gyro2: {gyro1}")
#                 # print("Processing", count)
#                 # print("Acceleration", acc1)
#                 # print("Gyro", gyro1)
#                 # print("Accesleration 2", acc2)
#                 # print("Gyro2", gyro2)
#
#         count = count+1
#
#         #print(f"Acc: {sensor_data[0]}, Gyro: {sensor_data[1]}")
#         #await asyncio.sleep(1)


async def main():
    # Create an asyncio queue
    queue = asyncio.Queue()

    # Run both tasks concurrently
    task1 = asyncio.create_task(read_sensor(queue))
    task2 = asyncio.create_task(process_data(queue))

    # Wait for both tasks to complete
    await asyncio.gather(task1, task2)


if __name__ == "__main__":
    asyncio.run(main())
