import numpy as np
import asyncio
import bleak
import time
import sys

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

# def assign_payload_type(value):
#     if value == 0:
#         return ble_sensor.PayloadMode.orientationEuler
#     elif value == 1:
#         return ble_sensor.PayloadMode.orientationQuaternion
#     elif value == 2:
#         return ble_sensor.PayloadMode.customMode1
#     elif value == 3:
#         return ble_sensor.PayloadMode.rateQuantities




async def main():
    devices = await Scanner.scan_and_filter_xsens_dot()

    if not devices:
        print("No Xsens DOT, considering turn them on or shake them from sleep mode")
        return

    sensors = []
    for d in devices:
        sensors.append(BleSensor(d.address))
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
        # default is no recording, here to enable the recording after heading reset
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


if __name__ == "__main__":
    asyncio.run(main())
