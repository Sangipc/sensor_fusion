import asyncio
import multiprocessing
import time
import ble_sensor
import dataprocessing

Ts = 0.01  # sampling rate
N = 2  # number of IMUs
sensors = []

async def read_measurement(queue, device):
    sensor_queue = multiprocessing.Manager().Queue()
    _, address = device
    sensor = ble_sensor.BleSensor(address, sensor_queue)
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
    await sensor.setOutputRate(60)

    #print("Starting measurement reading")
    #print(sensor)
    sensor.payloadType = ble_sensor.PayloadMode.rateQuantities
    sensor.queue = queue
    await sensor.startMeasurement()

    print("\nNotifications enabled. Waiting for data...")
    #await asyncio.sleep(0.2)  # heading reset
    print("Start Recording")

    # default is no recording
    sensor.fileName = sensor.create_csvfile()
    sensor.recordFlag = True

    # Run within the timeToRunInMinute time range.
    startTimeSeconds = int(round(time.time()))
    timeToRunInMinute = 0.5
    print(f"run the test for {timeToRunInMinute} minutes")
    while int(round(time.time())) - startTimeSeconds < timeToRunInMinute * 60:
        await asyncio.sleep(0.001)

    # stop measurement
    await sensor.stopMeasurement()
    #await asyncio.sleep(0.5)

def always_read_imu(id, sensor_dict, devices):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    async def read_measurement_wrapper():
        await read_measurement(sensor_dict[id], devices[id])

    loop.run_until_complete(read_measurement_wrapper())


