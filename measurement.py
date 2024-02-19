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
    await sensor.setOutputRate(30)

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

def always_process_data(sensor_dict):
    t = time.time()
    while True:
        data = {}
        for id in range(N):
            queue_data = sensor_dict.get(id)
            data[id] = queue_data.get()
            # print(data[id])

        quats = dataprocessing.process_data(data)
        # print(quats)
        inclination = dataprocessing.calculate_inclination(quats[0], quats[1])
        print(inclination)

        compute_time = time.time() - t
        still_wait = Ts - compute_time
        if still_wait <= 0:
            still_wait = 0

        time.sleep(still_wait)
        t = time.time()
