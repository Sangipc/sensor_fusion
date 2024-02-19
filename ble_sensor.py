from datetime import datetime
from multiprocessing.queues import Queue

import bleak
import asyncio
import aioprocessing
import time
import struct
import os
import csv
import numpy as np
from enum import Enum
import multiprocessing
from multiprocessing import Manager, Process


# from qmt import jointAxisEstHingeOlsson

class DotData:
    def __init__(self):
        self.name = []
        self.address = []
        self.timestamp = []
        self.acc = np.array([0, 0, 0])
        self.angularVelocity = np.array([0, 0, 0])
        self.magneticField = np.array([0, 0, 0])
class PayloadMode(Enum):
    orientationEuler = 0
    orientationQuaternion=1
    customMode1 = 2
    rateQuantities = 3
class BleSensor:
    BLE_MID_SYNCING                      = 0x02
    BLE_UUID_RECORDING_CONTROL           = "15177001-4947-11e9-8646-d663bd873d93"
    SYNCING_ID_SYNCING_RESULT            = 0x03
    SYNCING_ID_START_SYNCING             = 0x01
    BLE_UUID_RECORDING_ACK               = "15177002-4947-11e9-8646-d663bd873d93"
    DOT_Configuration_ServiceUUID         = "15171000-4947-11E9-8646-D663BD873D93"
    DOT_Configuration_Control_CharacteristicUUID = "15171002-4947-11E9-8646-D663BD873D93"
    DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93"
    DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93"
    Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93"
    DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93"
    DOT_MediumPayload_CharacteristicUUID = "15172003-4947-11E9-8646-D663BD873D93"
    DOT_LongPayload_CharacteristicUUID = "15172002-4947-11E9-8646-D663BD873D93"
    DOT_Battery_CharacteristicUUID = "15173001-4947-11E9-8646-D663BD873D93"

    Select_Orientation_Euler = bytes([1, 1, 4])
    Deselect_Orientation_Euler = bytes([1, 0, 4])
    Select_Orientation_Quaternion = bytes([1, 1, 5])
    Deselect_Orientation_Quaternion = bytes([1, 0, 5])
    Select_CustomMode1 = bytes([1, 1, 22])
    Deselect_CustomMode1 = bytes([1, 0, 22])
    Select_ratequantities = bytes([1, 1, 20])
    Deselect_ratequantities = bytes([1, 0, 20])
    Heading_Reset_Buffer = bytes([1, 0])
    ROLLOVER = 4294967295
    CLOCK_DELTA = 0.0002

    def __init__(self, address, queue):
        self._client = None
        self.dotdata = []
        self.name = []
        self.address = address
        self.recordFlag = False
        self.fileName = []
        self.payloadType = None
        self.characteristics = []
        self.syncManager = None
        self.syncTimestamp = 0
        self.sensorTimestamp = 0
        self.queue = queue
        # manager = multiprocessing.Manager()
        # self.imu_queues = manager.dict()
        # with Manager() as manager:
        #     self.queue = manager.Queue()


    async def readRecordingAck(self):
        data = await self._client.read_gatt_char[self.BLE_UUID_RECORDING_ACK]
        print(self.address + " BLE_UUID_RECORDING_ACK read " + data.hex())
        if error:
            return
        # print(self.address + " BLE_UUID_RECORDING_ACK read " + isInSyncingProgress)
        # Removed isInSyncingProgress in below condition. TODO
        if len(data) >= 5 and data[0] == self.BLE_MID_SYNCING and data[2] == self.SYNCING_ID_SYNCING_RESULT:
            isSuccess = (data[3] == 0x00)
            self.sendSyncingEvent('bleSensorSyncingDone', {'sensor': self, 'isSuccess': isSuccess})

    def checkSum(self,byte_array):
        _sum = 0

        for byte in byte_array:
            _sum += byte

        check_sum_result = (0x00FF & (-_sum))

        print("sum: {}, checkSum: {}".format(_sum, check_sum_result))

        return check_sum_result

    async def startBleSyncing(self, rootAddress):
        addressSlice = rootAddress.split(":")
        if len(addressSlice) != 6:
            addressSlice = rootAddress.split("-")
            if len(addressSlice) != 6:
                print("[startSyncing] invalid MAC address " + rootAddress)
                return False
        data = []
        data.append(self.BLE_MID_SYNCING) # MID
        #print(data)
        data.append(0x07) # LEN, exclude checkSum
        data.append(self.SYNCING_ID_START_SYNCING)
        print(data)
        for i in range(len(addressSlice)):
            data.append(int(addressSlice[len(addressSlice) - i - 1], 16))
        print(data)
        data.append(self.checkSum(data))
        print(data)
        buffer = bytes(data)
        await self._client.write_gatt_char(self.BLE_UUID_RECORDING_CONTROL, buffer, response=False)
        print(f"Successfully wrote data to characteristic")
        await asyncio.sleep(0.1)  # wait for response

    def sendSyncingEvent( self, eventName, parameters ):
        if self.syncManager == None :
            return
        if eventName == 'bleSensorError':
            time.sleep(0.1)
            self.syncManager.eventHandler(eventName, parameters)
            return
        self.syncManager.eventHandler(eventName, parameters)


    # Get the tags for fetching Battery Information
    async def getDeviceTag(self):
        await asyncio.sleep(0.2)  # wait for response
        # Read the device tag
        read_bytes = await self._client.read_gatt_char(self.DOT_Configuration_Control_CharacteristicUUID)
        device_tag_bytes = read_bytes[8:24]
        # remove the blank bytes
        device_tag_bytes = device_tag_bytes.replace(b'\x00', b'')
        # decode to ascii name
        device_tag = device_tag_bytes.decode()
        print(f"read device tag: {device_tag}.")
        return device_tag

    async def getBatteryInfo(self):
        # assign the device tag
        self.name = await self.getDeviceTag()
        # get battery info
        read_bytes = await self._client.read_gatt_char(self.DOT_Battery_CharacteristicUUID)
        await asyncio.sleep(0.1)  # wait for response
        batteryLevel = read_bytes[0]
        status = read_bytes[1]
        if status == 1:
            batteryChargingStatus = "Charging"
        else:
            batteryChargingStatus = "Not Charging"
        batteryInfo = f"{self.name}: batteryLevel: {batteryLevel}%, chargingStatus:{batteryChargingStatus}"
        return batteryInfo

    # Connecting sensors and Start Measurement
    async def connect(self):
        print(f"connecting to {self.address}")
        self._client = bleak.BleakClient(self.address)
        await self._client.connect()
        for service in self._client.services:
            # print("[Service] {}".format(service))
            for char in service.characteristics:
                if "read" in char.properties:
                    value = await self._client.read_gatt_char(char.uuid)
                    # print("[Characteristic] {} {}, Value: {}".format(char, ','.join(char.properties), value))


    async def enable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)
        await asyncio.sleep(0.1)  # wait for response



    async def startMeasurement(self):
        print("starting measurement")
        if self.payloadType == PayloadMode.orientationEuler:
            print("PayloadMode = orientationEuler")
            await self._client.start_notify(self.DOT_ShortPayload_CharacteristicUUID,
                                            self.orientationEuler_notification_handler)
            await self.enable_sensor(self.DOT_Control_CharacteristicUUID, self.Select_Orientation_Euler)

        elif self.payloadType == PayloadMode.orientationQuaternion:
            print("PayloadMode = orientationQuaternion")
            await self.enable_sensor(self.DOT_Control_CharacteristicUUID, self.Select_Orientation_Quaternion)
            await self._client.start_notify(self.DOT_ShortPayload_CharacteristicUUID,
                                            self.orientationQuaternion_notification_handler)
        elif self.payloadType == PayloadMode.customMode1:
            print("PayloadMode = customMode1")
            await self.enable_sensor(self.DOT_Control_CharacteristicUUID, self.Select_CustomMode1)
            await self._client.start_notify(self.DOT_MediumPayload_CharacteristicUUID, self.customMode1_notification_handler)

        elif self.payloadType == PayloadMode.rateQuantities:
            print("PayloadMode = rateQuantities")
            await self.enable_sensor(self.DOT_Control_CharacteristicUUID, self.Select_ratequantities)
            await self._client.start_notify(self.DOT_MediumPayload_CharacteristicUUID,
                                            self.rateQuantities_notification_handler)


    # Disconnecting sensors and Stop Measurement
    async def disable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)

    async def poweroffSensor(self):
        try:
            bArr = bytearray(32)
            bArr[0] = 2
            bArr[1] = 0
            bArr[2] = 1
            await self._client.write_gatt_char(self.DOT_Configuration_Control_CharacteristicUUID, bArr)
            print(f"power off sensor {self.address} successful")
        except bleak.exc.BleakError as e:
            print(f"power off sensor  {self.address} : {e}")

    async def stopMeasurement(self):
        if self.payloadType == PayloadMode.orientationEuler:
            print("Stop Measurement PayloadMode = orientationEuler")
            try:
                await self.disable_sensor(self.DOT_Control_CharacteristicUUID, self.Deselect_Orientation_Euler)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")

            try:
                await self._client.stop_notify(self.DOT_ShortPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

        elif self.payloadType == PayloadMode.orientationQuaternion:
            print("Stop Measurement PayloadMode = orientationQuaternion")
            try:
                await self.disable_sensor(self.DOT_Control_CharacteristicUUID, self.Deselect_Orientation_Quaternion)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")
            try:
                await self._client.stop_notify(self.DOT_ShortPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

        elif self.payloadType == PayloadMode.customMode1:
            print("Stop Measurement PayloadMode = customMode1")
            try:
                await self.disable_sensor(self.DOT_Control_CharacteristicUUID, self.Deselect_CustomMode1)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")
            try:
                await self._client.stop_notify(self.DOT_MediumPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

        elif self.payloadType == PayloadMode.rateQuantities:
            print("Stop Measurement PayloadMode = ratequantities")
            try:
                await self.disable_sensor(self.DOT_Control_CharacteristicUUID, self.Deselect_ratequantities)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")
            try:
                await self._client.stop_notify(self.DOT_MediumPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

    # Heading Reset
    async def doHeadingReset(self):
        await self._client.write_gatt_char(self.Heading_Reset_Control_CharacteristicUUID, self.Heading_Reset_Buffer)
        await asyncio.sleep(0.1)  # wait for response

    # Set Output rate
    async def setOutputRate(self, outputRate: int):
        if outputRate in [1, 4, 10, 12, 15, 20, 30, 60, 120]:
            bArr = bytearray(32)
            bArr[0] = 16
            bArr[24] = outputRate & 0xff
            bArr[25] = (outputRate >> 8) & 0xff
            print(f"Set output Rate to {outputRate}")
            return await self._client.write_gatt_char(self.DOT_Configuration_Control_CharacteristicUUID, bArr)
        else:
            print(f"Invalid output rate{outputRate}")
            return False

    # Data Parsing
    def timeStampConvert(self, data):
        t = bytes.fromhex(data[:8])
        return struct.unpack('<I', t)[0]


    # def convertSensorData(sensor, data, measuringPayloadId, isSyncingEnabled):
    #     if isSyncingEnabled:
    #         self.syncTimestamp = self.timeStampConvert(data)
    #     else:
    #         hrTime = process.hrtime()
    #         systemtime = hrTime[0] * 1000000 + hrTime[1] / 1000
    #         self.setSynchronizedTimestamp(data, systemtime)

    def setSynchronizedTimestamp(self, data):
        # dt = datetime.now()
        # systemTime = dt.microsecond
        rootTime = time.time()
        # rootTime = self.syncManager.root_timestamp

        sensorTimestamp = self.timeStampConvert(data)
        if self.syncTimestamp == 0:
            self.syncTimestamp = rootTime
            sensorTimestamp = sensorTimestamp
            return
        sensorTimeDiff = sensorTimestamp - self.sensorTimestamp
        if sensorTimeDiff < 0:
            sensorTimeDiff += self.ROLLOVER
        self.sensorTimestamp = sensorTimestamp
        self.syncTimestamp = self.syncTimestamp + sensorTimeDiff*(1+self.CLOCK_DELTA)
        if self.syncTimestamp > rootTime:
            self.syncTimestamp = rootTime
        return self.syncTimestamp

    def accConvert(self, data):
        AccX = struct.unpack('<f', bytes.fromhex(data[8:16]))[0]
        AccY = struct.unpack('<f', bytes.fromhex(data[16:24]))[0]
        AccZ = struct.unpack('<f', bytes.fromhex(data[24:32]))[0]
        Acc = np.array([AccX, AccY, AccZ])
        return Acc

    def eulerConvert(self, data):
        roll = struct.unpack('<f', bytes.fromhex(data[8:16]))[0]
        pitch = struct.unpack('<f', bytes.fromhex(data[16:24]))[0]
        yaw = struct.unpack('<f', bytes.fromhex(data[24:32]))[0]
        euler = np.array([roll, pitch, yaw])
        return euler

    def quaternionConvert(self, data):
        # unpacks the 4-byte data from the 8th to the 16th hex digit, convert to a float
        w = struct.unpack('<f', bytes.fromhex(data[8:16]))[0]
        x = struct.unpack('<f', bytes.fromhex(data[16:24]))[0]
        y = struct.unpack('<f', bytes.fromhex(data[24:32]))[0]
        z = struct.unpack('<f', bytes.fromhex(data[32:40]))[0]
        quat = np.array([w, x, y, z])
        return quat

    def orientationEuler_notification_handler(self, sender, data):
        # 20 bytes of data, only the first 16 bytes valid
        hexData = data.hex()
        time = self.setSynchronizedTimestamp(hexData)
        euler = self.eulerConvert(hexData)
        stringToPrint = "Time: {:.0f}, Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(
            time, euler[0], euler[1], euler[2])
        print(stringToPrint)
        if self.recordFlag == True:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.eulerAngle = euler
            self.record_data(dotdata)

    def customMode1_notification_handler(self, sender, data):
        hexData = data.hex()
        time = self.setSynchronizedTimestamp(hexData)
        euler = self.eulerConvert(hexData)
        freeAccX = struct.unpack('<f', bytes.fromhex(hexData[32:40]))[0]
        freeAccY = struct.unpack('<f', bytes.fromhex(hexData[40:48]))[0]
        freeAccZ = struct.unpack('<f', bytes.fromhex(hexData[48:56]))[0]
        freeAcc = np.array([freeAccX, freeAccY, freeAccZ])
        gyroX = struct.unpack('<f', bytes.fromhex(hexData[56:64]))[0]
        gyroY = struct.unpack('<f', bytes.fromhex(hexData[64:72]))[0]
        gyroZ = struct.unpack('<f', bytes.fromhex(hexData[72:80]))[0]
        gyro = np.array([gyroX, gyroY, gyroZ])
        stringToPrint = "Time: {:.0f}, Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(
            time, euler[0], euler[1],
            euler[2])
        stringToPrint += ", freeAccX: {:.1f}, freeAccY: {:.1f}, freeAccZ: {:.1f}".format(freeAccX, freeAccY,
                                                                                         freeAccZ)
        stringToPrint += ", gyroX: {:.1f}, gyroY: {:.1f}, gyroZ: {:.1f}".format(gyroX, gyroY,
                                                                                gyroZ)
        stringToPrint += " "
        print(stringToPrint)
        if self.recordFlag:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.eulerAngle = euler
            dotdata.freeAcc = freeAcc
            dotdata.angularVelocity = gyro
            self.record_data(dotdata)

    def orientationQuaternion_notification_handler(self, sender, data):
        # print(f"Received data: {data.hex()}")
        # 20 bytes of data
        hexData = data.hex()
        time = self.setSynchronizedTimestamp(hexData)
        quat = self.quaternionConvert(hexData)
        stringToPrint = "Time: {:.0f}, q0: {:.1f}, q1: {:.1f}, q2: {:.1f}, q3: {:.1f}".format(
            time, quat[0], quat[1], quat[2], quat[3])
        print(stringToPrint)
        if self.recordFlag == True:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.quaternion = quat
            self.record_data(dotdata)

    async def add_to_queue(self, sensor_data):
        if self.queue is not None:
            self.queue.put(sensor_data)
            #print("Data in queue")
            await asyncio.sleep(0.01)

    def rateQuantities_notification_handler(self, sender, data):

        hexData = data.hex()
        time = self.setSynchronizedTimestamp(hexData)
        acc = self.accConvert(hexData)
        gyroX = struct.unpack('<f', bytes.fromhex(hexData[32:40]))[0]
        gyroY = struct.unpack('<f', bytes.fromhex(hexData[40:48]))[0]
        gyroZ = struct.unpack('<f', bytes.fromhex(hexData[48:56]))[0]
        gyro = np.array([gyroX, gyroY, gyroZ])

        magX = struct.unpack('<h', bytes.fromhex(hexData[56:60]))[0] / (2 ** 12)
        magY = struct.unpack('<h', bytes.fromhex(hexData[60:64]))[0] / (2 ** 12)
        magZ = struct.unpack('<h', bytes.fromhex(hexData[64:68]))[0] / (2 ** 12)
        mag = np.array([magX, magY, magZ])
        stringToPrint = " "
        stringToPrint += "{:.0f}".format(time)
        stringToPrint += "AccX: {:.4f}, AccY: {:.4f}, AccZ: {:.4f}".format(acc[0], acc[1], acc[2])
        stringToPrint += " gyroX: {:.4f}, gyroY: {:.4f}, gyroZ: {:.4f}".format(gyroX, gyroY, gyroZ)

        stringToPrint += " magX: {:.4f}, magY: {:.4f}, magZ: {:.4f}".format(magX, magY, magZ)
        stringToPrint += " "
        #print(stringToPrint)
        sensor_data = {'gyro': gyro, 'acc': acc}
        asyncio.create_task(self.add_to_queue(sensor_data))

        if self.recordFlag:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.acc = acc
            dotdata.angularVelocity = gyro
            dotdata.magneticField = mag
            self.record_data(dotdata)




    # Store data into CSV
    def create_csvfile(self):
        # Extract necessary information from the data
        m_address = self.address.replace(':', '_')
        # Create the folder path
        folder_path = os.path.join(os.getcwd(), 'data_logging')
        # Create the folder if it does not exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        file_name = f"{m_address}.csv"
        # Create the file path
        file_path = os.path.join(folder_path, file_name)
        # Write the title row and the data to the csv file
        with open(file_path, mode='w', newline='') as csv_file:
            print(f"file created: {file_name}")
            writer = csv.writer(csv_file)
            if self.payloadType == PayloadMode.orientationEuler:
                writer.writerow(['SampleTimeFine', 'roll', 'pitch', 'yaw'])
            elif self.payloadType == PayloadMode.orientationQuaternion:
                writer.writerow(['SampleTimeFine', 'q0', 'q1', 'q2', 'q3'])
            elif self.payloadType == PayloadMode.customMode1:
                writer.writerow(
                    ['SampleTimeFine', 'roll', 'pitch', 'yaw', 'freeAccX', 'freeAccY', 'freeAccZ',
                     'gyroX', 'gyroY', 'gyroZ'])
            elif self.payloadType == PayloadMode.rateQuantities:
                writer.writerow(
                    ['SampleTimeFine', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyr_X', 'Gyr_Y', 'Gyr_Z', 'Mag_X', 'Mag_Y',
                     'Mag_Z'])

        csv_file.close()
        return file_path

    def record_data(self, data):
        # Write the new data to the existing csv file
        with open(self.fileName, mode='a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            if self.payloadType == PayloadMode.orientationEuler:
                writer.writerow([int(data.timestamp), data.eulerAngle[0], data.eulerAngle[1],
                                 data.eulerAngle[2]])
            elif self.payloadType == PayloadMode.orientationQuaternion:
                writer.writerow([data.timestamp, data.quaternion[0], data.quaternion[1],
                                 data.quaternion[2], data.quaternion[3]])
            elif self.payloadType == PayloadMode.customMode1:
                writer.writerow([data.timestamp, data.eulerAngle[0], data.eulerAngle[1],
                                 data.eulerAngle[2], data.freeAcc[0], data.freeAcc[1], data.freeAcc[2],
                                 data.angularVelocity[0], data.angularVelocity[1], data.angularVelocity[2]])
            elif self.payloadType == PayloadMode.rateQuantities:
                writer.writerow([data.timestamp, round(data.acc[0], 6), round(data.acc[1], 6),
                                 round(data.acc[2], 6), round(data.angularVelocity[0], 6),
                                 round(data.angularVelocity[1], 6), round(data.angularVelocity[2], 6),
                                 round(data.magneticField[0], 6), round(data.magneticField[1], 6),
                                 round(data.magneticField[2], 6)])


