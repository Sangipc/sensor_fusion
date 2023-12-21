from events_manager import listen
import time
import asyncio
import struct

class SyncManager:
    isInSyncingProgress = False
   
    def __init__(self, globalConnectedSensors):
        self.currentProgress = 0
        self.sensors = globalConnectedSensors
        self.globalSyncingSensors = []
        self.syncingTimeoutId = -1
        self.syncingDoneList = []
        self.root_timestamp = 0
        #self.init()
    
    # def init(self):9
    #     listen('syncingEvent', self.eventHandler, False, eventName, parameters)
    def timestamp_handler(self,data):
        hexData = data.hex()
        time = self.timeStampConvert(hexData)
        self.root_timestamp = time
        print("Root Adress timestamp",self.root_timestamp)


    def timeStampConvert(self, data):
        t = bytes.fromhex(data[:8])
        return struct.unpack('<I', t)[0]
    def eventHandler(self, eventName, parameters):
        print(parameters.sensor.address + " Syncing eventName " + eventName)
        switcher = {
            'bleSensorDisconnected': self.handleBleSensorDisconnected,
            'bleSensorConnected': self.handleBleSensorConnected,
            'bleSensorSyncingDone': self.handleBleSensorSyncingDone
        }
        handler = switcher.get(eventName, lambda: None)
        handler(parameters)
    
    def handleBleSensorDisconnected(self, parameters):
        # def reconnectSensor():
        #     self.bleHandler.connectSensor(parameters.sensor)
        # time.sleep(2)
        # reconnectSensor()
        pass
    
    def handleBleSensorConnected(self, parameters):
        time.sleep(1.5)
        parameters.sensor.readRecordingAck()
        
    
    def handleBleSensorSyncingDone(self, parameters):
        print(parameters.sensor.address + " isSuccess " + parameters.isSuccess)
        self.syncingDoneList.append({'sensor': parameters.sensor, 'isSuccess': parameters.isSuccess})
        print(parameters.sensor.address + " syncing done list: " + self.syncingDoneList)
        self.onSyncingDone()
    

    async def startSyncing(self):
        # print("startSyncing isInSyncingProgress " + isInSyncingProgress)
        # if isInSyncingProgress:
        #     return
        isInSyncingProgress = True
        self.globalSyncingSensors = []
        self.currentProgress = 0
        self.syncingDoneList = []
        isSuccess = False
        rootAddress = "D4:22:CD:00:80:C9"
        for sensor in self.sensors:
            if rootAddress == "":
                rootAddress = sensor.address
            self.globalSyncingSensors.append(sensor)
        print("startSyncing root " + rootAddress + ", devices ")
        print(self.globalSyncingSensors)
        for sensor in self.globalSyncingSensors:
            await sensor.startBleSyncing(rootAddress)

    def handleSyncingTimeout(self):
        print("Syncing timeout")
        if isInSyncingProgress:
            isInSyncingProgress = False
            self.onSyncingDone()
    
    def handleDisconnectSensors(self):
        # if isInSyncingProgress:
        #     for sensor in self.globalSyncingSensors:
        #         self.bleHandler.disconnectSensor(sensor)
        pass
    
    # Implement in parent class .TODO
    def onSyncingDone(self):
        print("onSyncingDone " + self.sensors.length + ", " + self.globalSyncingSensors.length + ", " + self.syncingDoneList.length)
        if self.sensors.length == self.globalSyncingSensors.length and self.syncingDoneList.length == self.globalSyncingSensors.length:
            successCount = 0
            for item in self.syncingDoneList:
                if item.isSuccess:
                    successCount += 1
            isAllSuccess = (successCount == self.syncingDoneList.length)
            if isAllSuccess:
                print("All sensors synchronized")

    def bleErrorHandler(error):
        if(error):
            print(sensor.address + " BLE_UUID_RECORDING_CONTROL write " + error)



    
