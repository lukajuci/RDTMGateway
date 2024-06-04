#pragma once

#include "SinglePortModule.h"
#include <Arduino.h>
#include <functional>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "Adafruit_MMC56x3.h"
#include "../mesh/generated/meshtastic/rdtmtelemetry.pb.h"
#include "ProtobufModule.h"
#include "NodeDB.h"


const float hardiron_x = 0;
const float hardiron_y = 0;
const short dir_reg = 0x0001;
const short spd_reg = 0x0002;
const float Pi = 3.14159;
const uint32_t heartbeat = 300000; // 5 minutes


class DataNodeModule : private concurrency::OSThread, public ProtobufModule<meshtastic_rdtmmetrics>
{
    
    public:
        DataNodeModule()
        : concurrency::OSThread("DataNodeModule"),
          ProtobufModule("DataNode", meshtastic_PortNum_PRIVATE_APP, &meshtastic_rdtmmetrics_msg)
        {
            lastMeasurementPacket = nullptr;
            setIntervalFromNow(10 * 1000);
        }
    private:        
        meshtastic_MeshPacket *lastMeasurementPacket;
        uint32_t lastSentToMesh = 0;
        uint32_t lastSentToPhone = 0;
        uint8_t readDir();
        uint8_t readSpd();
        uint8_t convertToHeading(float direction);
        uint8_t getTemp();
        bool firstTime = 1;
        unsigned long started = 0;
        float heading;
        float latitude;
        float longitude;
        uint8_t directions[30] = {};
        uint8_t speeds[30]={50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
                            50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
                            50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
        meshtastic_rdtmmetrics m;
        uint8_t counter;

    protected:
        virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_rdtmmetrics *p) override;
        virtual int32_t runOnce() override;
        bool sendTelemetry(uint8_t direction[], uint8_t speed[], NodeNum dest = NODENUM_BROADCAST);
        
};