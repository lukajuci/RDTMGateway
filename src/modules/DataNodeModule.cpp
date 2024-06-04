/**
 * @file RangeTestModule.cpp
 * @brief Implementation of the RangeTestModule class and RangeTestModuleRadio class.
 *
 * As a sender, this module sends packets every n seconds with an incremented PacketID.
 * As a receiver, this module receives packets from multiple senders and saves them to the Filesystem.
 *
 * The RangeTestModule class is an OSThread that runs the module.
 * The RangeTestModuleRadio class handles sending and receiving packets.
 */
#include "DataNodeModule.h"
#include "../mesh/generated/meshtastic/rdtmtelemetry.pb.h"
#include "Default.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include "main.h"
#include "power.h"
#include "target_specific.h"

SFE_UBLOX_GNSS myGNSS;
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define SEC_PER_MIN 60

int32_t DataNodeModule::runOnce()
{
#if defined(ARCH_ESP32) || defined(ARCH_NRF52)

    if (firstTime) {

        firstTime = 0;
        LOG_INFO("Initializing Data Node Module\n");

        //setup from standalone firmware
        //Serial2.begin(115200);
        //initialize modbus communication
        if (!ModbusRTUClient.begin(9600)) {
            LOG_INFO("Failed to start Modbus RTU Client!");
            Serial2.println("Failed to start Modbus RTU Client!");
            while (1);
        }
        RS485.begin(9600);
        //initialize magnetometer
        if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
            /* There was a problem detecting the MMC5603 ... check your connections */
            LOG_INFO("Ooops, no MMC5603 detected ... Check your wiring!");
            while (1) delay(10);
        }
        sensors_event_t event;
        mmc.getEvent(&event);
        heading = (atan2(event.magnetic.y-hardiron_y,event.magnetic.x-hardiron_x) * 180) / Pi;
        if (heading < 0){
            heading = 360 + heading;
        }
        //initialize GPS
        //WB_IO2 is pin 34
        //uint8_t WB_IO2 = 34;
        pinMode(WB_IO2, OUTPUT);
        digitalWrite(WB_IO2, 1);
        Wire.begin();
        if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
        {
            LOG_INFO("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
            while (1);
        }
        myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
        latitude = myGNSS.getLatitude(); 
        longitude = myGNSS.getLongitude();

        started = millis(); // make a note of when we started
        return (10000);      // Sending first message 5 seconds after initialization.
        
    } else {
        uint32_t now = millis();
        directions[counter] = readDir();
        speeds[counter] = readSpd();
        counter++;
        if ((((now - lastSentToMesh) >= heartbeat)) &&
            airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
            airTime->isTxAllowedAirUtil()) {
            sendTelemetry(directions, speeds);
            lastSentToMesh = now;
            counter = 0;
        }
        return(RUN_SAME);       
    }
#endif
return disable();
}

uint8_t DataNodeModule::convertToHeading(float direction){
    //convert to cardinal directions, 1 = North, 2 = NorthEast, 3 = East, 4 = SouthEast, 5 = South
    // 6 = SouthWest, 7 = West, 8 = NorthWest, 0 = Error Reading
    if(direction <= 22.5 || direction > 337.5){
        return 1;
    } else if (direction <= 67.5 ){
        return 2;
    } else if (direction <= 112.5){
        return 3;
    } else if (direction <= 157.5){
        return 4;
    } else if (direction <= 202.5){
        return 5;
    } else if (direction <= 247.5){
        return 6;
    } else if (direction <= 292.5){
        return 7;
    } else if (direction <= 337.5){
        return 8;
    } else{
        return 0;
    }
}

uint8_t DataNodeModule::readDir(){
    float direction;
    if (!ModbusRTUClient.requestFrom(1, HOLDING_REGISTERS, dir_reg, 01)){
        // Serial2.print("failed to read registers! ");
        // Serial2.println(ModbusRTUClient.lastError());
        return 0;
    }
    else{
        direction = ModbusRTUClient.read();
    }
    if(direction > 359){
            direction -= 360;
    } else if(direction < 0 ){
            direction += 360;
    }
    Serial2.println(direction);
    return convertToHeading(direction);    
}

uint8_t DataNodeModule::readSpd(){
    uint16_t spd1_value;
    uint16_t spd2_value;
    float spd_value;
    uint32_t temp;
    uint8_t speed;
    if (!ModbusRTUClient.requestFrom(1, HOLDING_REGISTERS, spd_reg, 2)){
        // Serial2.print("failed to read registers! ");
        // Serial2.println(ModbusRTUClient.lastError());
        //error value
        return 50;
    }
    else{
        spd1_value = ModbusRTUClient.read();
        spd2_value = ModbusRTUClient.read();
        temp = ((uint32_t)spd2_value << 16) + spd1_value;
        memcpy(&spd_value, &temp, sizeof(spd_value));
        //anemometer only accurate up to 40 m/s
        if(spd_value > 40){
            speed = 40;
        } else{
            speed = spd_value;
        }
        Serial2.println(speed);
        return speed;
    }
    
}

uint8_t DataNodeModule::getTemp(){
    return mmc.readTemperature();
}


bool DataNodeModule::sendTelemetry(uint8_t direction[], uint8_t speed[], NodeNum dest)
{
    meshtastic_rdtmmetrics m;
    bool valid = false;
    for (int i = 0; i < 30; i++){
        m.directions[i] = direction[i];
        m.speeds[i] = speed[i];
    }
    m.time = getTime();
    m.temperature = getTemp();
    m.latitude = latitude;
    m.longitude = longitude;
    m.NodeNum = nodeDB->getNodeNum();
    Serial2.println(m.NodeNum);

    
    LOG_INFO("(Sending): NodeNum=%f, temperature=%f, latitude=%f, longitude=%f, directions=%f, "
                "speeds=%f\n",
                m.NodeNum, m.temperature, m.latitude, m.longitude, m.directions, m.speeds);
    meshtastic_MeshPacket *p = allocDataProtobuf(m);  
    p->to = dest;
    p->hop_limit = 7;
    p->decoded.want_response = true;
    p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    //release previous packet before occupying a new spot
    if (lastMeasurementPacket != nullptr)
        packetPool.release(lastMeasurementPacket);

    lastMeasurementPacket = packetPool.allocCopy(*p);
    LOG_INFO("Sending packet to mesh\n");
    service.sendToMesh(p, RX_SRC_LOCAL, true);


    return valid;
}

bool DataNodeModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_rdtmmetrics *t)
{
#ifdef DEBUG_PORT
        const char *sender = getSenderShortName(mp);

        LOG_INFO("(Received from %s): NodeNum=%f, temperature=%f, latitude=%f, longitude=%f, directions=%f, "
                 "speeds=%f\n",
                 sender, t->NodeNum, t->temperature, t->latitude, t->longitude, t->directions, t->speeds);
#endif
        // release previous packet before occupying a new spot
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);

    return false; // Let others look at this message also if they want
}
