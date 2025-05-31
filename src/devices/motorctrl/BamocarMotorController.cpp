/*
 * BamocarMotorController.cpp
 */
#include "BamocarMotorController.h"
#include "../misc/Statemachine.h"
#include <sstream>


class StatemachineDevice;

BamocarMotorController::BamocarMotorController() : MotorController() {    
    selectedGear = DRIVE;
    commonName = "Bamocar Inverter";
    shortName = "BamocarInverter";
}

void BamocarMotorController::earlyInit()
{
    prefsHandler = new PrefHandler(BAMOCARINVERTER);
}

void BamocarMotorController::setup() {
    tickHandler.detach(this);

    Logger::info("add device: Bamocar Inverter (id:%X, %X)", BAMOCARINVERTER, this);

    loadConfiguration();
    MotorController::setup(); // run the parent class version of this function

    running = true;
    setPowerMode(modeTorque);
    setSelectedGear(DRIVE);
    setOpState(ENABLE);

    setAttachedCANBus(1);
    //Can Message to Bamocar for the actual speed
    attachedCANBus->attach(this, 0x100, 0x000, false);
    
    tickHandler.attach(this, CFG_TICK_INTERVAL_MOTOR_CONTROLLER_BAMOCAR);

    // on start up make sure that the car is not locked up
    // initialize freeRolling message
    freeRolling.len = 3;
    freeRolling.id = 0x201;
    freeRolling.buf[0] = 0x51;
    // 0x04 to DISABLE
    // var.buf[1] = 0x04;
    // 0x00 to ENABLE
    freeRolling.buf[1] = 0x04;
    freeRolling.buf[2] = 0x00;
    attachedCANBus->sendFrame(freeRolling);
    // initialize speed command message
    var.len = 3;
    var.id = 0x201; 

    //send message to get speed update every 100ms.
    var.buf[0] = 0x3D;
    var.buf[1] = 0x30;
    var.buf[2] = 0x64;
    attachedCANBus->sendFrame(var);

    //send message to get motor CONTROLLER temperature update every 100ms.
    var.buf[0] = 0x3D;
    var.buf[1] = 0x4a;
    var.buf[2] = 0x64;
    attachedCANBus->sendFrame(var);

    //send message to get motor temperature update every 100ms.
    var.buf[0] = 0x3D;
    var.buf[1] = 0x49;
    var.buf[2] = 0x64;

    attachedCANBus->sendFrame(var);
    enable_sent = false;
    disable_sent = true;
    last_sent_value = 0;
    mappedMotorTorque = 0;


    //FOR DEBUGGING - TESTING
    //send message to get mains voltage update every 100ms.
    // var.buf[0] = 0x3D;
    // var.buf[1] = 0x06;
    // var.buf[2] = 0x64;
    // attachedCANBus->sendFrame(var);


}



void BamocarMotorController::handleTick() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    //if the brake is pressed beyond a certain point set the speed back down to 0
    //if the brake is pressed also send a speed signal but lower than the current speed
    //if the throttle is released a little then the speed should still be similar, not immediately down to 0
    //max press accelerate max 
    //make it as close to a regular car
    //brakes regular car as well
    //willl never really have a cruising speed
    //linear
    //max push max acceleration

    //CONFIGURATIONS FOR THE MOTORCONTROLLER
    //CONFIGURATIONS FOR THE MOTORCONTROLLER

    
    MotorController::handleTick();
    switch (operationState) {
        case DISABLED:
            attachedCANBus -> sendFrame(freeRolling);
            last_sent_value = 0;
            disable_sent = true;
            enable_sent = false;
            break;
        case ENABLE:
            if (throttleRequested < 0) throttleRequested = 0;
            if (throttleRequested > 1000 && throttleRequested < 1300) throttleRequested = 1000;
            if (throttleRequested > 1400) throttleRequested = 0;

            throttleAnalogValue = throttleRequested / 10 * 10; // truncating it to the nearest 10th percent
            if (throttleAnalogValue < 50)
            {
                throttleAnalogValue = 0;
                if(!disable_sent){
                    attachedCANBus -> sendFrame(freeRolling);
                    last_sent_value = 0;
                    disable_sent = true;
                    enable_sent = false;
                }
            }
            else{
                mappedMotorTorque = throttleAnalogValue/10 * 20;
                uint32_t secondhalf = (mappedMotorTorque & 0xFF);
                uint32_t firsthalf = ((mappedMotorTorque >> 8));
                
                if (!enable_sent){
                    var.buf[0] = 0x51;
                    var.buf[1] = 0x00;
                    var.buf[2] = 0x00;
                    attachedCANBus->sendFrame(var);
                    disable_sent = false;
                    enable_sent = true;

                }
                else if (last_sent_value != mappedMotorTorque) //0x31 for speed, 0x90 for torque
                {
                    var.buf[0] = 0x90;
                    var.buf[1] = secondhalf;
                    var.buf[2] = firsthalf; 
                    attachedCANBus->sendFrame(var);
                    last_sent_value = mappedMotorTorque;
                }
                break;
            }
    }
}

void BamocarMotorController::handleCanFrame(const CAN_message_t &frame) {
    // Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
    //                   frame.id, frame.len, 
    //                   frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
    //                   frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);

    /*
     var.buf[0] = 0x3D;
    var.buf[1] = 0x30;
    var.buf[2] = 0x64;
    attachedCANBus->sendFrame(var);

    //send message to get motor CONTROLLER temperature update every 100ms.
    var.buf[0] = 0x3D;
    var.buf[1] = 0x4a;
    var.buf[2] = 0x64;
    attachedCANBus->sendFrame(var);

    //send message to get motor temperature update every 100ms.
    var.buf[0] = 0x3D;
    var.buf[1] = 0x49;
    var.buf[2] = 0x64;*/
    int payload = frame.buf[2] * 256 + frame.buf[1];
    switch (frame.buf[0]) {
        case 0x06: 
            Logger::info("Voltage reading : %d", payload);
            break;    
        case 0x4a:
        {
            double temp = motorControllerToCelsius(payload);
            Logger::info("BAMOCAR temp : %f", temp);
            break;
        }
        case 0x49:
        {
            double temp = motorToCelsius(payload);
            Logger::info("motor temp : %f", temp);
            break;
        }
        

}
}
static const struct {
    int32_t value;  
    int16_t tempC; 
} motorTempLookup[] = {
    {7414, -35}, {7687, -30}, {7962, -25}, {8240, -20}, {8520, -15},
    {8802, -10}, {9085, -5},  {9369,  0},  {9654,  5},  {9939, 10},
    {10225, 15}, {10510, 20}, {10795, 25}, {11080, 30}, {11364, 35},
    {11646, 40}, {11927, 45}, {12207, 50}, {12485, 55}, {12762, 60},
    {13036, 65}, {13308, 70}, {13578, 75}, {13846, 80}, {14111, 85},
    {14373, 90}, {14633, 95}, {14890, 100}, {15144, 105}, {15391, 110},
    {15632, 115}, {15852, 120}, {16061, 125}, {16251, 130}, {16421, 135},
    {16569, 140}, {16692, 145}, {16789, 150}, {16857, 155}
};

double BamocarMotorController::motorToCelsius(const uint16_t reading) const {
    for (int i = 1; i < (int)(sizeof(motorTempLookup) / sizeof(motorTempLookup[0])); ++i) {
        if (reading <= motorTempLookup[i].value) {
            double t1 = motorTempLookup[i-1].tempC;
            double t2 = motorTempLookup[i].tempC;
            double v1 = motorTempLookup[i-1].value;
            double v2 = motorTempLookup[i].value;

            double ratio = (reading - v1) / (v2 - v1);
            return t1 + ratio * (t2 - t1);
        }
    }
    return (double)motorTempLookup[sizeof(motorTempLookup) / sizeof(motorTempLookup[0]) - 1].tempC;
}

static const struct {
        int16_t tempC;
        uint16_t value;
    } controllerTempLookup[] = {
        { 125, 28480 }, { 120, 28179 }, { 115, 27851 }, { 110, 27497 },
        { 105, 27114 }, { 100, 26702 }, {  95, 26261 }, {  90, 25792 },
        {  85, 25296 }, {  80, 24775 }, {  75, 24232 }, {  70, 23671 },
        {  65, 23097 }, {  60, 22515 }, {  55, 21933 }, {  50, 21357 },
        {  45, 20793 }, {  40, 20250 }, {  35, 19733 }, {  30, 19247 },
        {  25, 18797 }, {  20, 18387 }, {  15, 18017 }, {  10, 17688 },
        {   5, 17400 }, {   0, 17151 }, {  -5, 16938 }, { -10, 16757 },
        { -15, 16609 }, { -20, 16487 }, { -25, 16387 }, { -30, 16308 }
    };

double BamocarMotorController::motorControllerToCelsius(const uint16_t reading) const {
    for (int i = 1; i < (int)(sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0])); ++i) {
        if (reading >= controllerTempLookup[i].value) {
            double t1 = controllerTempLookup[i-1].tempC;
            double t2 = controllerTempLookup[i].tempC;
            double v1 = controllerTempLookup[i-1].value;
            double v2 = controllerTempLookup[i].value;

            double ratio = (reading - v2) / (v1 - v2);
            return t2 + ratio * (t1 - t2);
        }
    }
    return (double)controllerTempLookup[sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0]) - 1].tempC;
}
    



void BamocarMotorController::setGear(Gears gear) {
    selectedGear = gear;
    //if the gear was just set to drive or reverse and the DMOC is not currently in enabled
    //op state then ask for it by name
    if (selectedGear != NEUTRAL) {
        operationState = ENABLE;
    }
    //should it be set to standby when selecting neutral? I don't know. Doing that prevents regen
    //when in neutral and I don't think people will like that.
}

DeviceId BamocarMotorController::getId() {
    return (BAMOCARINVERTER);
}

uint32_t BamocarMotorController::getTickInterval()
{
    return CFG_TICK_INTERVAL_MOTOR_CONTROLLER_BAMOCAR;
}

void BamocarMotorController::loadConfiguration() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();

    if (!config) {
        config = new BamocarMotorControllerConfiguration();
        setConfiguration(config);
    }

    MotorController::loadConfiguration(); // call parent
}

void BamocarMotorController::saveConfiguration() {
    MotorController::saveConfiguration();
}

BamocarMotorController bamocarMC;