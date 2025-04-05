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
    if (throttleRequested < 0) throttleRequested = 0;
    if (throttleRequested > 1000 && throttleRequested < 1300) throttleRequested = 1000;
    if (throttleRequested > 1400) throttleRequested = 0;

    throttleAnalogValue = throttleRequested / 10 * 10; // rounding it to the nearest 10th percent
    if (throttleAnalogValue < 50)
    {
        throttleAnalogValue = 0;
        if(!disable_sent){
            attachedCANBus -> sendFrame(freeRolling);
            last_sent_value = 0;
            disable_sent = true;
            enable_sent = false;
            Logger::info("Here1");

        }
    }
    else{
        // throttleAnalogValue = throttleAnalogValue/20;
        //131071 is 2^17-1 which is in binary is 16 1's since this uses two's complement, this gives you a speed of around -1, as A increases to its max of 100, it will subtract around 2^16-1 from the binary giving you just a leading bit of 1 and a very large negative number as your speed.
        mappedMotorTorque = throttleAnalogValue/10 * 327;
        // (the following comments disregard the if/else statement)
        // at a = 0 (throttle not pressed), a becomes 2^17 -1 which is 17 1s. When this number is passed through first and second half and through the frame, the 17th bit gets truncated (buf values are 8 bits) --> -1 speed command
        // at a = 1000 (fully pressed), a becomes 65535 which is 16 1s. 

        uint32_t secondhalf = (mappedMotorTorque & 0xFF);
        uint32_t firsthalf = ((mappedMotorTorque >> 8));
        
        if (!enable_sent){
            var.buf[0] = 0x51;
            var.buf[1] = 0x00;
            var.buf[2] = 0x00;
            attachedCANBus->sendFrame(var);
            disable_sent = false;
            enable_sent = true;
            Logger::info("Here2");

        }
        else if (last_sent_value != mappedMotorTorque) //0x31 for speed, 0x90 for torque
        {
            Logger::info("Here3");
            var.buf[0] = 0x90;
            var.buf[1] = secondhalf; //secondhalf
            var.buf[2] = firsthalf; // first half
            attachedCANBus->sendFrame(var);
            last_sent_value = mappedMotorTorque;
        }
    }
    
}

void BamocarMotorController::handleCanFrame(const CAN_message_t &frame) {
    Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                      frame.id, frame.len, 
                      frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                      frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
    // switch (frame.buf[0]) {
    //     case 0x06: 
    //         int voltage = frame.buf[2] * 256 + frame.buf[1];
    //         Logger::console("Voltage reading : %d", voltage);
    //         break;    

    //     }
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