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
    setOpState(STANDBY);

    setAttachedCANBus(1);
    //Can Message to Bamocar for the actual speed
    attachedCANBus->attach(this, 0x181, 0xFFF, false);
    
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

    var.buf[0] = 0x3D;
    var.buf[1] = 0x8f;
    var.buf[2] = 0xFF;
    attachedCANBus->sendFrame(var);
    
    enable_sent = false;
    disable_sent = true;
    last_sent_value = 0;
    mappedMotorTorque = 0;

    errorClear = false;
}



void BamocarMotorController::handleTick() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    MotorController::handleTick();
    
    if (extern_curr_state == S2){
        if (getOpState() == ENABLE){
            if (throttleRequested < 0) throttleRequested = 0;
            if (throttleRequested > 1000 && throttleRequested < 1300) throttleRequested = 1000;
            if (throttleRequested > 1400) throttleRequested = 0;

            throttleAnalogValue = throttleRequested / 10 * 10; // truncating it to the nearest 10th percent
            if (throttleAnalogValue <= 20)
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
                    //Transmitting transmission request BTB
                    var.buf[0] = 0x3D;
                    var.buf[1] = 0xE2;
                    var.buf[2] = 0x00;
                    attachedCANBus->sendFrame(var);

                    //Transmitting transmission request enable (hardware) refer to can manual
                    var.buf[0] = 0x3D;
                    var.buf[1] = 0xE8;
                    var.buf[2] = 0x00;
                    attachedCANBus->sendFrame(var);

                    //Transmitting Disable
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
            }
        }
    }
    
}

void BamocarMotorController::handleCanFrame(const CAN_message_t &frame) {
    // Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
    //                   frame.id, frame.len, 
    //                   frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
    //                   frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
    if (frame.buf[0] == 0x8F){
        if (frame.buf[1] == 0x20){ // mains voltage low, just clear it
            //this is the command to clear the error list
            var.buf[0] = 0x8E;
            var.buf[1] = 0x00;
            var.buf[2] = 0x00;
            attachedCANBus->sendFrame(var);
        }
    }
}

void BamocarMotorController::setGear(Gears gear) {
    selectedGear = gear;
}

void BamocarMotorController::setOpState(OperationState op){
    if (op == STANDBY){
        attachedCANBus -> sendFrame(freeRolling);
        last_sent_value = 0;
        disable_sent = true;
        enable_sent = false;
    }
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