/*
 * BamocarMotorController.cpp
 */
#include "BamocarMotorController.h"
#include <sstream>
#include "../../Statemachine.h"

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

    setAttachedCANBus(0);
    //Relevant BMS messages are 0x300 - 0x30F
    attachedCANBus->attach(this, 0x200, 0x7f0, false);
    
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
    bool once = true;

    
    MotorController::handleTick();
    if (throttleRequested < 0) throttleRequested = 0;
    if (throttleRequested > 1000 && throttleRequested < 1300) throttleRequested = 1000;
    if (throttleRequested > 1400) throttleRequested = 0;

    
    //Logger::warn("throttleRequested | %i", throttleRequested);
    // //rounding to nearest 10th
    // int a = throttleRequested;
   // throttleValue = throttleRequested;
    // Logger::console(extern_curr_state)

    if(extern_curr_state == S2)
    {
        //Logger::console("\n Bamocar: S2 loop");
        if (throttleAnalogValue < 50)
        {
            throttleAnalogValue = 0;
            
            // var.len = 3;
            // var.id = 0x201;

            // // attachedCANBus->sendFrame(var);
            // var.buf[0] = 0x51;
            // // 0x04 to DISABLE
            // // var.buf[1] = 0x04;
            // // 0x00 to ENABLE
            // var.buf[1] = 0x04;
            // var.buf[2] = 0x00;
            // attachedCANBus->sendFrame(var);

            attachedCANBus -> sendFrame(freeRolling);

        }
        else{
            throttleAnalogValue = throttleAnalogValue/20;
            //131071 is 2^17-1 which is in binary is 16 1's since this uses two's complement, this gives you a speed of around -1, as A increases to its max of 100, it will subtract around 2^16-1 from the binary giving you just a leading bit of 1 and a very large negative number as your speed.
            throttleAnalogValue = 65535 - throttleAnalogValue * 327;
            // (the following comments disregard the if/else statement)
            // at a = 0 (throttle not pressed), a becomes 2^17 -1 which is 17 1s. When this number is passed through first and second half and through the frame, the 17th bit gets truncated (buf values are 8 bits) --> -1 speed command
            // at a = 1000 (fully pressed), a becomes 65535 which is 16 1s. 

            uint32_t secondhalf = (throttleAnalogValue & 0xFF);
            uint32_t firsthalf = ((throttleAnalogValue >> 8));
            
            // Logger::warn("%i First half %i | Second half %i", a, firsthalf, secondhalf);
            // check if the motor will still spin even if the pedal is released all the way up

            var.len = 3;
            var.id = 0x201;

            // attachedCANBus->sendFrame(var);
            var.buf[0] = 0x51;
            // 0x04 to DISABLE
            // var.buf[1] = 0x04;
            // 0x00 to ENABLE
            var.buf[1] = 0x00;
            var.buf[2] = 0x00;
            attachedCANBus->sendFrame(var);


            
            var.buf[0] = 0x31;
            var.buf[1] = secondhalf; //secondhalf
            var.buf[2] = firsthalf; // first half
            attachedCANBus->sendFrame(var);
        }
    }   
    else if(extern_curr_state == S1){
        //Logger::console("\n Bamocar: S1 loop");
    }
    else {
        //Logger::console("\n Bamocar: S0 loop");
    }
}

void BamocarMotorController::handleCanFrame(const CAN_message_t &frame) {
    
    Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                      frame.id, frame.len, 
                      frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                      frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
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