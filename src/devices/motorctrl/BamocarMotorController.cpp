/*
 * BamocarMotorController.cpp
 */
#include "BamocarMotorController.h"
#include "../misc/Statemachine.h"
#include <sstream>


class StatemachineDevice;

// Registers that set up the Bamocar's speed limiter in torque mode (NDrive manual 12.1, 13.3).
// Only read here, never written, so we can see their current values and encoding.
static const struct { uint8_t reg; const char *name; } LIMITER_PARAMS[] = {
    {0x34, "N-Lim"},
    {0xC8, "N-100%"},
    {0x35, "ramps: low = N R-Acc ms, high = M R-Acc ms"},
    {0xED, "ramps: low = N R-Dec ms, high = M R-Dec ms"},
    {0x2C, "speed Kp"},
    {0x2D, "speed Ti"},
    {0x3B, "speed TiM"},
    {0x40, "status (bit 6 = T-Nlim, speed-limited in torque mode)"},
};
static const uint8_t NUM_LIMITER_PARAMS = sizeof(LIMITER_PARAMS) / sizeof(LIMITER_PARAMS[0]);

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

    // The speed limit itself is the motor controller's RPM setting (serial console: RPM=4500).
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    ConfigEntry entry;
    entry = {"LIMKP", "Bamocar speed limiter Kp (speed controller gain, 0-200)", &config->limiterKp, CFG_ENTRY_VAR_TYPE::UINT16, 0, 200, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"LIMTI", "Bamocar speed limiter Ti (speed controller integral time, ms, 0-10000)", &config->limiterTi, CFG_ENTRY_VAR_TYPE::UINT16, 0, 10000, 0, nullptr};
    cfgEntries.push_back(entry);

    running = true;
    setPowerMode(modeTorque);
    setSelectedGear(REVERSE);
    setOpState(ENABLE);

    setAttachedCANBus(0);
    //Can Message to Bamocar for the actual speed
    //attachedCANBus->attach(this, 0x181, 0xFFF, false);
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
    motorTemperature = 0;
    lastTempSendTime = 0;
    RequestDCMsg.len = 8;
    RequestDCMsg.id = 0x201;
    RequestDCMsg.buf[0] = 0xEB;
    RequestDCMsg.buf[1] = 0x3D;
    RequestDCMsg.buf[2] = 0X00;
    RequestDCMsg.buf[3] = 0x00;
    RequestDCMsg.buf[4] = 0x00;
    RequestDCMsg.buf[5] = 0x00;
    RequestDCMsg.buf[6] = 0x00;
    RequestDCMsg.buf[7] = 0x00;

    limiterParamsLogged = 0;
    lastLimiterRequest = 0;
    limiterNMax = 0;
    limiterTorqueAccRamp = 0;
    limiterTorqueDecRamp = 0;
    appliedNLim = 0xFFFFFFFF;
    lastNLimReadback = 0xFFFFFFFF;
    lastLimiterActive = 0xFF;
}

void BamocarMotorController::writeReg16(uint8_t reg, uint16_t value) {
    CAN_message_t msg;
    msg.id = 0x201;
    msg.len = 3;
    msg.buf[0] = reg;
    msg.buf[1] = value & 0xFF;
    msg.buf[2] = value >> 8;
    attachedCANBus->sendFrame(msg);
}

void BamocarMotorController::writeReg32(uint8_t reg, uint32_t value) {
    CAN_message_t msg;
    msg.id = 0x201;
    msg.len = 5;
    msg.buf[0] = reg;
    msg.buf[1] = value & 0xFF;
    msg.buf[2] = (value >> 8) & 0xFF;
    msg.buf[3] = (value >> 16) & 0xFF;
    msg.buf[4] = value >> 24;
    attachedCANBus->sendFrame(msg);
}

/*
 * Write the speed limiter setpoints to the Bamocar's RAM (never its EEPROM), then read back
 * N-Lim and the status word. Called once a second so a power-cycled inverter gets them again
 * and a changed RPM / LIMKP / LIMTI setting takes effect within a second.
 * N-Lim: 32767 = 100 % of N-100% (0xC8). RPM at or above N-100% turns the limiter off.
 */
void BamocarMotorController::applySpeedLimiter() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    if (limiterNMax == 0) return; // N-100% unknown: can't scale N-Lim

    uint32_t nLim = 32767;
    if (config->speedMax < limiterNMax) nLim = (uint32_t)config->speedMax * 32767 / limiterNMax;
    if (nLim != appliedNLim) {
        if (nLim == 32767)
            Logger::console("Bamocar speed limiter off (RPM %u >= N-100%% %u)", config->speedMax, limiterNMax);
        else
            Logger::console("Bamocar speed limit %u rpm (N-Lim %lu of 32767), Kp %u, Ti %u ms",
                            config->speedMax, (unsigned long)nLim, config->limiterKp, config->limiterTi);
        appliedNLim = nLim;
    }

    writeReg16(0x34, nLim);
    writeReg32(0x35, ((uint32_t)limiterTorqueAccRamp << 16) | BAMOCAR_LIMITER_SPEED_RAMP_MS);
    writeReg32(0xED, ((uint32_t)limiterTorqueDecRamp << 16) | BAMOCAR_LIMITER_SPEED_RAMP_MS);
    writeReg16(0x2C, config->limiterKp);
    writeReg16(0x2D, config->limiterTi);

    CAN_message_t req;
    req.id = 0x201;
    req.len = 3;
    req.buf[0] = 0x3D;
    req.buf[2] = 0x00;
    req.buf[1] = 0x34;
    attachedCANBus->sendFrame(req);
    req.buf[1] = 0x40;
    attachedCANBus->sendFrame(req);
}

/*
 * Ask once (3D <reg> 00) for every limiter register that hasn't answered yet.
 * Called every second until all have replied, in case the Bamocar powers up after us.
 */
void BamocarMotorController::requestLimiterParams() {
    CAN_message_t req;
    req.id = 0x201;
    req.len = 3;
    req.buf[0] = 0x3D;
    req.buf[2] = 0x00;
    for (uint8_t i = 0; i < NUM_LIMITER_PARAMS; i++) {
        if (limiterParamsLogged & (1 << i)) continue;
        req.buf[1] = LIMITER_PARAMS[i].reg;
        attachedCANBus->sendFrame(req);
    }
}



void BamocarMotorController::handleTick() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    MotorController::handleTick();

    attachedCANBus->sendFrame(RequestDCMsg);
    if (millis() - lastLimiterRequest > 1000) {
        lastLimiterRequest = millis();
        // read every limiter register once first, so the logs show the inverter's own values
        if (limiterParamsLogged != (1 << NUM_LIMITER_PARAMS) - 1) requestLimiterParams();
        else applySpeedLimiter();
    }
    if (millis() - lastTempSendTime > 3000) {
        uint16_t tempC = (uint16_t)((motorTemperature - 9369) / 55.21f);
        motorTempMsg.id = 0x476;
        motorTempMsg.len = 2;
        motorTempMsg.buf[0] = (tempC >> 8) & 0xFF;
        motorTempMsg.buf[1] = tempC & 0xFF;
        attachedCANBus->sendFrame(motorTempMsg);
        lastTempSendTime = millis();
    }
    if (getOpState() == THROTTLE_ERROR){
        extern_curr_state = S0;
        //Logger::info("throttle errored");
        if (!disable_sent){
            attachedCANBus->sendFrame(freeRolling);
            last_sent_value = 0;
            disable_sent = true;
            enable_sent = false;
        }
        
    }
    else if (getOpState() == PLAUSIBILITY_ERROR){
        Logger::info("plausbiltiy error");
        if(!disable_sent){
            attachedCANBus->sendFrame(freeRolling);
            last_sent_value = 0;
            disable_sent = true;
            enable_sent = false;
        }
    }
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
                    var.buf[0] = 0x90;
                    var.buf[1] = 0x00;
                    var.buf[2] = 0x00;
                    attachedCANBus->sendFrame(var);
                    attachedCANBus -> sendFrame(freeRolling);
                    last_sent_value = 0;
                    disable_sent = true;
                    enable_sent = false;
                }
            }
            else{

                //mappedMotorTorque = throttleAnalogValue/10 * 20;
                mappedMotorTorque = (int32_t)throttleAnalogValue * 0x7FFF / 1000; //torque command range changed to 100%
                int16_t signedTorque = (int16_t) mappedMotorTorque;
                if (selectedGear == REVERSE) 
                    signedTorque = -signedTorque;
                uint32_t secondhalf = (mappedMotorTorque & 0xFF);
                uint32_t firsthalf = ((mappedMotorTorque >> 8));
    //shit
                int16_t commandedTorqueNm = 0;

                
    if (extern_curr_state == S2 && getOpState() == ENABLE) {
        commandedTorqueNm = (uint16_t)((throttleAnalogValue / 1000.0f) * config->torqueMax);
    }
    CAN_message_t torqueNmMsg;
    torqueNmMsg.id = 0x473;
    torqueNmMsg.len = 2;
    torqueNmMsg.buf[0] = (commandedTorqueNm >> 8) & 0xFF;
    torqueNmMsg.buf[1] = commandedTorqueNm & 0xFF;
    attachedCANBus->sendFrame(torqueNmMsg);
//end of shit
                
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
                    var.buf[1] = (uint8_t) (signedTorque & 0xFF);
                    var.buf[2] = (uint8_t) ((signedTorque >> 8) & 0xFF); 
                    attachedCANBus->sendFrame(var);
                    last_sent_value = mappedMotorTorque;
                    Logger::console("TORQUE SENT: mapped=%d raw = [0x90, 0x%02X, 0x%02x]", mappedMotorTorque, secondhalf, firsthalf);
                }
            }
        }
    }
    
}

void BamocarMotorController::handleCanFrame(const CAN_message_t &frame) {
    //Logger::console("RAW id=%X mask_test=%X", frame.id, frame.id & 0xFFF);

    /*
     Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                       frame.id, frame.len, 
                       frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                       frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
           */            
    if (frame.buf[0] == 0x8F){
        if (frame.buf[1] == 0x20){ // mains voltage low, just clear it
            //this is the command to clear the error list
            var.buf[0] = 0x8E;
            var.buf[1] = 0x00;
            var.buf[2] = 0x00;
            attachedCANBus->sendFrame(var);
        }
    }
    if (frame.buf[0] == 0x49) {
    motorTemperature = (int16_t)(frame.buf[1] | (frame.buf[2] << 8));
    float motorTempC = (motorTemperature - 9369) / 55.21f;
    //Logger::console("Motor temp: %.1f C", motorTempC);
    }
    // Limiter parameter replies: REGID, value LE; DLC 4 = 16-bit, DLC 6 = 32-bit (CAN manual 4.5)
    for (uint8_t i = 0; i < NUM_LIMITER_PARAMS; i++) {
        if (frame.buf[0] != LIMITER_PARAMS[i].reg || (limiterParamsLogged & (1 << i))) continue;
        limiterParamsLogged |= 1 << i;
        uint32_t value = frame.buf[1] | ((uint32_t)frame.buf[2] << 8);
        if (frame.len >= 6) value |= ((uint32_t)frame.buf[3] << 16) | ((uint32_t)frame.buf[4] << 24);
        Logger::console("Bamocar reg 0x%02X %s: DLC %d, value %lu (0x%08lX), low16 %u, high16 %u",
                        LIMITER_PARAMS[i].reg, LIMITER_PARAMS[i].name, frame.len, (unsigned long)value,
                        (unsigned long)value, (unsigned)(value & 0xFFFF), (unsigned)(value >> 16));
        if (frame.buf[0] == 0xC8) limiterNMax = value & 0xFFFF;
        if (frame.buf[0] == 0x35) limiterTorqueAccRamp = value >> 16;
        if (frame.buf[0] == 0xED) limiterTorqueDecRamp = value >> 16;
        return;
    }
    // after the first readout: readbacks requested by applySpeedLimiter()
    if (appliedNLim != 0xFFFFFFFF) {
        if (frame.buf[0] == 0x34) {
            uint32_t readback = frame.buf[1] | ((uint32_t)frame.buf[2] << 8);
            if (readback != lastNLimReadback) {
                if (readback != appliedNLim)
                    Logger::error("Bamocar N-Lim readback %lu, expected %lu", (unsigned long)readback, (unsigned long)appliedNLim);
                else
                    Logger::console("Bamocar N-Lim readback OK (%lu)", (unsigned long)readback);
                lastNLimReadback = readback;
            }
        }
        if (frame.buf[0] == 0x40) {
            uint8_t active = (frame.buf[1] & 0x40) ? 1 : 0; // T-Nlim: speed-limited while in torque control
            if (active != lastLimiterActive) {
                Logger::console("Bamocar speed limiter %s", active ? "ACTIVE (cutting torque)" : "inactive");
                lastLimiterActive = active;
            }
        }
    }


}

void BamocarMotorController::setGear(Gears gear) {
    selectedGear = gear;
}

void BamocarMotorController::setOpState(OperationState op){
    OperationState prevOpState = getOpState();
    MotorController::setOpState(op);
    // if (prevOpState == ENABLE){
    //     Logger::console("Transitioning to %d from %d", op, prevOpState);
    //     attachedCANBus->sendFrame(freeRolling);
    //     last_sent_value = 0;
    //     disable_sent = true;
    //     enable_sent = false;
    // }

    Logger::console("Current OpState: %d", getOpState());
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
    // defaults are Unitek's recommendation for torque cruise control (NDrive manual 13.3)
    prefsHandler->read("LimiterKp", &config->limiterKp, 5);
    prefsHandler->read("LimiterTi", &config->limiterTi, 400);
}

void BamocarMotorController::saveConfiguration() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    prefsHandler->write("LimiterKp", config->limiterKp);
    prefsHandler->write("LimiterTi", config->limiterTi);
    MotorController::saveConfiguration(); // writes the checksum, so it goes last
}

BamocarMotorController bamocarMC;