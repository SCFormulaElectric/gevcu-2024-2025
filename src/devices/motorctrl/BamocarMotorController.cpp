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

// Registers read back every second to confirm the settings in the inverter (reportSettings()).
// The first five are written by applySpeedLimiter(); the current limits are only read.
enum { CF_NLIM, CF_RAMP_ACC, CF_RAMP_DEC, CF_KP, CF_TI, CF_IRAMP, CF_IMAX, CF_IMAXPK, CF_ICON };
static const uint8_t CONFIRM_REGS[BAMOCAR_CONFIRM_REG_COUNT] = {
    0x34, // N-Lim
    0x35, // ramps: high = M R-Acc, low = N R-Acc (ms)
    0xED, // ramps: high = M R-Dec, low = N R-Dec (ms)
    0x2C, // speed Kp
    0x2D, // speed Ti (ms)
    0x25, // current ramp (us)
    0x4D, // I max eff, motor maximum current (0.1 A rms)
    0xC4, // I max pk, inverter peak current
    0xC5, // I con eff, inverter continuous current
};

// Append "label value OK |", "label value MISMATCH (sent x) |" or "label no reply |"
static int appendCheck(char *buf, int pos, int size, const char *label, bool seen, uint32_t got, uint32_t want, const char *unit) {
    if (pos >= size) return pos;
    if (!seen)
        return pos + snprintf(buf + pos, size - pos, " %s no reply |", label);
    if (got == want)
        return pos + snprintf(buf + pos, size - pos, " %s %lu%s OK |", label, (unsigned long)got, unit);
    return pos + snprintf(buf + pos, size - pos, " %s %lu%s MISMATCH (sent %lu) |", label, (unsigned long)got, unit, (unsigned long)want);
}

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
    // Torque ramps: time from 0 to full torque (NDrive manual 8.14). Lower = snappier throttle.
    entry = {"MRACC", "Bamocar torque ramp up M R-Acc (ms 0 to full torque, 0-4000; 65535 = keep inverter value)", &config->torqueAccRamp, CFG_ENTRY_VAR_TYPE::UINT16, 0, 65535, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"MRDEC", "Bamocar torque ramp down M R-Dec (ms full torque to 0, 0-4000; 65535 = keep inverter value)", &config->torqueDecRamp, CFG_ENTRY_VAR_TYPE::UINT16, 0, 65535, 0, nullptr};
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
    appliedTorqueRamps = 0xFFFFFFFF;
    writtenKp = 0;
    writtenTi = 0;
    confirmSeen = 0;
    memset(confirmValue, 0, sizeof(confirmValue));
    memset(lastSettingsReport, 0, sizeof(lastSettingsReport));
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
 * Print what the inverter actually holds (read back last second) next to what the GEVCU wrote.
 * Only prints when something changed, so a changed setting gets one confirmation line.
 */
void BamocarMotorController::reportSettings() {
    if (appliedNLim == 0xFFFFFFFF) return; // nothing written yet

    char buf[sizeof(lastSettingsReport)];
    int size = sizeof(buf);
    memset(buf, 0, size); // whole buffer is compared with the last report
    int pos = snprintf(buf, size, "BAMOCAR SETTINGS (read back):");
    uint32_t ramp35 = confirmValue[CF_RAMP_ACC], rampED = confirmValue[CF_RAMP_DEC];
    bool seen35 = confirmSeen & (1 << CF_RAMP_ACC), seenED = confirmSeen & (1 << CF_RAMP_DEC);
    pos = appendCheck(buf, pos, size, "N-Lim(32767=off)", confirmSeen & (1 << CF_NLIM), confirmValue[CF_NLIM] & 0xFFFF, appliedNLim, "");
    pos = appendCheck(buf, pos, size, "M R-Acc", seen35, ramp35 >> 16, appliedTorqueRamps >> 16, " ms");
    pos = appendCheck(buf, pos, size, "M R-Dec", seenED, rampED >> 16, appliedTorqueRamps & 0xFFFF, " ms");
    pos = appendCheck(buf, pos, size, "N R-Acc", seen35, ramp35 & 0xFFFF, BAMOCAR_LIMITER_SPEED_RAMP_MS, " ms");
    pos = appendCheck(buf, pos, size, "N R-Dec", seenED, rampED & 0xFFFF, BAMOCAR_LIMITER_SPEED_RAMP_MS, " ms");
    pos = appendCheck(buf, pos, size, "speed Kp", confirmSeen & (1 << CF_KP), confirmValue[CF_KP] & 0xFFFF, writtenKp, "");
    pos = appendCheck(buf, pos, size, "speed Ti", confirmSeen & (1 << CF_TI), confirmValue[CF_TI] & 0xFFFF, writtenTi, " ms");
    if (pos > size - 2) pos = size - 2; // first line truncated
    buf[pos] = '\0';
    int secondLine = pos + 1;           // second line starts after the first one's terminator
    pos = secondLine;
    if (pos < size) pos += snprintf(buf + pos, size - pos, "BAMOCAR CURRENT LIMITS (read only):");
    if (pos < size) {
        if (confirmSeen & (1 << CF_IRAMP)) pos += snprintf(buf + pos, size - pos, " current ramp %lu us |", (unsigned long)(confirmValue[CF_IRAMP] & 0xFFFF));
        else pos += snprintf(buf + pos, size - pos, " current ramp no reply |");
    }
    if (pos < size) {
        if (confirmSeen & (1 << CF_IMAX)) {
            uint16_t v = confirmValue[CF_IMAX] & 0xFFFF;
            pos += snprintf(buf + pos, size - pos, " I max eff (motor) %u = %u.%u A rms |", v, v / 10, v % 10);
        } else pos += snprintf(buf + pos, size - pos, " I max eff (motor) no reply |");
    }
    if (pos < size) {
        if (confirmSeen & (1 << CF_IMAXPK)) pos += snprintf(buf + pos, size - pos, " I max pk %lu (raw) |", (unsigned long)(confirmValue[CF_IMAXPK] & 0xFFFF));
        else pos += snprintf(buf + pos, size - pos, " I max pk no reply |");
    }
    if (pos < size) {
        if (confirmSeen & (1 << CF_ICON)) pos += snprintf(buf + pos, size - pos, " I con eff %lu (raw)", (unsigned long)(confirmValue[CF_ICON] & 0xFFFF));
        else pos += snprintf(buf + pos, size - pos, " I con eff no reply");
    }

    buf[size - 1] = '\0';
    if (memcmp(buf, lastSettingsReport, size) == 0) return;
    memcpy(lastSettingsReport, buf, size);
    // Serial directly: Logger::console formats into a 200 byte buffer and these lines can be longer
    Serial.println(buf);
    Serial.println(buf + secondLine);
}

/*
 * Write the speed limiter setpoints to the Bamocar's RAM (never its EEPROM), then read back
 * N-Lim and the status word. Called once a second so a power-cycled inverter gets them again
 * and a changed RPM / LIMKP / LIMTI / MRACC / MRDEC setting takes effect within a second.
 * N-Lim: 32767 = 100 % of N-100% (0xC8). RPM at or above N-100% turns the limiter off.
 * The torque ramps share registers 0x35 / 0xED with the speed ramps (high / low half).
 */
void BamocarMotorController::applySpeedLimiter() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    if (limiterNMax == 0) return; // N-100% unknown: can't scale N-Lim

    reportSettings(); // compare last second's readbacks with what was written then

    uint32_t nLim = 32767;
    if (config->speedMax < limiterNMax) nLim = (uint32_t)config->speedMax * 32767 / limiterNMax;
    appliedNLim = nLim;

    uint16_t accRamp = limiterTorqueAccRamp;
    uint16_t decRamp = limiterTorqueDecRamp;
    if (config->torqueAccRamp <= 4000) accRamp = config->torqueAccRamp;
    if (config->torqueDecRamp <= 4000) decRamp = config->torqueDecRamp;
    appliedTorqueRamps = ((uint32_t)accRamp << 16) | decRamp;
    writtenKp = config->limiterKp;
    writtenTi = config->limiterTi;

    writeReg16(0x34, nLim);
    writeReg32(0x35, ((uint32_t)accRamp << 16) | BAMOCAR_LIMITER_SPEED_RAMP_MS);
    writeReg32(0xED, ((uint32_t)decRamp << 16) | BAMOCAR_LIMITER_SPEED_RAMP_MS);
    writeReg16(0x2C, writtenKp);
    writeReg16(0x2D, writtenTi);

    CAN_message_t req;
    req.id = 0x201;
    req.len = 3;
    req.buf[0] = 0x3D;
    req.buf[2] = 0x00;
    // read everything back so reportSettings() can confirm it next second
    confirmSeen = 0;
    for (uint8_t i = 0; i < BAMOCAR_CONFIRM_REG_COUNT; i++) {
        req.buf[1] = CONFIRM_REGS[i];
        attachedCANBus->sendFrame(req);
    }
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

    //attachedCANBus->sendFrame(RequestDCMsg); // DC bus voltage not needed from the Bamocar (bytes were also swapped)
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
                    //Logger::console("TORQUE SENT: mapped=%d raw = [0x90, 0x%02X, 0x%02x]", mappedMotorTorque, secondhalf, firsthalf);
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
        // replaced by the BAMOCAR SETTINGS report
        //Logger::console("Bamocar reg 0x%02X %s: DLC %d, value %lu (0x%08lX), low16 %u, high16 %u",
        //                LIMITER_PARAMS[i].reg, LIMITER_PARAMS[i].name, frame.len, (unsigned long)value,
        //                (unsigned long)value, (unsigned)(value & 0xFFFF), (unsigned)(value >> 16));
        if (frame.buf[0] == 0xC8) limiterNMax = value & 0xFFFF;
        if (frame.buf[0] == 0x35) limiterTorqueAccRamp = value >> 16;
        if (frame.buf[0] == 0xED) limiterTorqueDecRamp = value >> 16;
        return;
    }
    // after the first readout: readbacks requested by applySpeedLimiter()
    if (appliedNLim != 0xFFFFFFFF) {
        for (uint8_t i = 0; i < BAMOCAR_CONFIRM_REG_COUNT; i++) {
            if (frame.buf[0] != CONFIRM_REGS[i]) continue;
            uint32_t value = frame.buf[1] | ((uint32_t)frame.buf[2] << 8);
            if (frame.len >= 6) value |= ((uint32_t)frame.buf[3] << 16) | ((uint32_t)frame.buf[4] << 24);
            confirmValue[i] = value;
            confirmSeen |= 1 << i;
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
    prefsHandler->read("TorqueAccRamp", &config->torqueAccRamp, BAMOCAR_KEEP_INVERTER_RAMP);
    prefsHandler->read("TorqueDecRamp", &config->torqueDecRamp, BAMOCAR_KEEP_INVERTER_RAMP);
}

void BamocarMotorController::saveConfiguration() {
    BamocarMotorControllerConfiguration *config = (BamocarMotorControllerConfiguration *)getConfiguration();
    prefsHandler->write("LimiterKp", config->limiterKp);
    prefsHandler->write("LimiterTi", config->limiterTi);
    prefsHandler->write("TorqueAccRamp", config->torqueAccRamp);
    prefsHandler->write("TorqueDecRamp", config->torqueDecRamp);
    MotorController::saveConfiguration(); // writes the checksum, so it goes last
}

BamocarMotorController bamocarMC;