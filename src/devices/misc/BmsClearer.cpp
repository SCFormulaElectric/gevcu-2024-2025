#include "BmsClearer.h"
#include "../../sys_io.h"

bool bms_fault_delayed = false;
BmsClearer::BmsClearer() : Device() {
    commonName = "BMS Clearer";
    shortName = "BmsClearer";
}

void BmsClearer::earlyInit() {
    prefsHandler = new PrefHandler(BMSCLEARER_ID);
}

void BmsClearer::setup() {
    tickHandler.detach(this);
    Logger::info("add device: BmsClearer (id: %X, %X)", BMSCLEARER_ID, this);

    Device::setup();

    setAttachedCANBus(0);  // BMS is on bus 0
    attachedCANBus->attach(this, 0x303, 0xFFF, false);
    attachedCANBus->attach(this, 0x18EEFF80, 0x1FFFFFFF, true);
    attachedCANBus->attach(this, 0x1839F380, 0x1FFFFFF8, true);
    attachedCANBus->attach(this, 0x1839F388, 0x1FFFFFF8, true);
    tickHandler.attach(this, BMSCLEARER_TICK);

    bms_fault_register = 0;

    clear_bms_msg.len = 8;
    clear_bms_msg.id = 0x7e3;
    clear_bms_msg.buf[0] = 0x01;
    clear_bms_msg.buf[1] = 0x04;
    clear_bms_msg.buf[2] = 0x00;
    clear_bms_msg.buf[3] = 0x00;
    clear_bms_msg.buf[4] = 0x00;
    clear_bms_msg.buf[5] = 0x00;
    clear_bms_msg.buf[6] = 0x00;
    clear_bms_msg.buf[7] = 0x00;

    bmsDelay = millis();
    bmsPendingFlag = 0;
    bms_fault_delayed = false;

    lastMsg18EEFF80 = millis();
    lastMsg1839F380 = millis();
    timeout18EEFF80 = false;
    timeout1839F380 = false;

    faultNumMsg.id = 0x588;
    faultNumMsg.len = 1;
    faultNumMsg.buf[0] = 0;
    
    faultNum = 0;

    MaxCellTmpMsg.id = 0x738;
    MaxCellTmpMsg.len = 1;
    MaxCellTmpMsg.buf[0] = 20;
    maxtempvalue = 0;
    faultClearDelay = 0;
    overTempStart = 0;

    // EDIT START - Bamocar RPM reading setup
    attachedCANBus->attach(this, 0x181, 0xFFF, false);

    bamocarRequest.len = 3;
    bamocarRequest.id = 0x201;

    // Request N-100% (Nmax) once — reply tells us what rpm = 32767
    bamocarRequest.buf[0] = 0x3D;
    bamocarRequest.buf[1] = 0xC8;
    bamocarRequest.buf[2] = 0x00;
    attachedCANBus->sendFrame(bamocarRequest);

    nmaxRpm = 0;
    nmaxReceived = false;
    speedNum = 0;
    currentRpm = 0;

    rpmMsg.id = 0x474;
    rpmMsg.len = 2;
    rpmMsg.buf[0] = 0;
    rpmMsg.buf[1] = 0;
    // EDIT END

    // EDIT START - Bamocar diagnostics init
    bamo_iist = 0;
    bamo_status = 0;
    bamo_ramp = 0;
    bamo_imax = 0;
    bamo_imaxpk = 0;
    bamo_icon = 0;
    bamo_staticReceived = false;
    // EDIT END
}

void BmsClearer::handleTick() {
    //    if (systemIO.getDigitalIn(1)) {
    //        if (bmsPendingFlag == 0){
    //            bmsPendingFlag = 1;
    //            bmsDelay = millis();
    //            faultClearDelay = 1;
    //        }
    //    if (bmsPendingFlag == 1){
    //        if (millis() - bmsDelay > 30000 && systemIO.getDigitalIn(1)){
    //            bms_fault_delayed = true;
    //            bmsPendingFlag = 0;
    //        }else{
    //            //attachedCANBus->sendFrame(clear_bms_msg);
    //            //Logger::console("BmsClearer: BMS fault high — sending clear");
    //        }
    //    }
    //}
    //else{
    //    bmsPendingFlag = 0;
    //    bms_fault_delayed = false;
    //}
    uint32_t now = millis();

    bool prev1833 = timeout18EEFF80;
    bool prev1839 = timeout1839F380;
    timeout18EEFF80 = (now - lastMsg18EEFF80) > 600;   // 3x 200ms period
    timeout1839F380 = (now - lastMsg1839F380) > 300;   // 3x 100ms period

    if (timeout18EEFF80 && !prev1833)
        Logger::error("BmsClearer: timeout on 0x18EEFF80 (no msg for >600ms)");
    else if (!timeout18EEFF80 && prev1833)
        Logger::info("BmsClearer: 0x18EEFF80 restored");

    if (timeout1839F380 && !prev1839)
        Logger::error("BmsClearer: timeout on 0x1839F380 (no msg for >300ms)");
    else if (!timeout1839F380 && prev1839)
        Logger::info("BmsClearer: 0x1839F380 restored");

    faultNumMsg.buf[0] = faultNum;
    attachedCANBus->sendFrame(faultNumMsg);
    MaxCellTmpMsg.buf[0] = maxtempvalue;
    attachedCANBus->sendFrame(MaxCellTmpMsg);
    // EDIT START - send RPM on 0x474 (big-endian int16)
    int16_t rpmOut = (int16_t)currentRpm;
    rpmMsg.buf[0] = (rpmOut >> 8) & 0xFF;
    rpmMsg.buf[1] = rpmOut & 0xFF;
    attachedCANBus->sendFrame(rpmMsg);
    // EDIT END
    Logger::console("BMS Fault #%d active | CAN timeouts: %s %s | Motor RPM: %ld", faultNum,
        timeout18EEFF80 ? "18EEFF80!" : "18EEFF80-ok",
        timeout1839F380 ? "1839F380!" : "1839F380-ok",
        currentRpm);

    // EDIT START - re-request Nmax if not yet received
    if (!nmaxReceived) {
        bamocarRequest.buf[0] = 0x3D;
        bamocarRequest.buf[1] = 0xC8;
        bamocarRequest.buf[2] = 0x00;
        attachedCANBus->sendFrame(bamocarRequest);
    }
    // EDIT END

    // EDIT START - Bamocar diagnostic requests (1Hz)
    // Static params: request until received
    if (!bamo_staticReceived) {
        uint8_t staticRegs[] = { 0x25, 0x4D, 0xC4, 0xC5 };
        for (uint8_t r : staticRegs) {
            bamocarRequest.buf[0] = 0x3D;
            bamocarRequest.buf[1] = r;
            bamocarRequest.buf[2] = 0x00;
            attachedCANBus->sendFrame(bamocarRequest);
        }
    }
    // Dynamic params: request every tick
    bamocarRequest.buf[0] = 0x3D;
    bamocarRequest.buf[1] = 0x20;  // I_IST actual current
    bamocarRequest.buf[2] = 0x00;
    attachedCANBus->sendFrame(bamocarRequest);
    bamocarRequest.buf[1] = 0xA0;  // status word
    attachedCANBus->sendFrame(bamocarRequest);
    // EDIT END

    if (maxtempvalue > 60) {
    if (overTempStart == 0) overTempStart = millis();
    if (millis() - overTempStart > 10000) {
        bms_fault_delayed = true;
        Logger::error("BmsClearer: overtemp %d C for >10s", maxtempvalue);
    }
} else {
    overTempStart = 0;
    bms_fault_delayed = false;
}

    //if (millis() - bmsDelay > 5000 && faultClearDelay == 1){
    //    attachedCANBus->sendFrame(clear_bms_msg);
    //    Logger::console("First clear");
    //    faultClearDelay = 0;
    //}
}

void BmsClearer::handleCanFrame(const CAN_message_t &frame) {
    if (frame.id == 0x303) {
        bms_fault_register = ((uint32_t)frame.buf[0] << 24) | ((uint32_t)frame.buf[1] << 16)
                           | ((uint32_t)frame.buf[2] << 8)  |  (uint32_t)frame.buf[3];
        if (bms_fault_register != 0x00000000) {
            Logger::console("BmsClearer: faulted");
              //  //attachedCANBus->sendFrame(clear_bms_msg);
        }

        
        if (bms_fault_register != 0) {
        for (int i = 31; i >= 4; i--) {
            if (bms_fault_register & (1u << i)) {
                faultNum = 32 - i;  // bit 31 → 1, bit 4 → 28
                break;
        }
    }
        }
        else{
            faultNum = 0;
}
        //Logger::console("BMS Fault #%d active", faultNum);
}
    if (frame.id == 0x18EEFF80) {
        lastMsg18EEFF80 = millis();
    }
    if (frame.id == 0x1839F380){
        lastMsg1839F380 = millis();
        maxtempvalue = frame.buf[2];
        //Logger::console("MAX TEMP VALUE IS: %u \r\n", maxtempvalue);
    }
    if (frame.id >= 0x1839F386 && frame.id <= 0x1839F38D) {
        uint8_t frameIdx = frame.id - 0x1839F386;
        char buf[48];
        int pos = 0;
        for (int i = 0; i < frame.len && i < 8; i++)
            pos += snprintf(buf + pos, sizeof(buf) - pos, "<%02X> ", frame.buf[i]);
        Logger::console("0x1839F386 frame[%d]: %s", frameIdx, buf);
    }
    // EDIT START - Bamocar RPM reading + diagnostics
    if (frame.id == 0x181 && frame.len >= 3) {
        int16_t val = (int16_t)((uint16_t)frame.buf[1] | ((uint16_t)frame.buf[2] << 8));
        switch (frame.buf[0]) {
            case 0xC8:
                nmaxRpm = val;
                nmaxReceived = true;
                Logger::info("BmsClearer: Bamocar Nmax = %d rpm", nmaxRpm);
                break;
            case 0x30:
                speedNum = val;
                if (nmaxReceived && nmaxRpm != 0)
                    currentRpm = (int32_t)((speedNum / 32767.0f) * nmaxRpm);
                break;
            case 0x20:
                bamo_iist = val;
                Logger::console("BAMO I_IST(actual current): %d", bamo_iist);
                break;
            case 0xA0:
                bamo_status = (uint16_t)val;
                Logger::console("BAMO STATUS: 0x%04X", bamo_status);
                break;
            case 0x25:
                bamo_ramp = val;
                Logger::info("BAMO RAMP: %d", bamo_ramp);
                break;
            case 0x4D:
                bamo_imax = val;
                Logger::info("BAMO I_MAX: %d", bamo_imax);
                break;
            case 0xC4:
                bamo_imaxpk = val;
                Logger::info("BAMO I_MAX_PK: %d", bamo_imaxpk);
                break;
            case 0xC5:
                bamo_icon = val;
                Logger::info("BAMO I_CON_EFF: %d", bamo_icon);
                if (!bamo_staticReceived && bamo_ramp != 0 && bamo_imax != 0 && bamo_imaxpk != 0)
                    bamo_staticReceived = true;
                break;
        }
    }
    // EDIT END

    if (frame.id >= 0x1839F381 && frame.id <= 0x1839F385) {
        uint8_t frameIdx = frame.id - 0x1839F381;  // 0–4
        uint8_t base = frameIdx * 8;               // thermistor index 0, 8, 16, 24, 32
        Logger::console("TEMPS [%d-%d]: %d %d %d %d %d %d %d %d",
            base, base + 7,
            (int8_t)frame.buf[0], (int8_t)frame.buf[1],
            (int8_t)frame.buf[2], (int8_t)frame.buf[3],
            (int8_t)frame.buf[4], (int8_t)frame.buf[5],
            (int8_t)frame.buf[6], (int8_t)frame.buf[7]);
    }
}

DeviceId BmsClearer::getId() { return (BMSCLEARER_ID); }
DeviceType BmsClearer::getType() { return DEVICE_MISC; }
void BmsClearer::loadConfiguration() { Device::loadConfiguration(); }
void BmsClearer::saveConfiguration() { Device::saveConfiguration(); }

BmsClearer bmsClearer;
