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
    attachedCANBus->attach(this, 0x1839F380, 0x1FFFFFF8, true);
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

    faultNumMsg.id = 0x588;
    faultNumMsg.len = 1;
    faultNumMsg.buf[0] = 0;
    
    faultNum = 0;

    MaxCellTmpMsg.id = 0x738;
    MaxCellTmpMsg.len = 1;
    MaxCellTmpMsg.buf[0] = 20;
    maxtempvalue = 0;
    faultClearDelay = 0;

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
    faultNumMsg.buf[0] = faultNum;
    attachedCANBus->sendFrame(faultNumMsg); 
    MaxCellTmpMsg.buf[0] = maxtempvalue;
    attachedCANBus->sendFrame(MaxCellTmpMsg); 

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
        Logger::console("BMS Fault #%d active", faultNum);
}
    if (frame.id == 0x1839F380){
        maxtempvalue = frame.buf[2];
        Logger::console("MAX TEMP VALUE IS: %u \r\n", maxtempvalue);
    }
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
