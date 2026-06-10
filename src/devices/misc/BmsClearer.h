#ifndef BMSCLEARER_H_
#define BMSCLEARER_H_

#include <Arduino.h>
#include "../../config.h"
#include "../../TickHandler.h"
#include "../../CanHandler.h"
#include "../../Logger.h"
#include "../Device.h"

#define BMSCLEARER_ID       0x103A
#define BMSCLEARER_TICK     1000000  // 1 second

extern bool bms_fault_delayed;
class BmsClearer : public Device, CanObserver {
public:
    BmsClearer();
    void earlyInit();
    void setup();
    void handleTick();
    void handleCanFrame(const CAN_message_t &frame);
    DeviceId getId();
    DeviceType getType();
    void loadConfiguration();
    void saveConfiguration();


private:
    uint32_t bmsDelay;
    uint32_t bmsPendingFlag;
    uint32_t bms_fault_register;
    CAN_message_t clear_bms_msg;
    CAN_message_t faultNumMsg;
    uint8_t faultNum = 0;
    CAN_message_t MaxCellTmpMsg;
    uint8_t maxtempvalue;
    uint32_t faultClearDelay;
    uint32_t overTempStart;


    uint32_t lastMsg18EEFF80;  // expected every 200ms
    uint32_t lastMsg1839F380;  // expected every 100ms
    bool timeout18EEFF80;
    bool timeout1839F380;

    // EDIT START - Bamocar RPM reading
    CAN_message_t bamocarRequest;
    CAN_message_t rpmMsg;
    int16_t nmaxRpm;       // N-100% (0xC8): rpm value that corresponds to num 32767
    bool nmaxReceived;
    int16_t speedNum;      // raw signed value from SPEED_IST (0x30)
    int32_t currentRpm;
    // EDIT END

    // EDIT START - Bamocar diagnostics
    int16_t bamo_iist;        // actual phase current (0x20)
    uint16_t bamo_status;     // status word (0xA0)
    int16_t bamo_ramp;        // internal torque ramp (0x25)
    int16_t bamo_imax;        // max current (0x4D)
    int16_t bamo_imaxpk;      // peak current limit (0xC4)
    int16_t bamo_icon;        // continuous current limit (0xC5)
    bool bamo_staticReceived; // true once ramp/current limits are read
    // EDIT END
};

#endif
