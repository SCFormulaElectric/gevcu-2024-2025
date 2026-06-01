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
};

#endif
