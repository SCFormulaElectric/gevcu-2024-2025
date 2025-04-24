#ifndef Dashboard_H_
#define Dashboard_H_

#include <Arduino.h>
#include "../Device.h"
#include "../../DeviceManager.h"
#include "../../CanHandler.h"


#define DashboardID 0x521
#define DashboardTickInt 1000000

class DashboardDevice: public Device, CanObserver{
public:
    DashboardDevice(); //called nearly immediately to initialize your own variables
    void setup(); //called only if the device is actually enabled
    void earlyInit(); //called early and whether or not the device is enabled. Just used to setup configuration
    void handleTick(); 
    void handleCanFrame(const CAN_message_t &frame);
    int decode_hex(const int64_t first_half, const int64_t second_half) const;
    DeviceId getId();
    DeviceType getType();
    double motorToCelsius(const uint16_t reading) const;
    double motorControllerToCelsius(const uint16_t reading) const;
private:
    CAN_message_t var;
    int motor_temp;
    int bamocar_temp;
    int speed;
    int battery;
    //add variables so that we don't send unecessary CAN messages
    int prev_motor_temp;
    int prev_bamocar_temp;
    int prev_speed;
    int prev_battery;
};
#endif
