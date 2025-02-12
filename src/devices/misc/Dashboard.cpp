#include "Dashboard.h"

// #include "BamocarMotorController.h"

DashboardDevice::DashboardDevice():Device() {
    commonName = "Dashboard";
    shortName = "Dash";
}

void DashboardDevice::earlyInit()
{
    prefsHandler = new PrefHandler(DashboardID);
}

void DashboardDevice::setup() {
    tickHandler.detach(this);
    Logger::info("add device: DashboardDevice (id: %X, %X)", DashboardID, this);
    Device::setup(); // run the parent class version of this function
    setAttachedCANBus(0);
    //Bamocar only sends ID 190. if needed we can add more things that it listens to later (ie. BMS)
    attachedCANBus->attach(this, 0x190, 0xfff, false);
    tickHandler.attach(this, DashboardTickInt);
    speed = 0;
    bamocar_temp = 0;
    motor_temp = 0;
    battery = 0;
    
    //set previous values to dummy number so that the first inital message will be sent.
    prev_speed = 99999;
    prev_bamocar_temp = 99999;
    prev_motor_temp = 99999;
    prev_battery = 99999;

    var.len = 4; // changed to 4, max space for classes
    var.id = 0x203; // Can change the ID if needed
}
/**
 * since the can frames come into with backward order, we will have to reverse them again and add the two numbers.
 */
int DashboardDevice::decode_hex(const int64_t first_half, const int64_t second_half) const{
    //second_half has 256 more weight since it is in the 2nd place of base 16, 16^2 = 256.
    return second_half * 256 + first_half;
}


/*For all multibyte integers the format is MSB first, LSB last
*/
void DashboardDevice::handleCanFrame(const CAN_message_t &frame) {
    switch (frame.buf[0]){
        //motor speed
        case (0x30):
            speed = decode_hex(frame.buf[1], frame.buf[2]);
            break;
        //bamocar temp
        case (0x4a):
            bamocar_temp = decode_hex(frame.buf[1], frame.buf[2]);
            break;
        //motor temp1
        case (0x49):
            motor_temp = decode_hex(frame.buf[1], frame.buf[2]);
            break;
    }
}

DeviceId DashboardDevice::getId() {
    return (DashboardID);
}

DeviceType DashboardDevice::getType() {
    return (DEVICE_MISC);
}

void DashboardDevice::handleTick()
{
    if (prev_motor_temp != motor_temp || prev_bamocar_temp != bamocar_temp || speed != prev_speed || battery != prev_battery){
        var.buf[0] = speed;
        var.buf[1] = motor_temp;
        var.buf[2] = battery;
        var.buf[3] = bamocar_temp;
        attachedCANBus->sendFrame(var);
        prev_speed = speed;
        prev_motor_temp = motor_temp;
        prev_battery = battery;
        prev_bamocar_temp = bamocar_temp;
    }
}
DashboardDevice dash_device;
