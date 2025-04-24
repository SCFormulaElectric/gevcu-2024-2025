#include "Dashboard.h"

// #include "BamocarMotorController.h"

static const struct {
    int32_t value;  
    int16_t tempC; 
} motorTempLookup[] = {
    {7414, -35}, {7687, -30}, {7962, -25}, {8240, -20}, {8520, -15},
    {8802, -10}, {9085, -5},  {9369,  0},  {9654,  5},  {9939, 10},
    {10225, 15}, {10510, 20}, {10795, 25}, {11080, 30}, {11364, 35},
    {11646, 40}, {11927, 45}, {12207, 50}, {12485, 55}, {12762, 60},
    {13036, 65}, {13308, 70}, {13578, 75}, {13846, 80}, {14111, 85},
    {14373, 90}, {14633, 95}, {14890, 100}, {15144, 105}, {15391, 110},
    {15632, 115}, {15852, 120}, {16061, 125}, {16251, 130}, {16421, 135},
    {16569, 140}, {16692, 145}, {16789, 150}, {16857, 155}
};

double DashboardDevice::motorToCelsius(const uint16_t reading) const {
    for (int i = 1; i < (int)(sizeof(motorTempLookup) / sizeof(motorTempLookup[0])); ++i) {
        if (reading <= motorTempLookup[i].value) {
            double t1 = motorTempLookup[i-1].tempC;
            double t2 = motorTempLookup[i].tempC;
            double v1 = motorTempLookup[i-1].value;
            double v2 = motorTempLookup[i].value;

            double ratio = (reading - v1) / (v2 - v1);
            return t1 + ratio * (t2 - t1);
        }
    }
    return (double)motorTempLookup[sizeof(motorTempLookup) / sizeof(motorTempLookup[0]) - 1].tempC;
}

static const struct {
        int16_t tempC;
        uint16_t value;
    } controllerTempLookup[] = {
        { 125, 28480 }, { 120, 28179 }, { 115, 27851 }, { 110, 27497 },
        { 105, 27114 }, { 100, 26702 }, {  95, 26261 }, {  90, 25792 },
        {  85, 25296 }, {  80, 24775 }, {  75, 24232 }, {  70, 23671 },
        {  65, 23097 }, {  60, 22515 }, {  55, 21933 }, {  50, 21357 },
        {  45, 20793 }, {  40, 20250 }, {  35, 19733 }, {  30, 19247 },
        {  25, 18797 }, {  20, 18387 }, {  15, 18017 }, {  10, 17688 },
        {   5, 17400 }, {   0, 17151 }, {  -5, 16938 }, { -10, 16757 },
        { -15, 16609 }, { -20, 16487 }, { -25, 16387 }, { -30, 16308 }
    };

double DashboardDevice::motorControllerToCelsius(const uint16_t reading) const {
    for (int i = 1; i < (int)(sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0])); ++i) {
        if (reading >= controllerTempLookup[i].value) {
            double t1 = controllerTempLookup[i-1].tempC;
            double t2 = controllerTempLookup[i].tempC;
            double v1 = controllerTempLookup[i-1].value;
            double v2 = controllerTempLookup[i].value;

            double ratio = (reading - v2) / (v1 - v2);
            return t2 + ratio * (t1 - t2);
        }
    }
    return (double)controllerTempLookup[sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0]) - 1].tempC;
}

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
    setAttachedCANBus(1);
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
            bamocar_temp = motorControllerToCelsius(bamocar_temp);
            break;
        //motor temp1
        case (0x49):
            motor_temp = decode_hex(frame.buf[1], frame.buf[2]);
            motor_temp = motorToCelsius(motor_temp);
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
