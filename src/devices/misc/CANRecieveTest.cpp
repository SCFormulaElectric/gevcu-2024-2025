#include "CANRecieveTest.h"

CANRecieveTest::CANRecieveTest() : Device() {
    commonName = "CAN Recieve Test";
    shortName = "CAN";
}

void CANRecieveTest::earlyInit()
{
    prefsHandler = new PrefHandler(CANRecieveTestID);
}

void CANRecieveTest::setup() {

    Logger::info("add device: CANRecieveTest (id: %X, %X)", CANRecieveTestID, this);


    Device::setup(); // run the parent class version of this function

    setAttachedCANBus(1);

    //Relevant BMS messages are 0x300 - 0x30F
    attachedCANBus->attach(this, 0x300, 0x7f0, false);
}

/*For all multibyte integers the format is MSB first, LSB last
*/
void CANRecieveTest::handleCanFrame(const CAN_message_t &frame) {
    Logger::info("Test id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                      frame.id, frame.len, 
                      frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                      frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
}

DeviceId CANRecieveTest::getId() {
    return (CANRecieveTestID);
}

DeviceType CANRecieveTest::getType() {
    return (DEVICE_MISC);
}

CANRecieveTest canRecieveTest;