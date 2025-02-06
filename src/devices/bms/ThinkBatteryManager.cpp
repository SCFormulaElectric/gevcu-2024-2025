/*
 * ThinkBatteryManager.cpp
 *
 * Interface to the BMS which is within the Think City battery packs
 *
Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "ThinkBatteryManager.h"
#include "FaultCodes.h"

ThinkBatteryManager::ThinkBatteryManager() : BatteryManager() {
    allowCharge = false;
    allowDischarge = false;
    commonName = "Think City BMS";
    shortName = "ThinkBMS";
}

void ThinkBatteryManager::earlyInit()
{
    prefsHandler = new PrefHandler(THINKBMS);
}

void ThinkBatteryManager::setup() {
    tickHandler.detach(this);

    Logger::info("add device: Th!nk City BMS (id: %X, %X)", THINKBMS, this);

    loadConfiguration();

    BatteryManager::setup(); // run the parent class version of this function

    ThinkBatteryManagerConfiguration *config = (ThinkBatteryManagerConfiguration *)getConfiguration();

    ConfigEntry entry;
    //        cfgName                 helpText                               variable ref        Type                   Min Max Precision Funct
    entry = {"THINK-CANBUS", "Set which CAN bus to connect to (0-2)", &config->canbusNum, CFG_ENTRY_VAR_TYPE::BYTE, 0, 2, 0, nullptr};
    cfgEntries.push_back(entry);

    setAttachedCANBus(config->canbusNum);

    //We will specify the broadcast message for the BMS can frame through the orion BMS software
    attachedCANBus->attach(this, 0x300, 0x7f0, false);

    tickHandler.attach(this, CFG_TICK_INTERVAL_BMS_THINK);
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 0);
}

/*For all multibyte integers the format is MSB first, LSB last
*/
void ThinkBatteryManager::handleCanFrame(const CAN_message_t &frame) {
    
    /*
        The format for this message is as follows:

        Byte 0: Cell ID (8 bit, starting with 0)
        Byte 1&2: Instant Voltage (16 bit, unit: 0.1mv)
        Byte 3&4: Internal Resistance (15 bit, unit: 0.01mOhm)
        Byte 5&6: Open Voltage (16 bit, unit: 0.1mv)
        Byte 7: Checksum (8 bit)

        Bit 8 in byte #3 is whether or not the cell is shunting (1 indicates current is being shunted, 0 means it is not).

        Checksum Calculation:

        Take the broadcast ID and add 8 (the length).

        Add bytes 0-6 to the value from step 1.

        Chop off the least significant 8 bits (effectively turning it into an unsigned byte) and that will be the checksum value.

        If the computed checksum does not equal the provided checksum, the values should be discarded.
     */
    int temp;
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 1);
    canHandlerBus0.process(frame);
    switch (frame.id) {
        case 0x308: // Cell Data
            {
                uint8_t cellID = frame.buf[0];
                uint16_t instantVoltage = (frame.buf[1] << 8) | frame.buf[2]; // 0.1mV unit
                uint16_t internalResistance = ((frame.buf[3] & 0x7F) << 8) | frame.buf[4]; // 0.01mOhm unit
                uint16_t openVoltage = (frame.buf[5] << 8) | frame.buf[6]; // 0.1mV unit
                bool isShunting = (frame.buf[3] & 0x80) != 0; // Bit 8 in byte 3

                // Checksum verification
                uint8_t checksum = frame.buf[7];
                uint16_t computedChecksum = frame.id + 8; // Step 1
                for (int i = 0; i < 7; i++) {
                    computedChecksum += frame.buf[i]; // Step 2
                }
                computedChecksum &= 0xFF; // Step 3 - Keep only the least significant 8 bits

                if ((uint8_t)computedChecksum != checksum) {
                    // Invalid data, discard values
                    return;
                }
            }
            break;

        case 0x309: // Fault Data
            {
                uint8_t checksum = frame.buf[7];
                uint16_t computedChecksum = frame.id + 8; // Step 1
                for (int i = 0; i < 7; i++) {
                    computedChecksum += frame.buf[i]; // Step 2
                }
                computedChecksum &= 0xFF; // Step 3

                if ((uint8_t)computedChecksum != checksum) {
                    Logger::error("BMS Fault Data: Checksum mismatch!");
                    return;
                }

                // Extract the 32-bit fault register (assuming the fault register is in the first 4 bytes)
                uint32_t faultRegister = (frame.buf[0] << 24) | (frame.buf[1] << 16) | (frame.buf[2] << 8) | frame.buf[3];

                Logger::info("BMS Fault Codes:");

                // Iterate over the faultCodeMap to check which bits are set
                for (const auto &fault : faultCodeMap) {
                    uint32_t faultMask = fault.first;  // Fault bit mask (e.g., 0x80000000 for P0AA1)
                    
                    // If the corresponding fault bit is set in the faultRegister
                    if (faultRegister & faultMask) {
                        // Output the fault code and description
                        Logger::info("  Fault Code: %s (%s)", fault.second.first.c_str(), fault.second.second.c_str());
                    }
                }
            }
            break;


    }
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 2);
}

void ThinkBatteryManager::handleTick() {
    BatteryManager::handleTick(); //kick the ball up to papa

    sendKeepAlive();

}

//Contactors in pack will close if we sent these two frames with all zeros.
void ThinkBatteryManager::sendKeepAlive()
{
    CAN_message_t output;
    output.len = 3;
    output.id = 0x310;
    output.flags.extended = 0; //standard frame
    for (int i = 0; i < 8; i++) output.buf[i] = 0;
    attachedCANBus->sendFrame(output);

    output.id = 0x311;
    output.len = 2;
    attachedCANBus->sendFrame(output);
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 3);
}

DeviceId ThinkBatteryManager::getId()
{
    return (THINKBMS);
}

bool ThinkBatteryManager::hasPackVoltage()
{
    return true;
}

bool ThinkBatteryManager::hasPackCurrent()
{
    return true;
}

bool ThinkBatteryManager::hasTemperatures()
{
    return true;
}

bool ThinkBatteryManager::isChargeOK()
{
    return allowCharge;
}

bool ThinkBatteryManager::isDischargeOK()
{
    return allowDischarge;
}

void ThinkBatteryManager::loadConfiguration() {
    ThinkBatteryManagerConfiguration *config = (ThinkBatteryManagerConfiguration *)getConfiguration();

    if (!config) {
        config = new ThinkBatteryManagerConfiguration();
        setConfiguration(config);
    }

    BatteryManager::loadConfiguration(); // call parent

    prefsHandler->read("CanbusNum", &config->canbusNum, 0);
}

void ThinkBatteryManager::saveConfiguration() {
    ThinkBatteryManagerConfiguration *config = (ThinkBatteryManagerConfiguration *)getConfiguration();

    if (!config) {
        config = new ThinkBatteryManagerConfiguration();
        setConfiguration(config);
    }
    prefsHandler->write("CanbusNum", config->canbusNum);
    BatteryManager::saveConfiguration();
}

ThinkBatteryManager thinkBMS;


