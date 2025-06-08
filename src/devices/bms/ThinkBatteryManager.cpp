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

    setAttachedCANBus(1);

    //We will specify the broadcast message for the BMS can frame through the orion BMS software
    attachedCANBus->attach(this, 0x300, 0x7f0, false);
}

/*For all multibyte integers the format is MSB first, LSB last
*/
void ThinkBatteryManager::handleCanFrame(const CAN_message_t &frame) {
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 1);
    // https://www.orionbms.com/manuals/utility_o2/ where we got these values from.
    switch (frame.id) {
        case 0x301:
            {
                uint16_t pack_current = (frame.buf[0] << 8) | frame.buf[1];
                uint16_t pack_instant_volt = (frame.buf[2] << 8) | frame.buf[3];
                uint8_t pack_soc = frame.buf[4];
                uint16_t pack_open_volt = (frame.buf[5] << 8) | frame.buf[6]; 

                Logger::info("Pack Current: %.1f A, Instant Voltage: %.1f V, Open Voltage: %.1f V, SOC: %u%%",
                                pack_current / 10.0, pack_instant_volt / 10.0, pack_open_volt / 10.0, pack_soc/2.0);
            }
            break;
        case 0x302:
            {
                uint16_t pack_resistance = (frame.buf[2] << 8) | frame.buf[3];
                uint8_t high_temp = frame.buf[4];
                uint8_t low_temp = frame.buf[5];
                Logger::info("Pack Resistance: %.3f ohms, High Temp: %.1fC, Low Temp: %.1fC", 
                                pack_resistance / 1000.0, high_temp * 1.0, low_temp * 1.0);

            }
            break;
        case 0x303:
        {
            uint8_t checksum = frame.buf[7];
            uint16_t computedChecksum = frame.id + 8;
            for (int i = 0; i < 7; i++) {
                computedChecksum += frame.buf[i];
            }
            computedChecksum &= 0xFF;
            if ((uint8_t)computedChecksum != checksum) {
                Logger::error("BMS Fault Data: Checksum mismatch!");
                return;
            }
            uint32_t faultRegister = (frame.buf[0] << 24) | (frame.buf[1] << 16) | (frame.buf[2] << 8) | frame.buf[3];
            Logger::info("BMS Fault Codes:");
            for (const auto &fault : faultCodeMap) {
                uint32_t faultMask = fault.first;
                if (faultRegister & faultMask) {
                    Logger::error("  Fault Code: %s (%s)", fault.second.first.c_str(), fault.second.second.c_str());
                }
            }
        }
        break;
    }
    crashHandler.addBreadcrumb(ENCODE_BREAD("THBMS") + 2);
}

void ThinkBatteryManager::handleTick() {
    BatteryManager::handleTick(); //kick the ball up to papa
}

//Contactors in pack will close if we sent these two frames with all zeros.
void ThinkBatteryManager::sendKeepAlive()
{
    
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

    prefsHandler->read("CanbusNum", &config->canbusNum, 1);
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


