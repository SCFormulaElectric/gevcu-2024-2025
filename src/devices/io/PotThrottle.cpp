/*
 * PotThrottle.cpp
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

#include "PotThrottle.h"
#include "../motorctrl/MotorController.h"


/*
 * Constructor
 */
PotThrottle::PotThrottle() : Throttle() {    
    commonName = "Potentiometer (analog) accelerator";
    shortName = "PotAccel";
}

void PotThrottle::earlyInit()
{
    prefsHandler = new PrefHandler(POTACCELPEDAL);
}

/*
 * Setup the device.
 */
void PotThrottle::setup() {
    crashHandler.addBreadcrumb(ENCODE_BREAD("PTTHR") + 0);
    tickHandler.detach(this); // unregister from TickHandler first

    Logger::info("add device: PotThrottle (id: %X, %X)", POTACCELPEDAL, this);

    loadConfiguration();

    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();

    Throttle::setup(); //call base class

    ConfigEntry entry;
    entry = {"TPOT", "Number of pots to use (1 or 2)", &config->numberPotMeters, CFG_ENTRY_VAR_TYPE::BYTE, 1, 2, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"TTYPE", "Set throttle subtype (1=std linear, 2=inverse)", &config->throttleSubType, CFG_ENTRY_VAR_TYPE::BYTE, 1, 2, 0, DEV_PTR(&PotThrottle::describeThrottleType)};
    cfgEntries.push_back(entry);
    entry = {"T1ADC", "Set throttle 1 ADC pin", &config->AdcPin1, CFG_ENTRY_VAR_TYPE::BYTE, 0, 255, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"T1MN", "Set throttle 1 min value", &config->minimumLevel1, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"T1MX", "Set throttle 1 max value", &config->maximumLevel1, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"T2ADC", "Set throttle 2 ADC pin", &config->AdcPin2, CFG_ENTRY_VAR_TYPE::BYTE, 0, 255, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"T2MN", "Set throttle 2 min value", &config->minimumLevel2, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"T2MX", "Set throttle 2 max value", &config->maximumLevel2, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);

    //set digital ports to inputs and pull them up all inputs currently active low
    //pinMode(THROTTLE_INPUT_BRAKELIGHT, INPUT_PULLUP); //Brake light switch
    fault_throttle_low_a = false;
    fault_throttle_high_a = false;
    fault_throttle_low_b = false;
    fault_throttle_high_b = false;
    fault_throttle_mismatch_ab = false;
    fault_brake_throttle_engaged = false;

    motorController = deviceManager.getMotorController();
    tickHandler.attach(this, CFG_TICK_INTERVAL_POT_THROTTLE);
}

/*
 * Process a timer event.
 */
void PotThrottle::handleTick() {
    crashHandler.addBreadcrumb(ENCODE_BREAD("PTTHR") + 1);
    Throttle::handleTick(); // Call parent which controls the workflow
    bool brakeEngaged;
    if (deviceManager.getBrake()){
        brakeEngaged = deviceManager.getBrake()->getLevel() > 0;
    }
    else{
        brakeEngaged = false;
    }
    bool throttleOver25 = getLevel() > 250;
    bool throttleUnder5 = getLevel() < 50;

    if (!fault_brake_throttle_engaged && brakeEngaged && throttleOver25 && motorController->getOpState() != 4) {
        fault_brake_throttle_engaged = true;
        Logger::error("Brake engaged while throttle > 25%% — initiating motor shutdown.");
        motorController->setOpState(5);
    }

    if (fault_brake_throttle_engaged) {
        if (throttleUnder5) {
            Logger::info("Throttle < 5%% — resuming regular operation.");
            fault_brake_throttle_engaged = false;
            motorController->setOpState(2);
        } else {
            if (motorController->getOpState() == 2) {
                motorController->setOpState(5);
            }
        }
    }
}

/*
 * Retrieve raw input signals from the throttle hardware.
 */
RawSignalData *PotThrottle::acquireRawSignal() {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();

    rawSignal.input1 = systemIO.getAnalogIn(config->AdcPin1);
    rawSignal.input2 = systemIO.getAnalogIn(config->AdcPin2);
    return &rawSignal;
}

/*
 * Perform sanity check on the ADC input values. The values are normalized (without constraining them)
 * and the checks are performed on a 0-1000 scale with a percentage tolerance
 */
bool PotThrottle::validateSignal(RawSignalData *rawSignal) {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();
    int32_t calcThrottle1, calcThrottle2;
    
    // error in throttle analog 1
    // too high 

    calcThrottle1 = normalizeInput(rawSignal->input1, config->minimumLevel1, config->maximumLevel1 );
    if (calcThrottle1 > (1000 + CFG_THROTTLE_TOLERANCE)) {
        if (status == OK)
            Logger::error(POTACCELPEDAL, "ERR_HIGH_T1: throttle 1 value out of range: %i", calcThrottle1);
        status = ERR_HIGH_T1;
        faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_HIGH_A, true);
        if (fault_throttle_high_a) {
            if (motorController->getOpState() == 2){
                motorController->setOpState(4);
            }
        } else {
            fault_throttle_high_a = true;
        }
        return false;
    } else {
        if (calcThrottle1 > 1000) calcThrottle1 = 1000;
        faultHandler.cancelOngoingFault(POTACCELPEDAL, FAULT_THROTTLE_HIGH_A);
        fault_throttle_high_a = false;
    }

    // error in throttle analog 
    // too low 
    if (calcThrottle1 < (0 - CFG_THROTTLE_TOLERANCE)) {
        if (status == OK)
            Logger::error(POTACCELPEDAL, "ERR_LOW_T1: throttle 1 value out of range: %i ", calcThrottle1);
        status = ERR_LOW_T1;
        faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_LOW_A, true);
        if (fault_throttle_low_a) {
            if (motorController->getOpState() == 2){
                motorController->setOpState(4);
            }
        } else {
            fault_throttle_low_a = true;
        }
        return false;
    } else {
        if (calcThrottle1 < 0) calcThrottle1 = 0;
        faultHandler.cancelOngoingFault(POTACCELPEDAL, FAULT_THROTTLE_LOW_A);
        fault_throttle_low_a = false;
    }

    if (config->numberPotMeters > 1) {
        calcThrottle2 = normalizeInput(rawSignal->input2, config->minimumLevel2, config->maximumLevel2);

        // error in throttle analog 2
        if (calcThrottle2 > (1000 + CFG_THROTTLE_TOLERANCE)) {
            if (status == OK)
                Logger::error(POTACCELPEDAL, "ERR_HIGH_T2: throttle 2 value out of range: %i", calcThrottle2);
            status = ERR_HIGH_T2;
            faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_HIGH_B, true);
            if (fault_throttle_high_b) {
                if (motorController->getOpState() == 2){
                    motorController->setOpState(4);
                }
            } else {
                fault_throttle_high_b = true;
            }
            return false;
        } else {
            if (calcThrottle2 > 1000) calcThrottle2 = 1000;
            faultHandler.cancelOngoingFault(POTACCELPEDAL, FAULT_THROTTLE_HIGH_B);
            fault_throttle_high_b = false;
        }

        // error in throttle analog 2
        if (calcThrottle2 < (0 - CFG_THROTTLE_TOLERANCE)) {
            if (status == OK)
                Logger::error(POTACCELPEDAL, "ERR_LOW_T2: throttle 2 value out of range: %i", calcThrottle2);
            status = ERR_LOW_T2;
            faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_LOW_B, true);
            if (fault_throttle_low_b) {
                if (motorController->getOpState() == MotorController::ENABLE){
                    motorController->setOpState(4);
                }
            } else {
                fault_throttle_low_b = true;
            }
            return false;
        } else {
            if (calcThrottle2 < 0) calcThrottle2 = 0;
            faultHandler.cancelOngoingFault(POTACCELPEDAL, FAULT_THROTTLE_LOW_B);
            fault_throttle_low_b = false;
        }

        // plausability diff too high
        if ((calcThrottle1 - ThrottleMaxErrValue) > calcThrottle2) { //then throttle1 is too large compared to 2
            if (status == OK)
                Logger::error(POTACCELPEDAL, "throttle 1 too high (%i) compared to 2 (%i)", calcThrottle1, calcThrottle2);
            status = ERR_MISMATCH;
            faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_MISMATCH_AB, true);
            if (fault_throttle_mismatch_ab) {
                if (motorController->getOpState() == 2){
                    motorController->setOpState(4);
                }
            } else {
                fault_throttle_mismatch_ab = true;
            }

            return false;
        }

        // plausability diff too high 
        else if ((calcThrottle2 - ThrottleMaxErrValue) > calcThrottle1) { //then throttle2 is too large compared to 1
            if (status == OK)
                Logger::error(POTACCELPEDAL, "throttle 2 too high (%i) compared to 1 (%i)", calcThrottle2, calcThrottle1);
            status = ERR_MISMATCH;
            faultHandler.raiseFault(POTACCELPEDAL, FAULT_THROTTLE_MISMATCH_AB, true);
            if (fault_throttle_mismatch_ab) {
                if (motorController->getOpState() == 2){
                    motorController->setOpState(4);
                }
            } else {
                fault_throttle_mismatch_ab = true;
            }
            return false;
        }
        else
        {
           faultHandler.cancelOngoingFault(POTACCELPEDAL, FAULT_THROTTLE_MISMATCH_AB);
            fault_throttle_mismatch_ab = false;
        }
        
    }

    // all checks passed -> throttle is ok
    if (status != OK)
        if (status != ERR_MISC) Logger::info(POTACCELPEDAL, (char *)Constants::normalOperation);

    if (motorController->getOpState() == 4 && motorController->getOpState() != 5){
        motorController->setOpState(2);
    }

    status = OK;
    return true;
}

/*
 * Convert the raw ADC values to a range from 0 to 1000 (per mille) according
 * to the specified range and the type of potentiometer.
 */
int16_t PotThrottle::calculatePedalPosition(RawSignalData *rawSignal) {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();
    uint16_t calcThrottle1, calcThrottle2;

    calcThrottle1 = normalizeAndConstrainInput(rawSignal->input1, config->minimumLevel1, config->maximumLevel1);
    // Logger::console("Calc throttle 1 %d", calcThrottle1);

    if (config->numberPotMeters > 1) {
        calcThrottle2 = normalizeAndConstrainInput(rawSignal->input2, config->minimumLevel2, config->maximumLevel2);
        // Logger::console("Calc throttle 2 %d", calcThrottle2);

        calcThrottle1 = (calcThrottle1 + calcThrottle2) / 2; // now the average of the two
    }
    // Logger::console("Final level value %d", calcThrottle1);
    return calcThrottle1;
}

/*
 * Return the device ID
 */
DeviceId PotThrottle::getId() {
    return (POTACCELPEDAL);
}

/*
 * Load the device configuration.
 * If possible values are read from EEPROM. If not, reasonable default values
 * are chosen and the configuration is overwritten in the EEPROM.
 */
void PotThrottle::loadConfiguration() {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();

    if (!config) { // as lowest sub-class make sure we have a config object
        config = new PotThrottleConfiguration();
        Logger::debug("loading configuration in throttle");
        setConfiguration(config);
    }

    Throttle::loadConfiguration(); // call parent

    //if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        Logger::debug(POTACCELPEDAL, Constants::validChecksum);
        prefsHandler->read("ThrottleMin1", (uint16_t *)&config->minimumLevel1, 0);

        //5 volts = 818 when max is 5000
        //5 volts = 1000 when max is 4090
        //910 dif = 18.2% dif
        prefsHandler->read("ThrottleMin1", (uint16_t *)&config->minimumLevel1, 600);
        prefsHandler->read("ThrottleMax1", (uint16_t *)&config->maximumLevel1, 3719);
        prefsHandler->read("ThrottleMin2", (uint16_t *)&config->minimumLevel2, 330);        
        prefsHandler->read("ThrottleMax2", (uint16_t *)&config->maximumLevel2, 1892);
        prefsHandler->read("NumThrottles", &config->numberPotMeters, 2);
        prefsHandler->read("ThrottleType", &config->throttleSubType, 1);

        prefsHandler->read("ADC1", &config->AdcPin1, 0);
        prefsHandler->read("ADC2", &config->AdcPin2, 5);

        // prefsHandler->read("ADC1", &config->AdcPin1, 0);
        // prefsHandler->read("ADC2", &config->AdcPin2, 1);

        // ** This is potentially a condition that is only met if you don't have the EEPROM hardware **
        // If preferences have never been set before, numThrottlePots and throttleSubType
        // will both be zero.  We really should refuse to operate in this condition and force
        // calibration, but for now at least allow calibration to work by setting numThrottlePots = 2
        if (config->numberPotMeters == 0 && config->throttleSubType == 0) {
            Logger::info(POTACCELPEDAL, "THROTTLE APPEARS TO NEED CALIBRATION/DETECTION - choose 'z' on the serial console menu");
            config->numberPotMeters = 2;
        }
    //}
    Logger::debug(POTACCELPEDAL, "# of pots: %d       subtype: %d", config->numberPotMeters, config->throttleSubType);
    Logger::debug(POTACCELPEDAL, "T1 MIN: %i MAX: %i      T2 MIN: %i MAX: %i", config->minimumLevel1, config->maximumLevel1, config->minimumLevel2,
                  config->maximumLevel2);
}

/*
 * Store the current configuration to EEPROM
 */
void PotThrottle::saveConfiguration() {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();

    Throttle::saveConfiguration(); // call parent

    prefsHandler->write("ThrottleMin1", (uint16_t)config->minimumLevel1);
    prefsHandler->write("ThrottleMax1", (uint16_t)config->maximumLevel1);
    prefsHandler->write("ThrottleMin2", (uint16_t)config->minimumLevel2);
    prefsHandler->write("ThrottleMax2", (uint16_t)config->maximumLevel2);
    prefsHandler->write("NumThrottles", config->numberPotMeters);
    prefsHandler->write("ThrottleType", config->throttleSubType);
    prefsHandler->write("ADC1", config->AdcPin1);
    prefsHandler->write("ADC2", config->AdcPin2);
    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}

String PotThrottle::describeThrottleType()
{
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();
    if (config->throttleSubType == 1) return String("Std Linear");
    if (config->throttleSubType == 2) return String("Inverse Linear");
    return String("Invalid Value!");
}
int16_t PotThrottle::getLevel()
{
    return calculatePedalPosition(acquireRawSignal());
}
//creation of a global variable here causes the driver to automatically register itself without external help
PotThrottle potThrottle;


