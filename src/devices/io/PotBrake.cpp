/*
 * PotBrake.cpp
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

#include "PotBrake.h"

/*
 * Constructor
 * Set which ADC channel to use
 */
PotBrake::PotBrake() : Throttle() {
    commonName = "Potentiometer (analog) brake";
    shortName = "PotBrake";
}

void PotBrake::earlyInit()
{
    prefsHandler = new PrefHandler(POTBRAKEPEDAL);
}

/*
 * Setup the device.
 */
void PotBrake::setup() {
    crashHandler.addBreadcrumb(ENCODE_BREAD("PTBRK") + 0);
    tickHandler.detach(this); // unregister from TickHandler first
    Logger::info("add device: PotBrake (id: %X, %X)", POTBRAKEPEDAL, this);
    
    loadConfiguration();
    PotBrakeConfiguration *config = (PotBrakeConfiguration *) getConfiguration();

    Throttle::setup(); //call base class

    ConfigEntry entry;
    entry = {"B1ADC", "Set brake ADC pin", &config->AdcPin1, CFG_ENTRY_VAR_TYPE::BYTE, 0, 255, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"B2ADC", "Set brake ADC pin", &config->AdcPin2, CFG_ENTRY_VAR_TYPE::BYTE, 0, 255, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"B1MN", "Set brake min value", &config->minimumLevel1, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"B1MX", "Set brake max value", &config->maximumLevel1, CFG_ENTRY_VAR_TYPE::UINT16, 0, 4096, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"BMINR", "Percent of full torque for start of brake regen", &config->minimumRegen, CFG_ENTRY_VAR_TYPE::BYTE, 0, 100, 0, nullptr};
    cfgEntries.push_back(entry);
    entry = {"BMAXR", "Percent of full torque for maximum brake regen", &config->maximumRegen, CFG_ENTRY_VAR_TYPE::BYTE, 0, 100, 0, nullptr};
    cfgEntries.push_back(entry);

    //set digital ports to inputs and pull them up all inputs currently active low
    //pinMode(THROTTLE_INPUT_BRAKELIGHT, INPUT_PULLUP); //Brake light switch

    
    tickHandler.attach(this, CFG_TICK_INTERVAL_POT_THROTTLE);
}

/*
 * Process a timer event.
 */
void PotBrake::handleTick() {
    crashHandler.addBreadcrumb(ENCODE_BREAD("PTBRK") + 1);
    Throttle::handleTick(); // Call parent which controls the workflow
}

/*
 * Retrieve raw input signals from the brake hardware.
 */
RawSignalData *PotBrake::acquireRawSignal() {
    PotBrakeConfiguration *config = (PotBrakeConfiguration *) getConfiguration();
    rawSignal.input1 = systemIO.getAnalogIn(config->AdcPin1);
    rawSignal.input2 = systemIO.getAnalogIn(config->AdcPin2);
    return &rawSignal;
}


/*
 * Perform sanity check on the ADC input values.
 */
// bool PotBrake::validateSignal(RawSignalData *rawSignal) {
//     PotBrakeConfiguration *config = (PotBrakeConfiguration *) getConfiguration();

//     if (rawSignal->input1 > (config->maximumLevel1 + CFG_BRAKE_TOLERANCE)) {
//         if (status == OK)
//             Logger::error(POTBRAKEPEDAL, (char *)Constants::valueOutOfRange, rawSignal->input1);
//         status = ERR_HIGH_T1;
//         return true; // even if it's too high, let it process and apply full regen !
//     }
//     if (rawSignal->input1 < (config->minimumLevel1 - CFG_BRAKE_TOLERANCE)) {
//         if (status == OK)
//             Logger::error(POTBRAKEPEDAL, (char *)Constants::valueOutOfRange, rawSignal->input1);
//         status = ERR_LOW_T1;
//         return false;
//     }

//     // all checks passed -> brake is OK
//     if (status != OK)
//         Logger::info(POTBRAKEPEDAL, (char *)Constants::normalOperation);
//     status = OK;
//     return true;
// }

bool PotBrake::validateSignal(RawSignalData *rawSignal) {
    PotThrottleConfiguration *config = (PotThrottleConfiguration *) getConfiguration();
    int32_t calcBrake1, calcBrake2;
    
    calcBrake1 = normalizeInput(rawSignal->input1, config->minimumLevel1, config->maximumLevel1 );
    Logger::console("Value is %d", calcBrake1);
    Logger::console("Value of 2 is %d", calcBrake2);

    if (calcBrake1 > (1000 + CFG_BRAKE_TOLERANCE)) // <-- Q, what is the break tolerance, define in .h
    {
        if (status == OK)
            Logger::error(POTBRAKEPEDAL, "ERR_HIGH_T1: brake 1 value out of range: %i", calcBrake1); //POTACCELPEDAL ?
        status = ERR_HIGH_T1;
        faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_BRAKE_HIGH, true); // Q, FAULT_BRAKE_HIGH might need to be expanded to include break 2, how do i choose a pin#?
        return false;
    }
    else
    {
        if (calcBrake1 > 1000) calcBrake1 = 1000;
        faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_BRAKE_HIGH);
    }

    if (calcBrake1 < (0 - CFG_BRAKE_TOLERANCE)) {
        if (status == OK)
            Logger::error(POTBRAKEPEDAL, "ERR_LOW_B1: brake 1 value out of range: %i ", calcBrake1);
        status = ERR_LOW_T1;
        faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_BRAKE_LOW, true);
        return false;
    }
    else
    {
        if (calcBrake1 < 0) calcBrake1 = 0;
        faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_BRAKE_LOW);
    }

    if (config->numberPotMeters > 1) {
        calcBrake2 = normalizeInput(rawSignal->input2, config->minimumLevel2, config->maximumLevel2);

        if (calcBrake2 > (1000 + CFG_BRAKE_TOLERANCE)) {
            if (status == OK)
                Logger::error(POTBRAKEPEDAL, "ERR_HIGH_T2: brake 2 value out of range: %i", calcBrake2);
            status = ERR_HIGH_T2;
            faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_BRAKE_HIGH, true);
            return false;
        }
        else
        {
            if (calcBrake2 > 1000) calcBrake2 = 1000;
            faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_BRAKE_HIGH);
        }

        if (calcBrake2 < (0 - CFG_BRAKE_TOLERANCE)) {
            if (status == OK)
                Logger::error(POTBRAKEPEDAL, "ERR_LOW_T2: brake 2 value out of range: %i", calcBrake2);
            status = ERR_LOW_T2;
            faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_BRAKE_LOW);
            return false;
        }
        else
        {
            if (calcBrake2 < 0) calcBrake2 = 0;
            faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_BRAKE_LOW);
        }

        if (config->throttleSubType == 2) {
            // inverted throttle 2 means the sum of the two throttles should be 1000
            if ( abs(1000 - calcBrake1 - calcBrake2) > ThrottleMaxErrValue) {
                if (status == OK)
                    Logger::error(POTBRAKEPEDAL, "Sum of brake 1 (%i) and brake 2 (%i) exceeds max variance from 1000 (%i) break values are different",
                                  calcBrake1, calcBrake2, ThrottleMaxErrValue);
                status = ERR_MISMATCH;
                faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_THROTTLE_MISMATCH_AB, true);
                return false;
            }
            else
            {
                faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_THROTTLE_MISMATCH_AB);
            }
        } else {
            if ((calcBrake1 - ThrottleMaxErrValue) > calcBrake2) { //then throttle1 is too large compared to 2
                if (status == OK)
                    Logger::error(POTBRAKEPEDAL, "brake 1 too high (%i) compared to 2 (%i)", calcBrake1, calcBrake2);
                status = ERR_MISMATCH;
                faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_THROTTLE_MISMATCH_AB, true);
                return false;
            }
            else if ((calcBrake2 - ThrottleMaxErrValue) > calcBrake1) { //then throttle2 is too large compared to 1
                if (status == OK)
                    Logger::error(POTBRAKEPEDAL, "brake 2 too high (%i) compared to 1 (%i)", calcBrake2, calcBrake1);
                status = ERR_MISMATCH;
                faultHandler.raiseFault(POTBRAKEPEDAL, FAULT_THROTTLE_MISMATCH_AB, true);
                return false;
            }
            else
            {
                faultHandler.cancelOngoingFault(POTBRAKEPEDAL, FAULT_THROTTLE_MISMATCH_AB);
            }
        }
    }

    // all checks passed -> throttle is ok
    if (status != OK)
        if (status != ERR_MISC) Logger::info(POTBRAKEPEDAL, (char *)Constants::normalOperation);
    status = OK;
    return true;
}

/*
 * Convert the raw ADC values to a range from 0 to 1000 (per mille) according
 * to the specified range and the type of potentiometer.
 */
int16_t PotBrake::calculatePedalPosition(RawSignalData *rawSignal) {
    PotBrakeConfiguration *config = (PotBrakeConfiguration *) getConfiguration();
    uint16_t calcBrake1, clampedLevel;

    if (config->maximumLevel1 == 0) //brake processing disabled if max is 0
        return 0;

    clampedLevel = constrain(rawSignal->input1, config->minimumLevel1, config->maximumLevel1);
    calcBrake1 = map(clampedLevel, config->minimumLevel1, config->maximumLevel1, (uint16_t) 0, (uint16_t) 1000);

    //This prevents flutter in the ADC readings of the brake from slamming regen on intermittently
    // just because the value fluttered a couple of numbers. This makes sure that we're actually
    // pushing the pedal. Without this even a small flutter at the brake will send minregen
    // out and ignore the accelerator. That'd be unpleasant.
    if (calcBrake1 < 15)
        calcBrake1 = 0;

    return calcBrake1;
}

/*
 * Overrides the standard implementation of throttle mapping as different rules apply to
 * brake based regen.
 */
int16_t PotBrake::mapPedalPosition(int16_t pedalPosition) {
    ThrottleConfiguration *config = (ThrottleConfiguration *) getConfiguration();
    int16_t brakeLevel, range;

    range = config->maximumRegen - config->minimumRegen;
    brakeLevel = -10 * range * pedalPosition / 1000;
    brakeLevel -= 10 * config->minimumRegen;
    Logger::avalanche(POTBRAKEPEDAL, "level: %d", brakeLevel);

    return brakeLevel;
}

/*
 * Return the device ID
 */
DeviceId PotBrake::getId() {
    return (POTBRAKEPEDAL);
}


int16_t PotBrake::getLevel()
{
    return calculatePedalPosition(acquireRawSignal());
}

/*
 * Return the device type
 */
DeviceType PotBrake::getType() {
    return (DEVICE_BRAKE);
}

/*
 * Load the device configuration.
 * If possible values are read from EEPROM. If not, reasonable default values
 * are chosen and the configuration is overwritten in the EEPROM.
 */
void PotBrake::loadConfiguration() {
    PotBrakeConfiguration *config = new PotBrakeConfiguration();
    setConfiguration(config);

    // we deliberately do not load config via parent class here !
    //818 is 1 volt
    //if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
        prefsHandler->read("BrakeMin", (uint16_t *)&config->minimumLevel1, 409);
        prefsHandler->read("BrakeMax", (uint16_t *)&config->maximumLevel1, 3681);
        prefsHandler->read("BrakeMin", (uint16_t *)&config->minimumLevel2, 409);
        prefsHandler->read("BrakeMax", (uint16_t *)&config->maximumLevel2, 3681);
        prefsHandler->read("BrakeADC1", &config->AdcPin1, 4);
        prefsHandler->read("BrakeADC2", &config->AdcPin2, 5);
        prefsHandler->read("numberPotMeters", (uint16_t *)&config->numberPotMeters, 2);
        
        Logger::debug(POTBRAKEPEDAL, "BRAKE MIN: %i MAX: %i", config->minimumLevel1, config->maximumLevel1);
        Logger::debug(POTBRAKEPEDAL, "Min: %i MaxRegen: %i", config->minimumRegen, config->maximumRegen);
}

/*
 * Store the current configuration to EEPROM
 */
void PotBrake::saveConfiguration() {
    PotBrakeConfiguration *config = (PotBrakeConfiguration *) getConfiguration();

    // we deliberately do not save config via parent class here !

    prefsHandler->write("BrakeMin", (uint16_t)config->minimumLevel1);
    prefsHandler->write("BrakeMax", (uint16_t)config->maximumLevel1);
    prefsHandler->write("BrakeMaxRegen", config->maximumRegen);
    prefsHandler->write("BrakeMinRegen", config->minimumRegen);
    prefsHandler->write("BrakeADC", config->AdcPin1);
    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}

DMAMEM PotBrake potBrake;