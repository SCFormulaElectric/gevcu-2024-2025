/*
 * CoolingController.h - implements a simple unified way to control fans, coolant pumps, heaters, etc that
 might be used in a car. 
 
 Copyright (c) 2021 Collin Kidder

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

#include "CoolingController.h"
#include "../../DeviceManager.h"

/*
 * Constructor
 */
CoolingController::CoolingController() : Device() {    
    commonName = "Cooling Controller";
    shortName = "CoolingController"; 

}

void CoolingController::earlyInit()
{
    prefsHandler = new PrefHandler(COOLCONTROL);
}

/*
 * Setup the device.
 */
void CoolingController::setup() {
    crashHandler.addBreadcrumb(ENCODE_BREAD("COOLING") + 0);
    tickHandler.detach(this); // unregister from TickHandler first
    setAttachedCANBus(1);
    attachedCANBus->attach(this, 0x200, 0x7f0, false);

    Logger::info("add device: CoolingController (id: %X, %X)", COOLCONTROL, this);

    loadConfiguration();

    Device::setup(); //call base class

    CoolingControllerConfiguration *config = (CoolingControllerConfiguration *)getConfiguration();

    StatusEntry stat;// Values that should not be changed to the outside world


    // stat = {"COOLING_AccumulatorFanOn", &isAccumulatorFanOn, CFG_ENTRY_VAR_TYPE::BYTE, 0, this};
    // deviceManager.addStatusEntry(stat);
    // stat = {"COOLING_AccumulatorPumpOn", &isAccumulatorPumpOn, CFG_ENTRY_VAR_TYPE::BYTE, 0, this};
    // deviceManager.addStatusEntry(stat);
    // stat = {"COOLING_MotorPummpOn", &isMotorPumpOn, CFG_ENTRY_VAR_TYPE::BYTE, 0, this};
    // deviceManager.addStatusEntry(stat);    
    // stat = {"COOLING_MotorFanOn", &isMotorFanOn, CFG_ENTRY_VAR_TYPE::BYTE, 0, this};
    // deviceManager.addStatusEntry(stat);    

    tickHandler.attach(this, CFG_TICK_INTERVAL_COOLCONTROL);

    // isAccumulatorPumpOn = false;
    // isMotorPumpOn = false;
    // isAccumulatorFanOn = false;
    // isMotorFanOn = false;

    // for flow sensor
    // pulseCount = 0;
    // lastTickTime = 0;
    // flowRate = 0;
    // threshold = 67; 
    // calibrationFactor = 4.5;
    // tickInterval = 1000;
    // lastDigitalInputState = false; 

    MAX_MOTOR_TEMP = 45;
    MAX_MOTOR_CTRL_TEMP = 70;
    motor_temp_percentage = 0;
    motor_ctrl_temp_percentage = 0;
    speed = 0;

}

double evaluateExpression(double x) {
    double exponent = -1.11 * pow(10, -4) * x;
    double result = 76.9 * exp(exponent);
    return result;
}

/*
 * Process a timer event. This is where you should be doing checks and updates. 
 */
void CoolingController::handleTick() {
    // setDigitalOutputPWM
    crashHandler.addBreadcrumb(ENCODE_BREAD("COOLING") + 1);
    Device::handleTick(); // Call parent which controls the workflow
    Logger::avalanche("Cooling Controller Tick Handler");

    CoolingControllerConfiguration *config = (CoolingControllerConfiguration *) getConfiguration();

    // Retrieve the temperature of the motor and the accumulator
    // int32_t motorTemperatureAnalogReading = systemIO.getAnalogIn(config->motorTemperatureSensorPin);
    // int32_t accumulatorTemperatureAnalogReading = systemIO.getAnalogIn(config->accumulatorTemperatureSensorPin);
    
    // double convertedVoltage = (motorTemperatureAnalogReading * (5.0 / 3071.0));
    // double division = (500000 / convertedVoltage) - 100000;
    // double result = evaluateExpression(division);
    // Logger::info(COOLCONTROL, "Temperature Reading in Celsius: %f", result);

    double max_temp_percent = max(motor_temp_percentage, motor_ctrl_temp_percentage);
    if(max_temp_percent >= 0.9){
         //duty cycle 90
         //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 90, 400);
    }
    else if(max_temp_percent >= 0.8){
        //duty cycle 80
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 80, 400);
    }
    else if(max_temp_percent>= 0.7){
        //duty cycle 70
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 70, 400);
    }
    else if(max_temp_percent>= 0.6){
        //duty cycle 60
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 60, 400);
    }

    if (speed < 5)
    {
        systemIO.setDigitalOutput(config->radiatorFanPin, false)
    }
    else{
        systemIO.setDigitalOutput(config->radiatorFanPin, true)
    }
    //  bool currentDigitalInputState = (bool) systemIO.getDigitalIn(config->flowSensorPin);
    // if (lastDigitalInputState == true && currentDigitalInputState == false) {
    //     // Falling edge detected (HIGH to LOW transition)
    //     pulseCount++;
    // }
    // lastDigitalInputState = currentDigitalInputState; // Update the last state

    // uint32_t currentTime = millis();
    // if (currentTime - lastTickTime >= (unsigned int)tickInterval) {
    //     calculateFlowRate();
    //     resetPulseCount();
    //     lastTickTime = currentTime;
    // }
}
/*
 * Return the device ID
 */
DeviceId CoolingController::getId() {
    return (COOLCONTROL);
}

DeviceType CoolingController::getType()
{
    return (DeviceType::DEVICE_CHARGER);
}

/*
 * Reset the pulse count.
 */
// void CoolingController::resetPulseCount() {
//     pulseCount = 0;
// }

/*
 * Calculate the flow rate.
 */
// void CoolingController::calculateFlowRate() {
//     if (pulseCount >= threshold) {
//         flowRate = (pulseCount / calibrationFactor) * (6000 / tickInterval);
//     } else {
//         flowRate = 0;
//     }
//     Logger::info(COOLCONTROL, "Flow Rate: %f L/min", flowRate);
// }


void CoolingController::handleCanFrame(const CAN_message_t &frame){
    u_int8_t payload = decode_hex(frame_buf[2], frame_buf[1]);
    switch(frame.buf[0]){
        case 0x49: //motor
            motor_temp_percentage = payload/MAX_MOTOR_TEMP;
        case 0x4a: //motor controller 
            motor_ctrl_temp_percentage = payload/MAX_MOTOR_TEMP;
        case 0x3D:
            speed = payload;
    }
}

int CoolingController::decode_hex(const int64_t first_half, const int64_t second_half) const{
    //second_half has 256 more weight since it is in the 2nd place of base 16, 16^2 = 256.
    return second_half * 256 + first_half;
}

/*
 * Load the device configuration.
 * If possible values are read from EEPROM. If not, reasonable default values
 * are chosen and the configuration is overwritten in the EEPROM.
 */
void CoolingController::loadConfiguration() {
    CoolingControllerConfiguration *config = (CoolingControllerConfiguration *) getConfiguration();

    if (!config) { // as lowest sub-class make sure we have a config object
        config = new CoolingControllerConfiguration();
        Logger::debug("loading configuration in CoolingController");
        setConfiguration(config);
    }
    
    Device::loadConfiguration(); // call parent

    //TODO Change pin number to pin for input
    // prefsHandler->read("motorTempeartureSensorPin", &config->motorTemperatureSensorPin, 0); // ANALOG0 PIN
    // prefsHandler->read("accumulatorTempeartureSensorPin", &config->accumulatorTemperatureSensorPin, 1); // ANALOG1 PIN
    // prefsHandler->read("fanAccumulatorPin", &config->fanAccumulatorPin, 255);
    // prefsHandler->read("fanMotorPin", &config->fanMotorPin, 255);
    // prefsHandler->read("waterAccumulatorPin", &config->waterAccumulatorPin, 255);
    prefsHandler->read("waterMotorPin", &config->waterMotorPin, 9);
    prefsHAndler->read("radiatorFanPin", &config->radiatorFanPin, 0);
    // prefsHandler->read("flowSensorPin", &config->flowSensorPin, 0);


    /*prefsHandler->read("motorPumpOnTemperature", &config->motorPumpOnTemperature, 0);
    prefsHandler->read("motorPumpOffTempearture", &config->motorPumpOffTempearture, 0);
    prefsHandler->read("accumulatorPumpOnTemperature", &config->accumulatorPumpOnTemperature, 0);
    prefsHandler->read("accumulatorPumpOffTemperature", &config->accumulatorPumpOffTemperature, 0);
    
    prefsHandler->read("motorFanOnTemperature", &config->motorFanOnTemperature, 0);
    prefsHandler->read("motorFanOffTemperature", &config->motorFanOffTemperature, 0);
    prefsHandler->read("accumulatorFanOnTemperature", &config->accumulatorFanOnTemperature, 0);
    prefsHandler->read("accumulatorFanOffTemperature", &config->accumulatorFanOffTemperature, 0);*/
}
/*
 * Store the current configuration to EEPROM
 */
void CoolingController::saveConfiguration() {
    CoolingControllerConfiguration *config = (CoolingControllerConfiguration *) getConfiguration();

    Device::saveConfiguration(); // call parent

    //TODO Change pin number to pin for input
    // prefsHandler->write("motorTempeartureSensorPin", config->motorTemperatureSensorPin);
    // prefsHandler->write("accumulatorTempeartureSensorPin", config->accumulatorTemperatureSensorPin);
    // prefsHandler->write("fanAccumulatorPin", config->fanAccumulatorPin);
    // prefsHandler->write("fanMotorPin", config->fanMotorPin);
    //prefsHandler->write("waterAccumulatorPin", config->waterAccumulatorPin);
    prefsHandler->write("waterMotorPin", config->waterMotorPin);
    prefsHAndler->write("radiatorFanPin", &config->radiatorFanPin);
    // prefsHandler->write("flowSensorPin", config->flowSensorPin);


   /*prefsHandler->write("motorPumpOnTemperature", config->motorPumpOnTemperature);
    prefsHandler->write("motorPumpOffTempearture", config->motorPumpOffTempearture);
    prefsHandler->write("accumulatorPumpOnTemperature", config->accumulatorPumpOnTemperature);
    prefsHandler->write("accumulatorPumpOffTemperature", config->accumulatorPumpOffTemperature);
    
    prefsHandler->write("motorFanOnTemperature", config->motorFanOnTemperature);
    prefsHandler->write("motorFanOffTemperature", config->motorFanOffTemperature);
    prefsHandler->write("accumulatorFanOnTemperature", config->accumulatorFanOnTemperature);
    prefsHandler->write("accumulatorFanOffTemperature", config->accumulatorFanOffTemperature);

    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();*/
}

CoolingController coolingController;
