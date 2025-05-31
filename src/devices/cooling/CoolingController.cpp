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
    motorController = deviceManager.getMotorController();
}

/*
    Convert resistance to temperature.
*/
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
    int32_t motorTemperatureAnalogReading = systemIO.getAnalogIn(config->motorTemperatureSensorPin);
    Logger::console("Print real Analogreading : %d", motorTemperatureAnalogReading);
    //int32_t accumulatorTemperatureAnalogReading = systemIO.getAnalogIn(config->accumulatorTemperatureSensorPin);
    double convertedVoltage = (motorTemperatureAnalogReading / 818.0);
    Logger::console("Voltage reading : %f", convertedVoltage);
    double before_radiator_resistance = (14666 * convertedVoltage) / (5 - convertedVoltage);
    Logger::console("resistance reading : %f", before_radiator_resistance);

    double temp_before_Radiator = thermistorToCelsius(before_radiator_resistance);
    Logger::info("Temperature before Radiator: %f", temp_before_Radiator);

    int32_t accumulatorTemperatureAnalogReading = systemIO.getAnalogIn(config->accumulatorTemperatureSensorPin);
    double convertedVoltageAfter = (accumulatorTemperatureAnalogReading / 818.0);
    Logger::console("Voltage reading : %f", convertedVoltageAfter);
    double after_radiator_resistance = (14666 * convertedVoltageAfter) / (5 - convertedVoltageAfter);
    Logger::console("resistance reading : %f", after_radiator_resistance);
    double temp_after_Radiator = thermistorToCelsius(after_radiator_resistance);
    Logger::info("Temperature after Radiator: %f", temp_after_Radiator);


    systemIO.setDigitalOutput(config->waterMotorPin,false);
    systemIO.setDigitalOutput(config->radiatorFanPin,false);

    
    //this entire thing was commented out
    int16_t max_temp_percent = max(motor_temp_percentage, motor_ctrl_temp_percentage);
    if(max_temp_percent >= 900){
         //duty cycle 90
         //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 75, 400);
        Logger::console("90 per");
    }
    else if(max_temp_percent >= 800){
        //duty cycle 80
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 70, 400);
        Logger::console("80 per");
    }
    else if(max_temp_percent>= 700){
        //duty cycle 70
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 60, 400);
        Logger::console("70 per");
    }
    else if(max_temp_percent>= 600){
        //duty cycle 60
        //TODO- check pin input
        systemIO.setDigitalOutput(config->waterMotorPin,true);
        systemIO.setDigitalOutputPWM(config->waterMotorPin, 50, 400);
        Logger::console("60 per");
    }

    // if (speed < 5)
    // {
    //     systemIO.setDigitalOutput(config->radiatorFanPin, false)
    // }
    // else{
    //     systemIO.setDigitalOutput(config->radiatorFanPin, true)
    // }
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


/*
 * Map the constrained level linearly to a signed value from 0 to 1000.
 */
int32_t CoolingController::normalizeInput(int32_t input, int32_t min, int32_t max) {
    return map(input, min, max, (int32_t) 0, (int32_t) 1000);
}

void CoolingController::handleCanFrame(const CAN_message_t &frame){
    u_int8_t payload = decode_hex(frame.buf[2], frame.buf[1]);
    switch(frame.buf[0]){
        case 0x49: //motor
            //this was commented out
            motor_temp_percentage = normalizeInput(payload, 0, MAX_MOTOR_TEMP);
        case 0x4a: //motor controller 
            //so was this
            motor_ctrl_temp_percentage = normalizeInput(payload, 0, MAX_MOTOR_CTRL_TEMP);
        case 0x3D:
            speed = payload;
    }
}

static const struct {
    double r_value;
    double temp;
} 
controllerTempLookup[] = {
    {332776, -40}, // -40°C, 332776 Ω
    {96481,  -20}, // -20°C,  96481 Ω
    {32566,    0}, //   0°C,  32566 Ω
    {12486,   20}, //  20°C,  12486 Ω
    {10000,   25}, //  25°C,  10000 Ω
    {5331,    40}, //  40°C,   5331 Ω
    {2490,    60}, //  60°C,   2490 Ω
    {1071,    85}, //  85°C,   1071 Ω
    {678,    100}, // 100°C,  678.1 Ω (truncated)
    {338,    120}  // 120°C,  338.2 ΩthermistorToCelsius (truncated)
};

double CoolingController::thermistorToCelsius(const double reading) const {
    for (int i = 1; i < (int)(sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0])); ++i) {
        if (reading >= controllerTempLookup[i].r_value) {
            double t1 = controllerTempLookup[i-1].temp;
            double t2 = controllerTempLookup[i].temp;
            double r1 = controllerTempLookup[i-1].r_value;
            double r2 = controllerTempLookup[i].r_value;

            double ratio = (reading - r2) / (r1 - r2);
            return t2 + ratio * (t1 - t2);
        }
    }
    return (double)controllerTempLookup[sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0]) - 1].temp;
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
    prefsHandler->read("motorTempeartureSensorPin", &config->motorTemperatureSensorPin, 4); // ANALOG0 PIN TEMP BEFORE RADIATOR
    prefsHandler->read("accumulatorTempeartureSensorPin", &config->accumulatorTemperatureSensorPin, 5); // ANALOG1 TEMP AFTER RADIATOR
    // prefsHandler->read("fanAccumulatorPin", &config->fanAccumulatorPin, 255);
    // prefsHandler->read("fanMotorPin", &config->fanMotorPin, 255);
    // prefsHandler->read("waterAccumulatorPin", &config->waterAccumulatorPin, 255);
    prefsHandler->read("waterMotorPin", &config->waterMotorPin, 0);
    prefsHandler->read("radiatorFanPin", &config->radiatorFanPin, 1);
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
    prefsHandler->write("motorTempeartureSensorPin", config->motorTemperatureSensorPin);
    prefsHandler->write("accumulatorTempeartureSensorPin", config->accumulatorTemperatureSensorPin);
    // prefsHandler->write("fanAccumulatorPin", config->fanAccumulatorPin);
    // prefsHandler->write("fanMotorPin", config->fanMotorPin);
    //prefsHandler->write("waterAccumulatorPin", config->waterAccumulatorPin);
    prefsHandler->write("waterMotorPin", config->waterMotorPin);
    prefsHandler->write("radiatorFanPin", config->radiatorFanPin); //removed 
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
