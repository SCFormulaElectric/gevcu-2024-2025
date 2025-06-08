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
    attachedCANBus->attach(this, 0x181, 0xFFF, false);

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
    MAX_MOTOR_TEMP = 45;
    MAX_MOTOR_CTRL_TEMP = 70;
    motor_temp_percentage = 0;
    motor_ctrl_temp_percentage = 0;
    speed = 0;
    motorController = deviceManager.getMotorController();

    //Since the pump and fan are active low they should be set to false to turn on 
    systemIO.setDigitalOutput(config->waterMotorPin,false);
    systemIO.setDigitalOutput(config->radiatorFanPin,false);
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
    // Logger::console("Print real Analogreading : %d", motorTemperatureAnalogReading);
    //int32_t accumulatorTemperatureAnalogReading = systemIO.getAnalogIn(config->accumulatorTemperatureSensorPin);
    double convertedVoltage = (motorTemperatureAnalogReading / 818.0);
    // Logger::console("Voltage reading : %f", convertedVoltage);
    double before_radiator_resistance = (14666 * convertedVoltage) / (5 - convertedVoltage);
    // Logger::console("resistance reading : %f", before_radiator_resistance);

    double temp_before_Radiator = thermistorToCelsius(before_radiator_resistance);
    Logger::info("Temperature before Radiator: %f", temp_before_Radiator);

    int32_t accumulatorTemperatureAnalogReading = systemIO.getAnalogIn(config->accumulatorTemperatureSensorPin);
    double convertedVoltageAfter = (accumulatorTemperatureAnalogReading / 818.0);
    // Logger::console("Voltage reading : %f", convertedVoltageAfter);
    double after_radiator_resistance = (14666 * convertedVoltageAfter) / (5 - convertedVoltageAfter);
    // Logger::console("resistance reading : %f", after_radiator_resistance);
    double temp_after_Radiator = thermistorToCelsius(after_radiator_resistance);
    Logger::info("Temperature after Radiator: %f", temp_after_Radiator);


    // This chunk of code is commented out because we are not doing dynamic cooling,
    // in the future, if you want to do dynamic cooling, add the specific logic here
    int16_t max_temp_percent = max(motor_temp_percentage, motor_ctrl_temp_percentage);
    if (max_temp_percent >= 1000){
        // motorController->setOpState(1); // disable motor if the temps are greater than 100%. ISSUE ! ! ! ! ! !. This will intefere with the throttle plausibilty stuff
                                            // need a better way to raise faults for the motor but for now since we arent doing anything for that we are chilling.
    }
    else{
        // motorController->setOpState(2); // 
        if(max_temp_percent >= 900){
         //duty cycle 90
        // systemIO.setDigitalOutput(config->waterMotorPin,true);
        // systemIO.setDigitalOutputPWM(config->waterMotorPin, 75, 400);
        }
        else if(max_temp_percent >= 800){
            //duty cycle 80
            // systemIO.setDigitalOutput(config->waterMotorPin,true);
            // systemIO.setDigitalOutputPWM(config->waterMotorPin, 70, 400);
        }
        else if(max_temp_percent>= 700){
            //duty cycle 70
            // systemIO.setDigitalOutput(config->waterMotorPin,true);
            // systemIO.setDigitalOutputPWM(config->waterMotorPin, 60, 400);
        }
        else if(max_temp_percent>= 600){
            //duty cycle 60
            // systemIO.setDigitalOutput(config->waterMotorPin,true);
            // systemIO.setDigitalOutputPWM(config->waterMotorPin, 50, 400);
        }
    }
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
 * Map the constrained level linearly to a signed value from 0 to 1000.
 */
int32_t CoolingController::normalizeInput(int32_t input, int32_t min, int32_t max) {
    return map(input, min, max, (int32_t) 0, (int32_t) 1000);
}

void CoolingController::handleCanFrame(const CAN_message_t &frame){
    uint16_t payload = decode_hex(frame.buf[1], frame.buf[2]);
    switch(frame.buf[0]){
        case 0x49: //motor
            motor_temp_percentage = normalizeInput(motorToCelsius(payload), 0, MAX_MOTOR_TEMP);
        case 0x4a: //motor controller 
            motor_ctrl_temp_percentage = normalizeInput(motorControllerToCelsius(payload), 0, MAX_MOTOR_CTRL_TEMP);
    }
}


// LUT for motor temperature
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

/************************************************
  motorToCelsius: 
    Function to convert motor temp to celsius
  Args: 
    reading (uint_16_t): first parameter
      raw hex bytes of the motor temperature msg
  Returns:
    double
************************************************/
int16_t CoolingController::motorToCelsius(uint16_t reading) const{
    for (int i = 1; i < (int)(sizeof(motorTempLookup) / sizeof(motorTempLookup[0])); ++i) {
        if (reading <= motorTempLookup[i].value) {
            double t1 = motorTempLookup[i-1].tempC;
            double t2 = motorTempLookup[i].tempC;
            double v1 = motorTempLookup[i-1].value;
            double v2 = motorTempLookup[i].value;

            double ratio = (reading - v1) / (v2 - v1);
            double result = t1 + ratio * (t2 - t1);
            return (int)result; // Truncate fractional part
        }
    }
    return (int)motorTempLookup[sizeof(motorTempLookup) / sizeof(motorTempLookup[0]) - 1].tempC;
}

// LUT for motor controller temperature 
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

/************************************************
  motorControllerToCelsius: 
    Function to convert motor controller temp to celsius
  Args: 
    reading (uint_16_t): first parameter
      raw hex bytes of the motor controller temperature msg
  Returns:
    double
************************************************/
int16_t CoolingController::motorControllerToCelsius(uint16_t reading) const {
    for (int i = 1; i < (int)(sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0])); ++i) {
        if (reading >= controllerTempLookup[i].value) {
            double t1 = controllerTempLookup[i-1].tempC;
            double t2 = controllerTempLookup[i].tempC;
            double v1 = controllerTempLookup[i-1].value;
            double v2 = controllerTempLookup[i].value;

            double ratio = (reading - v2) / (v1 - v2);
            double result = t2 + ratio * (t1 - t2);
            return (int)result; // Truncate fractional part
        }
    }
    return (int)controllerTempLookup[sizeof(controllerTempLookup) / sizeof(controllerTempLookup[0]) - 1].tempC;
}


static const struct {
    double r_value;
    double temp;
} 
thermistorTempLookup[] = {
    {332776, -40},
    {96481,  -20}, 
    {32566,    0}, 
    {12486,   20}, 
    {10000,   25}, 
    {5331,    40}, 
    {2490,    60}, 
    {1071,    85}, 
    {678,    100}, 
    {338,    120}  
};

double CoolingController::thermistorToCelsius(const double reading) const {
    for (int i = 1; i < (int)(sizeof(thermistorTempLookup) / sizeof(thermistorTempLookup[0])); ++i) {
        if (reading >= thermistorTempLookup[i].r_value) {
            double t1 = thermistorTempLookup[i-1].temp;
            double t2 = thermistorTempLookup[i].temp;
            double r1 = thermistorTempLookup[i-1].r_value;
            double r2 = thermistorTempLookup[i].r_value;

            double ratio = (reading - r2) / (r1 - r2);
            return t2 + ratio * (t1 - t2);
        }
    }
    return (double)thermistorTempLookup[sizeof(thermistorTempLookup) / sizeof(thermistorTempLookup[0]) - 1].temp;
}

uint16_t CoolingController::decode_hex(const uint8_t first_half, const uint8_t second_half) const{
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

    prefsHandler->read("motorTempeartureSensorPin", &config->motorTemperatureSensorPin, 4); 
    prefsHandler->read("accumulatorTempeartureSensorPin", &config->accumulatorTemperatureSensorPin, 5);
    prefsHandler->read("waterMotorPin", &config->waterMotorPin, 0);
    prefsHandler->read("radiatorFanPin", &config->radiatorFanPin, 1);
}
/*
 * Store the current configuration to EEPROM
 */
void CoolingController::saveConfiguration() {
    CoolingControllerConfiguration *config = (CoolingControllerConfiguration *) getConfiguration();

    Device::saveConfiguration(); // call parent
    prefsHandler->write("motorTempeartureSensorPin", config->motorTemperatureSensorPin);
    prefsHandler->write("accumulatorTempeartureSensorPin", config->accumulatorTemperatureSensorPin);
    prefsHandler->write("waterMotorPin", config->waterMotorPin);
    prefsHandler->write("radiatorFanPin", config->radiatorFanPin); 

    prefsHandler->saveChecksum();
    prefsHandler->forceCacheWrite();
}

CoolingController coolingController;
