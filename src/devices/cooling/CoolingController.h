/*
 * HeatCoolController.h - implements a simple unified way to control fans, coolant pumps, heaters, etc that
 might be used in a car. 
 *
 Copyright (c) 2021 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#ifndef HEATCOOL_H_
#define HEATCOOL_H_

#include <Arduino.h>
#include "../../config.h"
#include "../../sys_io.h"
#include "../../TickHandler.h"
#include "../../Logger.h"
#include "../../DeviceManager.h"
#include "../../FaultHandler.h"
#include "../../FaultCodes.h"

#define COOLCONTROL 0x3210
#define CFG_TICK_INTERVAL_COOLCONTROL     5'000'000 //5 seconds


class CoolingControllerConfiguration: public DeviceConfiguration {
public:
    //I/O pins
    // uint8_t fanAccumulatorPin;
    // uint8_t fanMotorPin;
    // uint8_t waterAccumulatorPin;
    uint8_t waterMotorPin;
    uint8_t radiatorFanPin;
    uint8_t accumulatorTemperatureSensorPin;
    uint8_t motorTemperatureSensorPin;
    // uint8_t flowSensorPin;

    // float motorPumpOnTemperature;
    // float motorPumpOffTempearture;
    // float accumulatorPumpOnTemperature;
    // float accumulatorPumpOffTemperature;


    // float motorFanOnTemperature;
    // float motorFanOffTemperature;
    // float accumulatorFanOnTemperature;
    // float accumulatorFanOffTemperature;
};

class CoolingController: public Device, CanObserver {
public:
    CoolingController();
    void setup();
    void earlyInit();
    void handleTick();
    DeviceId getId();
    DeviceType getType();

    void loadConfiguration();
    void saveConfiguration();
    void handleCanFrame(const CAN_message_t &frame);

    void resetPulseCount();
    void calculateFlowRate();
    double thermistorToCelsius(const double reading) const;

    uint16_t decode_hex(const uint8_t first_half, const uint8_t second_half) const;

    int32_t normalizeInput(int32_t input, int32_t min, int32_t max);


protected:

private:
    // bool isAccumulatorPumpOn;
    // bool isMotorPumpOn;
    // bool isAccumulatorFanOn;
    // bool isMotorFanOn;

    // int  pulseCount;
    // uint32_t lastTickTime;
    // float flowRate;
    // int threshold; 
    // float calibrationFactor;
    // uint32_t tickInterval;
    // bool lastDigitalInputState; 
    // PID constants
    #define PID_KP 1.0    // Proportional gain
    #define PID_KI 0.1    // Integral gain
    #define PID_KD 0.05   // Derivative gain

    // PID-related variables
    float prev_error = 0;  // Previous error for D term
    float integral = 0;    // Integral of the error
    float setpoint = 800;  // Desired temperature (e.g., 80%)
    //
    int16_t motorToCelsius(uint16_t reading) const;
    int16_t motorControllerToCelsius(uint16_t reading) const;
    int MAX_MOTOR_TEMP;
    int MAX_MOTOR_CTRL_TEMP;
    int speed;
    int16_t motor_temp_percentage;
    int16_t motor_ctrl_temp_percentage;
    MotorController* motorController;
};

#endif
