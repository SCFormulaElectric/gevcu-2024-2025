/*
 * BamocarMotorController.h
 */

#ifndef _BAMOCARINVERTER_H_
#define _BAMOCARINVERTER_H_

#include <Arduino.h>
#include "../../config.h"
#include "MotorController.h"
#include "../../sys_io.h"
#include "../../TickHandler.h"
#include "../../CanHandler.h"
#include "../misc/Statemachine.h"

#define BAMOCARINVERTER 0x1010
#define CFG_TICK_INTERVAL_MOTOR_CONTROLLER_BAMOCAR     10000 // 10 ms torque command rate; keep equal to CFG_TICK_INTERVAL_POT_THROTTLE

#define N_NOM 0x59
#define F_NOM 0x05
#define V_NOM 0x06
#define COS_PHI 0x0e
#define I_MAX 0x4d
#define I_NOM 0x4e
#define M_POLE 0x4f
#define BRAKE_DELAY 0xf1
#define M_TEMP 0xa3
#define FB_POLE 0xa7
#define FB_OFFSET 0x44
#define FB_INCR 0xa6
#define FACTOR_EXT 0x7e
#define AXIS 0xf8
#define DC_BUS 0xa5
#define REGEN_P 0x65
#define CUTOFF 0x1e
#define KP 0x1c
#define TI 0x1d
#define TIM_2 0x2b
#define XKP2 0xc9
#define KF 0xcb
#define RAMP 0x25
#define I_MAX_PK 0xc4
#define I_CON_EFF 0xc5
#define T_PEAK 0xf0
#define I_LIM_DIG 0x46
#define I_RED_N 0x3C
#define I_RED_TD 0x58
#define I_RED_TE 0x4c
#define I_RED_TM 0xa2
#define T_DC 0x07
#define V_DC 0x08
#define F_MIN 0x0b
#define V_CORNER 0x0c
#define F_CORNER 0X0d
#define KP_SPEED 0x2c
#define TI_SPEED 0x2d
#define TD_SPEED 0x2e
#define TIM_3 0x3b
/*
 * Class for DMOC specific configuration parameters
 */
class BamocarMotorControllerConfiguration : public MotorControllerConfiguration {
public:
    uint16_t limiterKp;   // Bamocar speed controller Kp (0x2C) used by the speed limiter
    uint16_t limiterTi;   // Bamocar speed controller Ti in ms (0x2D) used by the speed limiter
    uint16_t torqueAccRamp; // M R-Acc in ms (high half of 0x35); BAMOCAR_KEEP_INVERTER_RAMP = keep the inverter's value
    uint16_t torqueDecRamp; // M R-Dec in ms (high half of 0xED); BAMOCAR_KEEP_INVERTER_RAMP = keep the inverter's value
};

// Torque ramp setting value meaning "don't override, keep what the inverter has"
#define BAMOCAR_KEEP_INVERTER_RAMP 0xFFFF

// Speed ramps (N R-Acc / N R-Dec) written while the limiter is set up. With N-Lim < 100 % these
// ramps also cap acceleration in torque mode, so keep them short (NDrive manual 13.3).
#define BAMOCAR_LIMITER_SPEED_RAMP_MS 10

// Number of registers read back to confirm the inverter's settings (CONFIRM_REGS in the .cpp)
#define BAMOCAR_CONFIRM_REG_COUNT 9

class BamocarMotorController: public MotorController, CanObserver {
public:

public:
    virtual void handleTick();
    virtual void setup();
    void earlyInit();
    void setGear(Gears gear);

    BamocarMotorController();
    DeviceId getId();
    uint32_t getTickInterval();

    void handleCanFrame(const CAN_message_t &frame);

    virtual void loadConfiguration();
    virtual void saveConfiguration();
    void setOpState(OperationState op) override;
private:

    uint16_t torqueCommand;
    void timestamp();
    CAN_message_t var; // delete this when handleTick is fully functional
    CAN_message_t freeRolling;
    int throttleAnalogValue;
    bool enable_sent;
    bool disable_sent;
    int last_sent_value;
    int mappedMotorTorque;
    bool errorClear;
    uint32_t lastTempSendTime;
    int16_t motorTemperature;
    CAN_message_t motorTempMsg;
    CAN_message_t RequestDCMsg;

    // speed limiter (torque cruise control): read the registers once, then keep writing the setpoints
    void requestLimiterParams();
    void applySpeedLimiter();
    void writeReg16(uint8_t reg, uint16_t value);
    void writeReg32(uint8_t reg, uint32_t value);
    uint8_t limiterParamsLogged;   // bit i set once LIMITER_PARAMS[i] has been logged
    uint32_t lastLimiterRequest;
    uint16_t limiterNMax;          // N-100% (0xC8) in rpm, as read from the inverter
    uint16_t limiterTorqueAccRamp; // M R-Acc (high half of 0x35), kept when writing the speed ramp
    uint16_t limiterTorqueDecRamp; // M R-Dec (high half of 0xED), kept when writing the speed ramp
    uint32_t appliedNLim;          // N-Lim value last written (32767 = 100 % = limiter off)
    uint32_t lastNLimReadback;
    uint8_t lastLimiterActive;     // T-Nlim (0x40 bit 6) last seen; 0xFF = not seen yet
    uint32_t appliedTorqueRamps;   // (M R-Acc << 16) | M R-Dec last written; 0xFFFFFFFF = none yet
    uint16_t writtenKp;            // speed Kp (0x2C) last written
    uint16_t writtenTi;            // speed Ti (0x2D) last written

    // settings confirmation: registers read back every second, printed when they change
    void reportSettings();
    uint32_t confirmValue[BAMOCAR_CONFIRM_REG_COUNT];
    uint16_t confirmSeen;          // bit i set once CONFIRM_REGS[i] replied since the last request
    char lastSettingsReport[512];  // last report printed (two strings back to back)
};

#endif /* DMOC_H_ */



