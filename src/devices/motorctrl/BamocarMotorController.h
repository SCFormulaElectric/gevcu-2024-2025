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
 #define CFG_TICK_INTERVAL_MOTOR_CONTROLLER_BAMOCAR     40000
 
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
 };
 
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
 
 private:
     double motorControllerToCelsius(const uint16_t reading) const;
     double motorToCelsius(const uint16_t reading) const;
     uint16_t torqueCommand;
     void timestamp();
     CAN_message_t var; // delete this when handleTick is fully functional
     CAN_message_t freeRolling;
     int throttleAnalogValue;
     int mappedMotorTorque;
     bool enable_sent;
     bool disable_sent;
     int last_sent_value;
     
 };
 
 #endif /* DMOC_H_ */
 
 
 
 