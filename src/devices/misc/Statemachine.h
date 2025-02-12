#ifndef Statemachine_H_
#define Statemachine_H_

#include <Arduino.h>
#include "../Device.h"
#include "../../DeviceManager.h"
#include "../../CanHandler.h"
#include "../io/PotBrake.h"



#define StatemachineID 0x103
// #define StatemachineTickInt 1000000
#define StatemachineTickInt 500000

enum State {
    S0,
    S1,
    S2
};

extern State extern_curr_state;

class StatemachineDevice: public Device, CanObserver{
public:
    StatemachineDevice();    // called nearly immediately to initialize your own variables
    void setup();            // called only if the device is actually enabled
    void earlyInit();        // called early and whether or not the device is enabled. Just used to setup configuration
    void handleTick();       // this is called every 1000000 cycles (i think needs checking)
    void handleCanFrame(const CAN_message_t &frame); // handle can frame
    DeviceId getId();
    DeviceType getType();

    // State getState();     // not needed because the state is no longer private, public  
    void updateState(State); // just a function to update state, looks better

    StatemachineDevice(PotBrake *brake);  // added

private:
    // State curr_state;        
    int8_t dash_send_flag;    // controls when to send message gevcu to dash 
    int8_t dash_val_msg;      // set when we recieve the check from car
    CAN_message_t buzz_msg;   // the constructed msg to activate buzzer_msg 
    uint32_t counter_timer;   // control when to send another buzzer message
                              // NOTE: there needs to be a check in the dash 
                              //       that it'll only buzz once when recieved for the first time
                              //       any more messages after should be ignored
    // int32_t brake1;           // this si probably need to change (ask tim)
    // int32_t brake2;           // this is probably need to change (ask tim)
    int16_t brake; 
    bool tsms;                // this is circuit is connected
    bool r2d;                 // digital signal ready to drive
    bool threshold_brake;     // if it's above a threshold (ask tim)

    PotBrake *potBrake; // Pointer to a PotBrake instance

    
};
#endif


// 11/13/24 - you need test if they can go through each of the states (test the din signals)
//          - then test with the feedback between dash and gevcu
//          - test the brake signal (combine with tim)
