#include "Statemachine.h"
#include "../io/PotBrake.h"
#include "BmsClearer.h"

/*
  TODO: 
    - Send message to the dash and wait for a message check.

    11/13/24 
    - this is implemented on the gevcu side (not tested)      
    - someone is doing the  
    - Maybe on the dash end I'll send the state that it's currently
      if it's it was in state 2 then it changed to state 1, then you
      need to buzz the sound again, but when it's still in state 2 and
      it still hasn't recieved the confirmation message then you don't need to
      respond to the play the button msg

    11/13/24 
    - this is being implemented

    11/20/24 
    - the state machine flow is tested. now i just need to sync with tim on the
      brake stuff and use the real teensy 
    11/25/24
    - changed to pot brake value and merged with tim's code 
    - we have the brake pressure sensors, but we can't test it without the fluid
    - so we have to wait till next year

    02/25/25
    - need to test the brake pressure sensor 
    - test for latter 
    - we have 2 brake pressure sensors, do we cross then or does only one of them actually do any thing
    - brake in 2 weeks after the frame is painted 
    - technically they should be within a few from each other 
    - NOTE: the canbus on the gevcu is not working 
*/


State extern_curr_state = S0;  // Define and initialize the variable here


// StatemachineDevice::StatemachineDevice(PotBrake *brake){
//   commonName = "Statemachine";
//   shortName = "SM";
//   potBrake = brake;
// }

StatemachineDevice::StatemachineDevice():Device() {
    commonName = "Statemachine";
    shortName = "SM";
}

// State StatemachineDevice::getState() { return extern_curr_state; }

void StatemachineDevice::updateState(State x){extern_curr_state = x;}

void StatemachineDevice::earlyInit()
{
    prefsHandler = new PrefHandler(StatemachineID);
}

void StatemachineDevice::setup() {
    tickHandler.detach(this);
    Logger::info("add device: StatemachineDevice (id: %X, %X)", StatemachineID, this);

    Device::setup(); // run the parent class version of this function

    setAttachedCANBus(0);
    attachedCANBus->attach(this, 0x110, 0x00, false);
    tickHandler.attach(this, StatemachineTickInt);
    // set flags
    dash_send_flag = 1;
    dash_val_msg = 0;
    button_val_msg = 0;
    btnPress = 0;
    imdFault_latch = 0;
    tsms = 0;
    soundFlag = 0;
    clearBmsonStartFlag = 0;
    redlighttoggleflag = 0;
    current_value = 0;
    SOC_value = 0;


    // Constructed message to dashboard
    buzz_msg.len = 2;
    buzz_msg.id = 0x109;
    buzz_msg.buf[0] = 0x1;
    buzz_msg.buf[1] = 0x02;

    extern_curr_state = S0; // set the state to S0 on start up
    //dash faults via can (imd/bms)
    imd_msg.len = 1;
    imd_msg.id = 0x468;
    imd_msg.buf[0] = 0x00;
    bms_msg.len = 1;
    bms_msg.id = 0x469;
    bms_msg.buf[0] = 0x00;

    clear_bms_msg.len = 8;
    clear_bms_msg.id = 0x7e3;
    clear_bms_msg.buf[0] = 0x01;
    clear_bms_msg.buf[1] = 0x04;
    clear_bms_msg.buf[2] = 0x00;
    clear_bms_msg.buf[3] = 0x00;
    clear_bms_msg.buf[4] = 0x00;
    clear_bms_msg.buf[5] = 0x00;
    clear_bms_msg.buf[6] = 0x00;
    clear_bms_msg.buf[7] = 0x00;

    //redlight blink
    redlight_msg.len = 1;
    redlight_msg.id = 0x470;
    redlight_msg.buf[0] = 0x00;
    motecbuzztrg_msg.len = 1;
    motecbuzztrg_msg.id = 0x471;
    motecbuzztrg_msg.buf[0] = 0x00;
    
    triggerBSPDfault_msg.len = 1;
    triggerBSPDfault_msg.id = 0x471;

    R2D_msg.len = 2;
    R2D_msg.id = 0x475;
    R2D_msg.buf[0] = 0x01;
    imdFaultBlind = millis();
    
    CUR_msg.id = 0x799;
    CUR_msg.len = 2;

    SOC_msg.id = 0x798;
    SOC_msg.len = 2;


    

    /*
      buzz_msg[0] : a value to say hey buzz it up
      buzz_msg[1] : what was the prev state (either 1 or 2) 
                    you'll probably need to like change the 
                    code to save the previous state? or update the msg
                    right after, you can't take the current state cuz you 
                    don't know if it's from S1 -> S2 buzz or S2 -> S2 buzz
      buzz_msg[2] : idk a check sum (not needed honestly)
    */

}

/*For all multibyte integers the format is MSB first, LSB last
*/
void StatemachineDevice::handleCanFrame(const CAN_message_t &frame) {
    if(Logger::isDebug()){
        // Logger::debug("Statemachine id=%X", frame.id);/* len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                      // frame.id, frame.len, 
                      // frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                      // frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);*/
    }
    // change the id and the actual like contents of the CAN frame
    /*
      frame.buf[0] : value to turn on the 
      frame.buf[1] : what state the dash recieved is in 
      frame.buf[2] : idk a check sum (not needed honestly)
    */
   
        if(frame.id == 0x777){ 
          Logger::console("0x777 val: %u", frame.buf[0]);
        button_val_msg = 1; 
        attachedCANBus->sendFrame(clear_bms_msg);
    }
      else{
        button_val_msg = 0;
      }
      if (frame.id == 0x301){
        current_value = (frame.buf[0] << 8) | frame.buf[1];
        CUR_msg.buf[0] = frame.buf[0];
        CUR_msg.buf[1] = frame.buf[1];
        attachedCANBus->sendFrame(CUR_msg);
        Logger::console("Current val: %u", current_value);
      }

      if (frame.id== 0x302){
        SOC_value = (frame.buf[0] <<8) | frame.buf[1];
        SOC_msg.buf[0] = frame.buf[0];
        SOC_msg.buf[1] = frame.buf[1];
        attachedCANBus->sendFrame(SOC_msg);
        Logger::console("SOC: %u", SOC_value);
      }
//      if (frame.id == 0x303) {
//    bms_fault_register = ((uint32_t)frame.buf[0] << 24) | ((uint32_t)frame.buf[1] << 16)
//                       | ((uint32_t)frame.buf[2] << 8)  |  (uint32_t)frame.buf[3];
//    if (bms_fault_register & 0x01000000) {
//        Logger::error("BMS P0A1F: Internal Cell Communication Fault");
//    }
//    if (bms_fault_register == 0x01000000) {
//    // P0A1F is the only active fault
//    //attachedCANBus->sendFrame(clear_bms_msg);
//}
//}
    
}
DeviceId StatemachineDevice::getId() {
    return (StatemachineID);
}

DeviceType StatemachineDevice::getType() {
    return (DEVICE_MISC);
}

void StatemachineDevice::handleTick() {
  //attachedCANBus->sendFrame(clear_bms_msg);

/*
 *  read in the values
 */

  tsms       = systemIO.getDigitalIn(2);      // i think this is equivalent to the shutdown
  
  r2d        = systemIO.getDigitalIn(4);      // tested analogs austin 6/20 CAN
  brake1        = systemIO.getAnalogIn(0);      // tested analogs austin 6/20
  brake2        = systemIO.getAnalogIn(1);      // tested analogs austin 6/20
  LV_SOC        = systemIO.getAnalogIn(4);
  //Logger::console("brake 1 val: %u", brake1);
  //Logger::console("brake 2 val: %u", brake2);
  //Logger::console("soc val: %u", LV_SOC);

  if (millis() - imdFaultBlind > 30000){
      fault_imd       = systemIO.getDigitalIn(0); //um for us its inverted.
  }
  else{
    fault_imd = 1;
  }

  fault_bms       = systemIO.getDigitalIn(1);
  //fault_bms = bms_fault_delayed;
  //Logger::console("fault_imd val: %u", fault_imd);
  //Logger::console("fault_bms val: %u", fault_bms);
  
  
  if (!fault_imd){ //debounce imdFault. If it persists for 2 iterations (200ms) set FaultLatch
    faultCounter1++;
  }

      if (faultCounter1 > 1){ //thutapea: sloppy way to include BMS fault in TSSI state
      imdFault_latch = 1;
    }
  //if (millis() - testTime01 > 4000){
  //  tsms ^= 1;
  //  testTime01 = millis();
  //}
  if (tsms){
    //Logger::console("tsms");
    if (imdFault_latch == 0){
      //Logger::console("sanity check\n");
      triggerBSPDfault_msg.buf[0] = 0x01;
      attachedCANBus->sendFrame(triggerBSPDfault_msg);
      //Logger::console("light should be green \n");
    }
  }
  else{
    triggerBSPDfault_msg.buf[0] = 0x00;
    //attachedCANBus->sendFrame(triggerBSPDfault_msg);
  }

  if (imdFault_latch || fault_bms){

      if (millis() - lastredlightTime > 250){
        if (redlighttoggleflag == 1){
            redlight_msg.buf[0] ^= 0x01;
            redlighttoggleflag = 0;
            //Logger::console("sanity\n");
        }
        else {
          redlighttoggleflag = 1;
          redlight_msg.buf[0] ^= 0x00;
          //Logger::console(" check\n");
        }
      attachedCANBus->sendFrame(redlight_msg);
      lastredlightTime = millis();
      }

      else{
        redlight_msg.buf[0] = 0x00;
      //attachedCANBus->sendFrame(redlight_msg); 

      triggerBSPDfault_msg.buf[0] = 0x00;
      attachedCANBus->sendFrame(triggerBSPDfault_msg);
      }
  }

  /*if (tsms != 1){
    motecbuzztrg_msg.buf[0] = 1;
    attachedCANBus->sendFrame(motecbuzztrg_msg);
  }
  else{
    motecbuzztrg_msg.buf[0] = 0;
    attachedCANBus->sendFrame(motecbuzztrg_msg);
  }
    */
  if (button_val_msg){
      Logger::console("RECEIVED\n");
      btnPress = 1;
  }
  
  // tsms  = 1;                                // testing purposes
  //r2d  = 1;                                // testing purposes
  if (clearBmsonStartFlag == 0){
    clearBmsonStartFlag = 1;
    for (int i = 0; i <3; i++){
      attachedCANBus->sendFrame(clear_bms_msg);
      Logger::console("CLEARED BMS FROM STATEMACHINE ON START");
    }

    
  }
  //attachedCANBus->sendFrame(clear_bms_msg);

  if (fault_bms == 0){
    imd_msg.buf[0] = 2;
    Logger::console("fault_bms is 0");
  }
  else{
    imd_msg.buf[0] = 0;
    attachedCANBus->sendFrame(imd_msg);
  }
  if (imdFault_latch == 0){
    //Logger::console("I sent message\n");
    bms_msg.buf[0] = 2;
  }
  else{
    bms_msg.buf[0] = 0;
    attachedCANBus->sendFrame(bms_msg);
  }
  if (brake1 +  brake2 > 1000) { threshold_brake = true;}
  else {  threshold_brake = false; }

  if (extern_curr_state != S2 && threshold_brake && btnPress && tsms && imdFault_latch == 0){
     updateState(S2);
     btnPress = 0;
     button_val_msg = 0;
     soundFlag = 1;
     R2D_msg.buf[0] = 0x01;
     attachedCANBus->sendFrame(R2D_msg);
     soundTime = millis();
    }
    if (extern_curr_state != S2){ //might be able to remove this.
    btnPress = 0;
    button_val_msg = 0;
  }

    if (millis() -soundTime > 1500){
      R2D_msg.buf[0] = 0x00;
      attachedCANBus->sendFrame(R2D_msg);
    }
     
  
  if (extern_curr_state == S2) { // state 2
    if(!tsms || imdFault_latch){
      updateState(S0);
      btnPress = 0;
    }
    Logger::console("\n I am in state S2");
  }
}


int16_t StatemachineDevice::checkBrakeLevel() { // help with chat to get the function over here
        int16_t brake = -1; 
        if (potBrake) {
            int16_t brake = potBrake->getLevel();
            // Serial.println("Brake Level: " + String(level));
        }

        return brake;
    }
  

void StatemachineDevice::loadConfiguration() {
  Device::loadConfiguration();
}

 /*
  * Store the current configuration to EEPROM
  */
void StatemachineDevice::saveConfiguration() {
  Device::loadConfiguration();
}

// testDevice test_device;
StatemachineDevice statemachine_device;
