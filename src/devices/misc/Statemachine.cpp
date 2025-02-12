#include "Statemachine.h"
#include "../io/PotBrake.h"


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
*/


State extern_curr_state = S0;  // Define and initialize the variable here


StatemachineDevice::StatemachineDevice(PotBrake *brake) : potBrake(brake) {}

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
    //Relevant BMS messages are 0x300 - 0x30F
    attachedCANBus->attach(this, 0x310, 0x000, false);
    tickHandler.attach(this, StatemachineTickInt);

    // set flags
    dash_send_flag = 1;
    dash_val_msg = 0;

    // Constructed message to dashboard
    buzz_msg.len = 2;
    buzz_msg.id = 0x109;
    buzz_msg.buf[0] = 0x1;
    buzz_msg.buf[1] = 2;

    /*
      buzz_msg[0] : a value to say hey buzz it up
      buzz_msg[1] : what was the prev state (either 1 or 2) 
                    you'll probably need to like change the 
                    code to save the previous state? or update the msg
                    right after, you can't take the current state cuz you 
                    don't know if it's from S1 -> S2 buzz or S2 -> S2 buzz
      buzz_msg[2] : idk a check sum (not needed honestly)
    */

    counter_timer = 0; 
}

/*For all multibyte integers the format is MSB first, LSB last
*/
void StatemachineDevice::handleCanFrame(const CAN_message_t &frame) {
    if(Logger::isDebug()){
        Logger::debug("Statemachine id=%X len=%X data=%X,%X,%X,%X,%X,%X,%X,%X",
                      frame.id, frame.len, 
                      frame.buf[0], frame.buf[1], frame.buf[2], frame.buf[3],
                      frame.buf[4], frame.buf[5], frame.buf[6], frame.buf[7]);
    }
    // change the id and the actual like contents of the CAN frame
    /*
      frame.buf[0] : value to turn on the 
      frame.buf[1] : what state the dash recieved is in 
      frame.buf[2] : idk a check sum (not needed honestly)
    */
    if(frame.id == 0x110){ 
      if(frame.buf[0] == 0x1 && frame.buf[1] == 0x2)
        dash_val_msg = 1; 
    }
}
DeviceId StatemachineDevice::getId() {
    return (StatemachineID);
}

DeviceType StatemachineDevice::getType() {
    return (DEVICE_MISC);
}

void StatemachineDevice::handleTick() {

/*
 *  read in the values
 */

  // brake1 = 40;
  // brake2 = 0;

  // brake = PotBrake.getLevel();
  int16_t brakeLevel = potBrake->getLevel(); // Get the brake level
  brake = 10;                           // change this until it's time to test the pressure sensor 

  tsms   = systemIO.getDigitalIn(4);
  r2d    = systemIO.getDigitalIn(5);
  tsms   = 1;                           // testing purposes
  r2d    = 1;                           // testing purposes

  if (brake < 20)                       // change the value above some threshold
  {
    threshold_brake = true;
  }
  threshold_brake = true;
  if (extern_curr_state == S0) {        // state 0, this is tested
    if(threshold_brake && tsms && r2d){
      updateState(S1);
       buzz_msg.buf[1] = 1; // set to the first time you send the rdy buzzer
    } else {
      updateState(S0);
    }
    Logger::console("I am in state S0");
    Logger::console("DIN4: %d, DIN5: %d", tsms, r2d);
    Logger::console("end \n ");

  } else if (extern_curr_state == S1) { // state 1
    if (dash_send_flag) {
      dash_send_flag = 0; 
      attachedCANBus->sendFrame(buzz_msg);
      Logger::console("I sent message\n");
    }
    if (tsms && threshold_brake) {
      updateState(S1);
      if (dash_val_msg) {
        updateState(S2);
      }
    } else {
      updateState(S0);
    }

    counter_timer++;
    Logger::console("counter: %d", counter_timer);
    if (counter_timer > 50){
      dash_send_flag = 1;
      counter_timer = 0;
       buzz_msg.buf[1] = 2; // if you have to resend the signal, in state 2
    }
    Logger::console(" I am in state S1\n");

    /*
      As long as the tsms && brake && r2d are all valid
      Then proceeed to S2, else we'll have to replay this again

      It assumes that you have the brakes depressed in state 2, you might
      beable to get rid of it

      Note: I might need a timer on the redundancy and count some cycles
      before returning to s0
    */

  } else if (extern_curr_state == S2) { // state 2
    if(tsms){
      updateState(S2);
    } else {
      updateState(S0);
    }
    Logger::console("\n I am in state S2");
  }
}


void checkBrakeLevel() { // help with chat to get the function over here
        if (potBrake) {
            int16_t level = potBrake->getLevel();
            Serial.println("Brake Level: " + String(level));
        }
    }

// testDevice test_device;
StatemachineDevice statemachine_device;
