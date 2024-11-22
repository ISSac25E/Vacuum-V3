// Arduino User Config

/*
  You may enter Tuya Pairing mode by first turning off the Vacuum Cleaner using the switch found underneath
  Keep the battery Plugged in as this is what powers the Arduino.

  Next, enter Pairing Mode by holding down the Vacuum Cleaners Main button until the LED will start pulsing quickly
  You may have to hold the button for up to 30 seconds

  To Switch between AP and SMART pairing mode, release the pairing button and Press and hold again until the LED begins to flash at a different pattern

  To test wether the arduino is operational, you can simply tap the main button once (when the vacuum cleaner is switched off) and the Blue LED will pulse briefly
*/

/////////////////////////////////
/////////// TUYA Setup //////////
/////////////////////////////////

#define TUYA_PID "*****" // << This can be found in your TUYA account
#define TUYA_MCU_Version "1.0.0"    // << This field does not really matter. It can be a useful tool to keep track of revisions

#define TUYA_ModuleResetPin 7 // << configure pin number for Arduino to reset the TUYA module in the case of mis-communication or module error

#define DPID_UpdateInterval 800 // << set individual DPID update interval in milliseconds (this will only run when necessary)

#define DPID_UpdateAllInterval 10000 // << set update ALL DPID's interval in milliseconds. Keep this as long as possible to prevent app crashing and stuttering

//////////////////////////////////
//////////// Pin Setup ///////////
//////////////////////////////////

// define all LED Pins in the Vacuum:
#define Vacuum_BlueLedPin A3
#define Vacuum_YellowLedPin A2
#define Vacuum_RedLedPin A1

// define Vacuum Button Pin. This pin will also be used to enter pairing mode
#define Vacuum_ButtonPin A0

// define 5v Power pin. This will be used to detect wether the vacuum is switched on or not.
#define Vacuum_PowerPin 11

// define Vacuum brush motor Pin. This will be used to determine wether the vacuum is Not Moving, Cleaning, or Going Home
#define Vacuum_BrushMotorPin 12

// define Battery Pin for measuring voltage:
#define Vacuum_BatteryPin A6

// define Vacuum IR Pin. This will be used to control the vacuum through the IR protocol(the pin will be directly connected to the Vacuum Cleaner's IR Pin):
#define Vacuum_IrPin 10

// define debug led pin. If debug led is not needed, simply comment out this line:
#define Vacuum_DebugLedPin 13

/////////////////////////////////
/////////// DPID Setup //////////
/////////////////////////////////

#define VacuumStatus_DPID 101  // << Status DPID, String Type. Updates Status Of vacuum Cleaner
#define VacuumControl_DPID 102 // << Control DPID, Enum Type. Used to control vacuum Cleaner
#define VacuumBattery_DPID 103 // << Battery Voltage DPID, Value Type. Used to measure battery voltage