#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinPort\PinPort_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\SequenceBuild\SequenceBuild_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\LedMacro\LedMacro_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinDriver\PinDriver_1.0.1.h"

#define TUYA_PID "xfakot50jmkfgkcy" // << This can be found in your TUYA account
#define TUYA_MCU_Version "1.0.0"    // << This version does not really matter. You can put anything you like. It can be a useful tool to keep track of revisions

#define TUYA_WiFi_ModeSetPin 3 // << configure pin number for the button you'll use to Pair your TUYA Device

#define TUYA_ModuleResetPin 7 // << configure pin number for Arduino to reset the TUYA module in the case of mis-communication or module error

#define StatusLedPin 5 // << pin Number of status LED. Make sure pin is a 'PWM' and NOT FROM TIMER 2 (5, 6, 9, or 10. NOT 3 or 11)

#define DPID_UpdateInterval 1000 // << set update DPID's interval in milliseconds

#define DPID_VacuumStatus 101
#define DPID_VacuumControl 102
#define DPID_VacuumBattery 103

/////////////////////////////////
////////// LED Setup: //////////
/////////////////////////////////
// setup sequence builder for LED:
SequenceBuild ledBuild;

// setup macro for led AND servo (total 2):
LedMacro _macro[1];
LedMacroManager macro(_macro, 1);

// PWM led value:
uint8_t ledVal = 0;

////////////////////////////////////
////////// Button Setup: //////////
////////////////////////////////////
// setup pinDriver for brownout handling and debouncing:
PinDriver pinInput(TUYA_WiFi_ModeSetPin);
InputMacro pinMacro(HIGH);

//////////////////////////////////
////////// TUYA Setup: //////////
//////////////////////////////////
// config Tuya Module communication with Hardware Serial:
TuyaWifi tuya_module;

// setup DPID array to pass onto 'TuyaWifi':
unsigned char dpid_array[3][2] = {{DPID_VacuumStatus, DP_TYPE_STRING},
                                  {DPID_VacuumControl, DP_TYPE_ENUM},
                                  {DPID_VacuumBattery, DP_TYPE_VALUE}}; // there are two DPID's per each stepper (position_DPID and command_DPID) + one DPID for the servo

// setup module reset pin:
PinPort moduleResetPin(TUYA_ModuleResetPin);

uint32_t updateAll_Timer = 0; // timer to handle periodic updating of DPID's

const unsigned char s1[] = {"Ready"};
const unsigned char s3[] = {"Charging"};
const unsigned char s2[] = {"Cleaning"};


const unsigned char *s[3] = {s1, s2, s3};
int slen[3] = {sizeof(s1), sizeof(s2), sizeof(s3)};

uint8_t sp = 0;

void setup()
{
  // initialize hardware Serial to 9600 for communication with TUYA Module:
  Serial.begin(9600);

  // setup pin module reset pin, stepper direction and step pin, led output pin, and servo pin:
  {
    // set module reset pin to Input so user can reset Module too without damaging Arduino:
    moduleResetPin.set(INPUT);
    moduleResetPin.write(LOW);

    // LED pin setup:
    pinMode(StatusLedPin, OUTPUT);
  }

  // set up TUYA Module:
  {
    unsigned char pid[] = {TUYA_PID};
    unsigned char mcu_ver[] = {TUYA_MCU_Version};

    // init TUYA PID and MCU Version:
    tuya_module.init(pid, mcu_ver);

    // Set all TUYA DPID's:
    tuya_module.set_dp_cmd_total(dpid_array, 3);

    // set callbacks:
    tuya_module.dp_process_func_register(dp_process);       // function to proccess incomming commands
    tuya_module.dp_update_all_func_register(dp_update_all); // function to update all DPID states at once
  }

  // set initial LED sequence:
  ledBuild.setPrioritySequence(init_led, 0, true);
}

void loop()
{
  // run tuya uart service:
  tuya_module.uart_service();

  // run all handles:
  outputHandle();
  buttonHandle();
  moduleErrorHandle();

  // check update all timer:
  if (millis() - updateAll_Timer >= DPID_UpdateInterval)
    dp_update_all();
}

/*
  outputHandle():
    handles LED sequenses and servo sequences
*/
inline void outputHandle()
{
  // run all necessary handles first:
  ledBuild.run();
  macro.run();

  // write to output pins:
  analogWrite(StatusLedPin, ledVal);

  // determine led sequence:
  if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
    ledBuild.setSequence(ap_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
    ledBuild.setSequence(smart_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR || tuya_module.mcu_get_wifi_work_state() == WIFI_STATE_UNKNOWN)
    ledBuild.setSequence(error_led, 0, true);
  else
    ledBuild.setSequence(idle_led, 0, true);
}

/*
  buttonHandle():
    handle WiFi Mode button, sets TUYA Module to Pairing mode
*/
inline void buttonHandle()
{
  /*
    WiFi_Config:
      0 = don't set WiFi
      1 = Smart_Mode
      2 = AP_Mode
  */
  static uint8_t WiFi_Config = 0;
  static uint32_t WiFi_setTimer = 0;
  const uint16_t WiFi_setInterval = 20000;

  // check for pin state change:
  if (pinMacro(pinInput))
  {
    if (pinMacro) // button released
    {
      WiFi_Config = 0; // stop setting up WiFi
      if (pinMacro.prevInterval() <= 1000)
      {
        sp = (++sp % 3);
      }
    }
    else // button pressed
    {
      WiFi_setTimer = (millis() - WiFi_setInterval); // reset wifi send timer so it sends immediately next timer
    }
  }

  // check if button has been pushed down for more than 1000ms
  if (!pinMacro && !pinMacro.triggered() && pinMacro.interval() > 1000)
  {
    pinMacro.trigger();

    // check if the module is already in Smart Config State:
    if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 2; // set to AP Mode
    else
      WiFi_Config = 1; // set to smart config
  }

  // set up wifi according to 'WiFi_Config'
  if (WiFi_Config)
  {
    // check if WiFi already done with setup:
    if (WiFi_Config == 1 && tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore
    else if (WiFi_Config == 2 && tuya_module.mcu_get_wifi_work_state() == AP_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore,
    else
    {
      if (millis() - WiFi_setTimer >= WiFi_setInterval)
      {
        WiFi_setTimer = millis(); // reset timer

        if (WiFi_Config == 1) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(SMART_CONFIG);
        else if (WiFi_Config == 2) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(AP_CONFIG);
      }
    }
  }
}

/*
  moduleErrorHandle():
    handle resetting module when error occurs
*/
inline void moduleErrorHandle()
{
  // keep track of the state for the reset pin. false/LOW = reset, true/HIGH = idle
  static bool resetPinState = true;
  const uint16_t moduleResetInterval = 10000;

  if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
  {
    static uint32_t moduleResetTimer = 0;
    if (resetPinState)
    {
      if (millis() - moduleResetTimer >= moduleResetInterval)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and OUTPUT:
        moduleResetPin.write(LOW);
        moduleResetPin.set(OUTPUT);

        // set pin state:
        resetPinState = false;
      }
    }
    else
    {
      if (millis() - moduleResetTimer >= 300)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and INPUT:
        moduleResetPin.set(INPUT);
        moduleResetPin.write(LOW);

        // set pin state:
        resetPinState = true;
      }
    }
  }
  else
  {
    // no module error, make sure pin is high:
    if (!resetPinState)
    {
      // set reset pin to input:
      moduleResetPin.set(INPUT);
      moduleResetPin.write(LOW);

      resetPinState = true; // reset pin state
    }
  }
}

/*
  dp_proccess():
    all DPID commands pass through this function
*/
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  // return:
  return TY_SUCCESS;
}

/*
  dp_update_all():
    updates all DPID values and resets update timer
*/
void dp_update_all()
{
  // reset interval timer:
  updateAll_Timer = millis();

  uint16_t dp_value = map(analogRead(A0), 0, 1023, 0, 200);

  tuya_module.mcu_dp_update(DPID_VacuumStatus, s[sp], slen[sp]);
  tuya_module.mcu_dp_update(DPID_VacuumBattery, dp_value, 1);
  tuya_module.mcu_dp_update(DPID_VacuumControl, 1, 1);
}

/////////////////////////////////////////////
////////// LED and Servo Sequences //////////
/////////////////////////////////////////////

// smart mode led sequence, fast pulse:
SB_FUNCT(smart_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 130, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(ledBuild.loop(1);)
SB_END

// ap mode led sequence, slow pulse:
SB_FUNCT(ap_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 130, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(ledBuild.loop(1);)
SB_END

// idle led, steady with very soft glow:
SB_FUNCT(idle_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 130, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(macro.quadEase(ledVal, 30, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(ledBuild.loop(0);)
SB_END

// error led, rapid flashing:
SB_FUNCT(error_led, macro.ready(ledVal))
SB_STEP(macro.set(ledVal, 130, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 130, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 130, 50);)
SB_STEP(macro.set(ledVal, 0, 1000);)
SB_STEP(ledBuild.loop(0);)
SB_END

// init led, slow fade in and small delay to allow full initialization of Arduino:
SB_FUNCT(init_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 130, 120);)
SB_STEP(macro.delay(ledVal, 1000);)
SB_END