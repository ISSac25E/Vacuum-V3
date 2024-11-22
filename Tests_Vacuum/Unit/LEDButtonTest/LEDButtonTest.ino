#define TUYA_PID "rtmm5fp1azxmqmzg" // << This can be found in your TUYA account
#define TUYA_MCU_Version "1.0.0"    // << This field does not really matter. It can be a useful tool to keep track of revisions

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinDriver\PinDriver_1.0.1.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\VacuumSoftwarePWM\VacuumSoftwarePWM_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\SequenceBuild\SequenceBuild_1.0.3.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\LedMacro\LedMacro_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinPort\PinPort_1.0.2.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"

unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);

TuyaWifi tuya_module((HardwareSerial *)&Serial);

// setup DPID array to pass onto 'TuyaWifi':
unsigned char dpid_array[3][2] = {
    {101, DP_TYPE_STRING},
    {102, DP_TYPE_ENUM},
    {103, DP_TYPE_VALUE}};

PinDriver pinInput(3);
InputMacro pinMacro(HIGH);

// setup module reset pin:
PinPort moduleResetPin(7);

// setup sequence builder for LED:
SequenceBuild ledBuild;

// setup macro for led AND servo (total 2):
LedMacro _macro[2];
LedMacroManager macro(_macro, 2);

// PWM led value:
uint8_t ledVal = 0;

bool ledInit = false;

void setup()
{
  Serial.begin(9600);

  moduleResetPin.pinMode(INPUT);
  moduleResetPin.digitalWrite(LOW);

  VacuumSoftwarePWM.setPin(13);
  VacuumSoftwarePWM.enable();

  // Setup Timer Interrupt Interval for LED (100Hz):
  {
    cli(); // stop all interrupts

    // reset Timer 1 Registers:
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0; // initialize timer  counter value to 0

    // set compare match A register for 100 Hz increments:
    OCR1A = 19999; // = 16000000 / (8(pre-scaler) * 100) - 1 (must be <65536)

    TCCR1B |= (1 << WGM12); // turn on CTC(Clear-Timer(counter)-On-Compare) mode

    // Set CS12, CS11 and CS10 bits for 8 prescaler:
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

    // enable timer compare interrupt on control mask register:
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow all interrupts
  }

  //////// set up TUYA Module: ////////
  {
    unsigned char pid[] = {TUYA_PID};
    unsigned char mcu_ver[] = {TUYA_MCU_Version};

    // init TUYA PID and MCU Version:
    tuya_module.init(pid, mcu_ver);

    // Set all TUYA DPID's:
    tuya_module.set_dp_cmd_total(dpid_array, 3);

    // set callbacks:
    tuya_module.dp_process_func_register(dp_process);       // function to proccess incoming commands
    tuya_module.dp_update_all_func_register(dp_update_all); // function to update all DPID states at once
  }
}

void loop()
{
  tuya_module.uart_service();

  moduleErrorHandle();
  buttonHandle();
  outputHandle();
}

/*
  outputHandle():
    handles LED sequences and servo sequences
*/
inline void outputHandle()
{

  if (ledInit)
  {
    if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
      ledBuild.setSequence(ap_mode_led, 0, true);
    else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      ledBuild.setSequence(smart_mode_led, 0, true);
    else if (!pinMacro)
      ledBuild.setSequence(on_led, 0, true);
    else
      ledBuild.setSequence(end_led, 0, true);
  }
  // determine led sequence:
  // if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
  //   ledBuild.setSequence(ap_mode_led, 0, true);
  // else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
  //   ledBuild.setSequence(smart_mode_led, 0, true);
  // else if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR || tuya_module.mcu_get_wifi_work_state() == WIFI_STATE_UNKNOWN)
  //   ledBuild.setSequence(error_led, 0, true);
  // else
  //   ledBuild.setSequence(idle_led, 0, true);
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
    }
    else // button pressed
    {
      if (!ledInit)
        ledBuild.setPrioritySequence(init_led, 0, true);
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
        moduleResetPin.digitalWrite(LOW);
        moduleResetPin.pinMode(OUTPUT);

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
        moduleResetPin.pinMode(INPUT);
        moduleResetPin.digitalWrite(LOW);

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
      moduleResetPin.pinMode(INPUT);
      moduleResetPin.digitalWrite(LOW);

      resetPinState = true; // reset pin state
    }
  }
}

/*
  ISR(TIMER1_COMPA_vect):
    Timer 1 Interrupt Vector for LED and Servo Output
    Runs 100 timers-per-second

    Writes to LED and Servo Outputs also
*/
ISR(TIMER1_COMPA_vect)
{
  // run all necessary handles first:
  ledBuild.run();
  macro.run();

  VacuumSoftwarePWM.write(ledVal);
}

unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  // return:
  return TY_SUCCESS;
}

void dp_update_all()
{
}

// smart mode led sequence, fast pulse:
SB_FUNCT(smart_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(ledBuild.loop(1);)
SB_END

// ap mode led sequence, slow pulse:
SB_FUNCT(ap_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(ledBuild.loop(1);)
SB_END

// idle led, steady with very soft glow:
SB_FUNCT(idle_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(macro.quadEase(ledVal, 100, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(ledBuild.loop(0);)
SB_END

// error led, rapid flashing:
SB_FUNCT(error_led, macro.ready(ledVal))
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 1000);)
SB_STEP(ledBuild.loop(0);)
SB_END

// init led, slow fade in and small delay to allow full initialization of Arduino:
SB_FUNCT(init_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 60);)
SB_STEP(macro.delay(ledVal, 500);)
SB_STEP(ledBuild.stop();
        ledInit = true;)
SB_END

SB_FUNCT(on_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 60);)
SB_END

SB_FUNCT(end_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 500);)
SB_STEP(macro.quadEase(ledVal, 0, 30);)
SB_STEP(ledBuild.stop();
        ledInit = false;)
SB_END