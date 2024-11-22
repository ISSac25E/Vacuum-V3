/*
Vacuum V3
Release Version: 1.0.0 (Initial Release)

ToDO:
  - Resolve "minor" issue with timer interrupt in Vacuum_IR core

////////////////////////////////////////////////////////////////////////////
// !!! Setup is done through "config.h" located in the same directory !!! //
////////////////////////////////////////////////////////////////////////////
*/

#include "Arduino.h"
#include "config.h"

////////////////////////////////////
//////// All required cores ////////
////////////////////////////////////

// TUYA MCU Serial SDK:
#include "Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"

// Vacuum IR for Sending IR codes:
#include "Core\Vacuum_IR\Vacuum_IR_1.0.0.h"

// LED Sequence builder and Macros:
#include "Core\SequenceBuild\SequenceBuild_1.0.3.h"
#include "Core\LedMacro\LedMacro_1.0.0.h"

// Button Input Driver and Macro:
#include "Core\InputMacro\InputMacro_1.0.1.h"
#include "Core\PinDriver\PinDriver_1.0.1.h"

// PinPort for fast digitalWrites and Reads, and software PWM for Status LED:
#include "Core\PinPort\PinPort_1.0.2.h"
#include "Core\VacuumSoftwarePWM\VacuumSoftwarePWM_1.0.0.h"

// Buffered Filter for PWM reading:
#include "Core\SimpleFilter\SimpleFilter_1.0.0.h"

// var par for easier variable handling
#include "Core\VarPar\VarPar_1.0.1.h"

// generate prototypes for required functions:
unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);

////////////////////////////////////
//////////// GPIO Setup ////////////
////////////////////////////////////
PinDriver Vac_Power(Vacuum_PowerPin, LOW); // << Setup Vacuum Power Pin Input Driver. (Set to no PULLUP)

// Setup LED Input Pins:
PinPort Vac_BlueLed(Vacuum_BlueLedPin);
PinPort Vac_YellowLed(Vacuum_YellowLedPin);
PinPort Vac_RedLed(Vacuum_RedLedPin);
// Setup Brush Motor Input Pin:
PinPort Vac_BrushMotor(Vacuum_BrushMotorPin);

// Setup Button Input and Output Driver:
PinPort Vac_ButtonOutput(Vacuum_ButtonPin);       // << OUTPUT
PinDriver Vac_ButtonInput(Vacuum_ButtonPin, LOW); // << INPUT, No PULLUP

#ifdef Vacuum_DebugLedPin
// define debug led pin:
PinPort DebugLed(Vacuum_DebugLedPin);
#endif

// *Vacuum Battery Voltage is Analog Input, Not PinPort needed

//////////////////////////////////////////////////////
//////////// Output Macros Setup and Misc ////////////
//////////////////////////////////////////////////////
// setup sequence builder for led and button output:
SequenceBuild LedBuild;
SequenceBuild ButtonBuild;
#ifdef Vacuum_DebugLedPin
// Set up Debug Led Build
SequenceBuild DebugLedBuild;
#endif

#ifdef Vacuum_DebugLedPin
// setup Three macros for Led, Button Output and debug Led:
LedMacro _Macro[3];
LedMacroManager Macro(_Macro, 3);
#else
// setup two macros for Led and Button Output:
LedMacro _Macro[2];
LedMacroManager Macro(_Macro, 2);
#endif

// pwm value of Led:
uint8_t LedVal = 0;

/*
  ButtonVal:
    0 = button off
    >0 = button on
*/
uint8_t ButtonVal = 0;

#ifdef Vacuum_DebugLedPin
// define debug led val:
uint8_t DebugLedVal = 0;
#endif

// setup global var for Led init(this will come in useful later):
bool LedInit = false;

/////////////////////////////////////////////
///////// Global INPUT States Setup /////////
/////////////////////////////////////////////
// Led Global States:
/*
  0 = Off
  1 = On
  2 = Blinking
  3 = PWM
*/
uint8_t Vac_BlueLedState = 0;
uint8_t Vac_YellowLedState = 0;
uint8_t Vac_RedLedState = 0;
/*
  Vac_BrushMotorState:
    0 = Brush not Spinning  (Not Moving)
    1 = Brush Spinning Slow (Going Home)
    2 = Brush Spinning Fast (Cleaning)
*/
uint8_t Vac_BrushMotorState = 0;

// Battery Voltage State:
uint16_t Vac_BatteryVoltageState = 0;

// Vacuum Power State:
bool Vac_PowerState = false;

//////////////////////////////////////////
//////////// IR Control Setup ////////////
//////////////////////////////////////////
// List of IR Codes:
volatile const uint8_t IR_Code_Stop[] PROGMEM = {22, 242, 0, 0, 255, 109, 0};

volatile const uint8_t IR_Code_Auto_Standard[] PROGMEM = {22, 112, 0, 0, 255, 174, 0};
volatile const uint8_t IR_Code_Auto_BoostIQ[] PROGMEM = {22, 176, 0, 0, 255, 46, 0};
volatile const uint8_t IR_Code_Auto_Max[] PROGMEM = {22, 48, 0, 0, 255, 206, 0};

volatile const uint8_t IR_Code_SpiralClean[] PROGMEM = {22, 49, 0, 0, 255, 207, 0};
volatile const uint8_t IR_Code_EdgeClean[] PROGMEM = {22, 57, 0, 0, 255, 192, 0};

volatile const uint8_t IR_Code_30MinStandard[] PROGMEM = {22, 117, 0, 0, 255, 168, 0};
volatile const uint8_t IR_Code_30MinBoostIQ[] PROGMEM = {22, 181, 0, 0, 255, 40, 0};
volatile const uint8_t IR_Code_30MinMax[] PROGMEM = {22, 61, 0, 0, 255, 196, 0};

volatile const uint8_t IR_Code_GoHome[] PROGMEM = {22, 247, 0, 0, 255, 106, 0};

/////////////////////////////////////////
//////////// TUYA DPID Setup ////////////
/////////////////////////////////////////
/*
  Vac_StatusState:
    0 = ERROR
    1 = Power Off
    2 = Idle  (Blue LED Not On)
    3 = Ready (Blue LED On)
    4 = Cleaning
    5 = Charging
    6 = Done Charging
    7 = Going Home
*/
Par_uint8_t TY_StatusState = 0; // Par var to help with updating to TUYA

// Battery Voltage State:
Par_uint16_t TY_BatteryVoltageState = 0;

// Update Timers for DPID's
uint32_t dpid_updateAll_timer = millis();
uint32_t dpid_update_timer = millis();

//////// Vacuum Status Setup ////////
// all possible Vacuum Status:
const unsigned char _s0_[] = {"ERROR"};
const unsigned char _s1_[] = {"Power Off"};
const unsigned char _s2_[] = {"Idle"};
const unsigned char _s3_[] = {"Ready"};
const unsigned char _s4_[] = {"Cleaning"};
const unsigned char _s5_[] = {"Charging"};
const unsigned char _s6_[] = {"Done Charging"};
const unsigned char _s7_[] = {"Going Home"};

// Setup List and Size of each status Message:
const unsigned char *TY_StatusList[] = {_s0_, _s1_, _s2_, _s3_, _s4_, _s5_, _s6_, _s7_};
const int TY_StatusListLen[] = {sizeof(_s0_), sizeof(_s1_), sizeof(_s2_), sizeof(_s3_), sizeof(_s4_), sizeof(_s5_), sizeof(_s6_), sizeof(_s7_)};

////////////////////////////////////
//////////// TUYA Setup ////////////
////////////////////////////////////
// Tuya Module communication:
TuyaWifi tuya_module((HardwareSerial *)&Serial);

// setup DPID array to pass onto 'TuyaWifi':
unsigned char dpid_array[3][2] = {
    {VacuumStatus_DPID, DP_TYPE_STRING},
    {VacuumControl_DPID, DP_TYPE_ENUM},
    {VacuumBattery_DPID, DP_TYPE_VALUE}};

// setup module reset pin:
PinPort moduleResetPin(TUYA_ModuleResetPin);

void setup()
{
  // initialize hardware Serial to 9600 for communication with TUYA Module:
  Serial.begin(9600);

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

  //////// Pin Setup ////////
  {
    // set module reset pin to Input so user can reset Module too without damaging Arduino:
    moduleResetPin.pinMode(INPUT);
    moduleResetPin.digitalWrite(LOW);

    // Set all LED Input Pins to Input, No PULLUP:
    Vac_BlueLed.pinMode(INPUT);
    Vac_BlueLed.digitalWrite(LOW);
    Vac_YellowLed.pinMode(INPUT);
    Vac_YellowLed.digitalWrite(LOW);
    Vac_RedLed.pinMode(INPUT);
    Vac_RedLed.digitalWrite(LOW);
    // Same for Brush Motor:
    Vac_BrushMotor.pinMode(INPUT);
    Vac_BrushMotor.digitalWrite(LOW);

    // Button Output only gets used occasionally. Set to input No PULLUP in the meantime:
    Vac_ButtonOutput.pinMode(INPUT);
    Vac_ButtonOutput.digitalWrite(LOW);

    // Setup Battery Voltage Pin:
    pinMode(Vacuum_BatteryPin, INPUT);

    // Setup IR pin:
    Vacuum_IR.setPin(Vacuum_IrPin);

    // Setup Software PWM:
    VacuumSoftwarePWM.setPin(Vacuum_BlueLedPin); // << Same pin as Blue LED

#ifdef Vacuum_DebugLedPin
    // setup debug led:
    DebugLed.digitalWrite(LOW);
    DebugLed.pinMode(OUTPUT);
#endif
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
  // run tuya uart service:
  tuya_module.uart_service();

  // run all handles:
  moduleErrorHandle();
  inputHandle();
  gpioHandle();
  statusProccessHandle();
  checkUpdateHandle();

  // check update all timer:
  if (millis() - dpid_updateAll_timer >= DPID_UpdateAllInterval && DPID_UpdateAllInterval)
    dp_update_all();
}

/*
  moduleErrorHandle():
    handle resetting module when error occurs
*/
inline void moduleErrorHandle()
{
  // keep track of the state for the reset pin. false/LOW = reset, true/HIGH = idle
  static bool resetPinState = true;
  static Par_bool moduleErrorVar = false; // << par var for Error state change
  static uint32_t moduleResetTimer = 0;
  const uint16_t moduleResetInterval = 10000;
  const uint16_t moduleResetHeadStart = 4000;

  // run par var for Error Status:
  moduleErrorVar = (bool)(tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR);

  // check for state change:
  if (moduleErrorVar.change())
  {
    if (moduleErrorVar) // Error has occurred
    {
      // Give timer head start. We dont need Module to be reset immediately
      moduleResetTimer = (millis() - (moduleResetInterval - moduleResetHeadStart));
    }
    else // Error has stopped
    {
      // set reset pin to input:
      moduleResetPin.pinMode(INPUT);
      moduleResetPin.digitalWrite(LOW);

      // set pin state:
      resetPinState = true;
    }
  }

  if (moduleErrorVar)
  {
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
}

/*
  inputHandle():
    Take measurements of LED Inputs, BrushMotor Input, and Battery Voltage Input and set Global Vars
*/
inline void inputHandle()
{
  //////// Setup Filter for Brush Motor ////////
  // Setup Filters //
  const uint16_t _brushMotor_FilterDataPoints = 256;

  // Setup Required Buffers for the The Filter(Brush Motor):
  static uint8_t _brushMotor_FilterBuffer_[BufferAverage_Bool_ByteSize(_brushMotor_FilterDataPoints)];
  static BufferAverage_Bool brushMotor_Filter(_brushMotor_FilterBuffer_, _brushMotor_FilterDataPoints); // << setup Filter for brush Motor

  // setup Timer for periodic Filter updates:
  static uint32_t pwmInputTimer = micros();

  ////// Setup macros Led Inputs //////
  static InputMacro blueLed_InputMacro(false);
  static InputMacro yellowLed_InputMacro(false);
  static InputMacro redLed_InputMacro(false);

  static Par_uint8_t brushMotor_StateTest = 0;

  static uint32_t brushMotor_TimerTest = millis();

  // Set Minimume State intervals for led's:
  const uint16_t led_pwmStateInterval = 100;
  const uint16_t led_blinkStateInterval = 1000;
  const uint16_t redLed_blinkStateInterval = 3000; // Set Red LED Blink state Longer. Must wait longer to clear ERROR

  // Setup up minimume change interval for brush motor to change states
  const uint16_t brushMotor_ChangeInterval = 2000;

  //////// Check and Update states on Led Input Pins (Blue, Yellow, Red) ////////
  //// Run all macros and pinDrivers:
  blueLed_InputMacro.run(Vac_BlueLed.digitalRead());
  yellowLed_InputMacro.run(Vac_YellowLed.digitalRead());
  redLed_InputMacro.run(Vac_RedLed.digitalRead());

  //// check state changes ////
  // Blue Led:
  if (blueLed_InputMacro.stateChange())
  {
    if (blueLed_InputMacro.prevInterval() < led_pwmStateInterval)
    {
      // prev interval was less than pwm Interval. Set to PWm
      Vac_BlueLedState = 3; // << pwm
    }
    else if (blueLed_InputMacro.prevInterval() < led_blinkStateInterval)
    {
      // prev interval is greater than pwm_interval but less than blink_interval. Set state to blinking
      Vac_BlueLedState = 2; // << Blinking
    }
    else
    {
      // prev interval is greater than both blink_interval and pwm_interval. Set to On or Off based on state:
      blueLed_InputMacro ? Vac_BlueLedState = 1 : Vac_BlueLedState = 0; // << set to On or off depending on state
    }
  }
  // Yellow Led:
  if (yellowLed_InputMacro.stateChange())
  {
    if (yellowLed_InputMacro.prevInterval() < led_pwmStateInterval)
    {
      // prev interval was less than pwm Interval. Set to PWm
      Vac_YellowLedState = 3; // << pwm
    }
    else if (yellowLed_InputMacro.prevInterval() < led_blinkStateInterval)
    {
      // prev interval is greater than pwm_interval but less than blink_interval. Set state to blinking
      Vac_YellowLedState = 2; // << Blinking
    }
    else
    {
      // prev interval is greater than both blink_interval and pwm_interval. Set to On or Off based on state:
      yellowLed_InputMacro ? Vac_YellowLedState = 1 : Vac_YellowLedState = 0; // << set to On or off depending on state
    }
  }
  // Red Led:
  if (redLed_InputMacro.stateChange())
  {
    if (redLed_InputMacro.prevInterval() < led_pwmStateInterval)
    {
      // prev interval was less than pwm Interval. Set to PWm
      Vac_RedLedState = 3; // << pwm
    }
    else if (redLed_InputMacro.prevInterval() < led_blinkStateInterval)
    {
      // prev interval is greater than pwm_interval but less than blink_interval. Set state to blinking
      Vac_RedLedState = 2; // << Blinking
    }
    else
    {
      // prev interval is greater than both blink_interval and pwm_interval. Set to On or Off based on state:
      redLed_InputMacro ? Vac_RedLedState = 1 : Vac_RedLedState = 0; // << set to On or off depending on state
    }
  }

  // check Led's during 'steady' states:
  // Blue Led:
  if (!blueLed_InputMacro.triggered() && blueLed_InputMacro.interval() >= led_blinkStateInterval)
  {
    // Led was in a steady state longer than max interval(blinking_interval). Set state back to on or off
    blueLed_InputMacro.trigger(); // trigger macro so it does not repeatedly run

    // set state to on or off, depending on macro_state:
    blueLed_InputMacro ? Vac_BlueLedState = 1 : Vac_BlueLedState = 0;
  }
  // Yellow Led:
  if (!yellowLed_InputMacro.triggered() && yellowLed_InputMacro.interval() >= led_blinkStateInterval)
  {
    // Led was in a steady state longer than max interval(blinking_interval). Set state back to on or off
    yellowLed_InputMacro.trigger(); // trigger macro so it does not repeatedly run

    // set state to on or off, depending on macro_state:
    yellowLed_InputMacro ? Vac_YellowLedState = 1 : Vac_YellowLedState = 0;
  }
  // Red Led:
  if (!redLed_InputMacro.triggered() && redLed_InputMacro.interval() >= redLed_blinkStateInterval)
  {
    // Led was in a steady state longer than max Error Blink interval(redLed_blinkInterval). Set state back to on or off
    redLed_InputMacro.trigger(); // trigger macro so it does not repeatedly run

    // set state to on or off, depending on macro_state:
    redLed_InputMacro ? Vac_RedLedState = 1 : Vac_RedLedState = 0;
  }

  //////// Check and Update state on pwm Input Pin (BrushMotor) ////////
  // update filters every (1ms)(1000us):
  if (micros() - pwmInputTimer >= 1000)
  {
    pwmInputTimer = micros(); // << reset timer

    // Update Brush Motor Filter with digital Read:
    brushMotor_Filter.write(Vac_BrushMotor.digitalRead());

    // setup Constraints for BrushMotor and update StateTest:
    if (brushMotor_Filter.avg() <= map(100 /*Constraint in mV*/, 0, 3300 /*mV*/, 0, _brushMotor_FilterDataPoints /*data Points*/))
    {
      // Vacuum is not Moving. BrushMotor is below 100mv
      brushMotor_StateTest = 0;
    }
    // Vacuum is Moving. BrushMotor is above 100mV. Check if its above 1000mV:
    else if (brushMotor_Filter.avg() <= map(1000 /*Constraint in mV*/, 0, 3300 /*mV*/, 0, _brushMotor_FilterDataPoints /*data Points*/))
    {
      // Vacuum is Going home. BrushMotor is spinning, but below 1000mV
      brushMotor_StateTest = 1;
    }
    else
    {
      // Vacuum is Cleaning. BrushMotor is spinning above 1000mV
      brushMotor_StateTest = 2;
    }

    ////////// Test States //////////
    // Check brushMotor for State Changes:
    if (brushMotor_StateTest.change())
    {
      // Brush Motor State changed:
      // reset Timer:
      brushMotor_TimerTest = millis();
    }

    ////////// Test Timers //////////
    // Test Brush Motor Timer:
    if (Vac_BrushMotorState != brushMotor_StateTest && millis() - brushMotor_TimerTest >= brushMotor_ChangeInterval)
    {
      // Change Global Val:
      Vac_BrushMotorState = brushMotor_StateTest;
    }
  }

  //////// Check and update vacuum Power State ////////
  // Run pinDriver and update global state:
  Vac_PowerState = Vac_Power.run();

  //////// Check and update vacuum Battery State ////////
  static uint32_t batteryVoltageTimer = millis();    // << update battery voltage occasionally
  const uint16_t batteryVoltageUpdateInterval = 100; // << set update interval for analog Read

  // Check timer for battery update:
  if (millis() - batteryVoltageTimer >= batteryVoltageUpdateInterval)
  {
    batteryVoltageTimer = millis(); // << reset timer

    // Update global Var:
    // 10K-ohm - 2.55K-ohm voltage divider is used
    // safe voltage range is 0v - 24.608v
    Vac_BatteryVoltageState = map(analogRead(Vacuum_BatteryPin), 0, 1023, 0, 248 /*max Voltage(24.6v) 2478 calibrated value*/);
  }
}

/*
  gpioHandle():
    Handle button input/output, handle Software PWM, handle change between vacuum Power on-off
*/
void gpioHandle()
{
  static InputMacro vac_PowerMacro(LOW); // << macro for vacuum power to keep track of state changes
  static bool buttonInputInit = false;   // << keep track of wether button is ready to be used

  static InputMacro vac_ButtonMacro(HIGH); // << macro for button Input

  // setup Vars for Tuya Wifi Config:
  /*
    WiFi_Config:
      0 = don't set WiFi
      1 = Smart_Mode
      2 = AP_Mode
  */
  static uint8_t WiFi_Config = 0;
  static uint32_t WiFi_setTimer = 0;
  const uint16_t WiFi_setInterval = 20000;

  // init vacuum power for first run
  static bool vac_powerInit = false;

  // run button input regardless of vacuum power state:
  vac_ButtonMacro(Vac_ButtonInput);

  // check for state change on vacuum power:
  if (vac_PowerMacro(Vac_PowerState) || !vac_powerInit)
  {
    // state has changed. Vacuum has just been power off or on:
    vac_powerInit = true; // << set vacuum power as initialized
    if (vac_PowerMacro)
    {
      // Vacuum Power on
      // disable and reset Led:
      VacuumSoftwarePWM.disable();
      LedBuild.stop();
      LedBuild.resetSequence();
      LedVal = 0;
      LedInit = false;

      // disable button input init on interrupt change:
      buttonInputInit = false;
    }
    else
    {
      // Vacuum power off
      // enable and reset Led:
      LedBuild.stop();
      LedBuild.resetSequence();
      LedVal = 0;
      LedInit = false;
      VacuumSoftwarePWM.enable();

      // reset button output:
      ButtonBuild.stop();
      ButtonBuild.resetSequence();
      ButtonVal = 0;

      // stop IR Send:
      Vacuum_IR.stop();

      // disable button input init on interrupt change:
      buttonInputInit = false;
    }
  }

  // run button macros:
  if (!vac_PowerMacro)
  {
    // check if button has been initialized yet after vacuum powered off:
    if (buttonInputInit)
    {
      // Run WiFi Config Button Macro:

      // Check for button state change:
      if (vac_ButtonMacro.stateChange())
      {
        // button has just been pressed or released:
        if (vac_ButtonMacro)
        {
          // Button Released
          WiFi_Config = 0; // << stop WiFi Config
        }
        else
        {
          // Button Pressed
          WiFi_setTimer = (millis() - WiFi_setInterval); // reset wifi send Timer so it triggers wifi set immediately on button press
          // check if led needs to be initialized:
          if (!LedInit)
            LedBuild.setPrioritySequence(init_led, 0, true);
        }
      }

      // check if button has been pushed down for more than 1000ms
      if (!vac_ButtonMacro && !vac_ButtonMacro.triggered() && vac_ButtonMacro.interval() > 1000)
      {
        vac_ButtonMacro.trigger();

        // check if the module is already in Smart Config State:
        if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
          WiFi_Config = 2; // set to AP Mode
        else
          WiFi_Config = 1; // set to smart config
      }
    }
    else
    {
      // wait for button to be released before initializing button:
      if (vac_ButtonMacro)
      {
        buttonInputInit = true;
      }
    }
  }

  // Run Tuya Wifi Config:
  if (WiFi_Config) // set up wifi according to 'WiFi_Config'
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

  // Run Led Output Sequences:
  if (LedInit) // only run if Led has been initialized
  {
    if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
      LedBuild.setSequence(ap_mode_led, 0, true);
    else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      LedBuild.setSequence(smart_mode_led, 0, true);
    else if (!vac_ButtonMacro) // << If Button is pressed but no Wifi Config mode is running, keep the led on/idle
      LedBuild.setSequence(on_led, 0, true);
    else // << no Wifi config mode is running and button is released, begin end_led sequence:
      LedBuild.setSequence(end_led, 0, true);
  }

#ifdef Vacuum_DebugLedPin
  // Run Debug Led:
  if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
    DebugLedBuild.setSequence(error_debugLed, 0, true);
  else
    DebugLedBuild.setSequence(idle_debugLed, 0, true);
#endif
}

/*
  statusProccessHandle():
    determine final status for TUYA to send out. Calculate values *if needed
    update "Vacuum Status" and "Vacuum Battery Voltage"
*/
inline void statusProccessHandle()
{
  ////////// Update Battery Voltage //////////
  TY_BatteryVoltageState = Vac_BatteryVoltageState; // << no need to recalculate anything, its already calculated

  ////////// Update Status //////////
  if (!Vac_PowerState) // << check Power, top priority state
  {
    // Vacuum is not powered on
    TY_StatusState = 1; // << set status to "Power Off"
  }
  else
  {
    // Vacuum is powered on
    if (Vac_RedLedState == 2 /*Blinking*/) // << Red Led second Priority. Blink Red means error
    {
      // Vacuum has an error
      TY_StatusState = 0; // << set status to "ERROR"
    }
    else
    {
      // Vacuum does not have an error
      if (Vac_YellowLedState) // << Yellow Led. Vacuum Charging
      {
        // Vacuum is Charging
        TY_StatusState = 5; // << set status to "Charging"
      }
      else
      {
        // Vacuum is not Charging
        // Check if Vacuum is Done Charging:
        if (Vac_RedLedState == 1 /*On*/ || Vac_RedLedState == 3 /*PWM*/)
        {
          // Vacuum is done Charging:
          TY_StatusState = 6; // << set status to "Done Charging"
        }
        else
        {
          // Check Brush Motor State:
          switch (Vac_BrushMotorState)
          {
          case 1:
            // Brush Spinning Slowly, Vacuum Going Home
            TY_StatusState = 7; // << set status to "Going Home"
            break;
          case 2:
            // Brush Spinning Quickly, Vacuum Cleaning
            TY_StatusState = 4; // << set status to "Cleaning"
            break;
          default:
            // Brush Not Spinning, Vacuum is Stationary
            if (Vac_BlueLedState)
            {
              // Blue Led is on, Vacuum is Ready
              TY_StatusState = 3; // << set status to "Ready"
            }
            else
            {
              // Blue Led is off, Vacuum is Idle
              TY_StatusState = 2; // << set status to "Idle"
            }
            break;
          }
        }
      }
    }
  }
}

/*
  checkUpdateHandle():
    check Tuya Values and update if needed
*/
inline void checkUpdateHandle()
{
  // only update at an interval:
  if (millis() - dpid_update_timer >= DPID_UpdateInterval && DPID_UpdateInterval)
  {
    if (TY_StatusState.change())
    {
      tuya_module.mcu_dp_update(VacuumStatus_DPID, TY_StatusList[TY_StatusState], TY_StatusListLen[TY_StatusState]);
      dpid_update_timer = millis(); // << reset update timer
    }
    if (TY_BatteryVoltageState.change())
    {
      tuya_module.mcu_dp_update(VacuumBattery_DPID, TY_BatteryVoltageState, 1);
      dpid_update_timer = millis(); // << reset update timer
    }
  }
}

unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
#ifdef Vacuum_DebugLedPin
  DebugLedBuild.setPrioritySequence(dpProccess_debugLed, 0, true); // indicate dp proccess
#endif
  // the only thing we are looking for is vacuum control dpid.
  /* vacuum control dpid ENUM:
    0 = Toggle(Button)
    1 = Start(IR Auto)
    2 = Stop(IR)
    3 = Auto Standard(IR)
    4 = Auto Boost IQ(IR)
    5 = Auto Max(IR)
    6 = Spiral Clean(IR)
    7 = Edge Clean(IR)
    8 = Home(IR)
  */

  if (dpid == VacuumControl_DPID) // << check if correct dpid
  {
    const uint8_t Vacuum_NewCommand = tuya_module.mcu_get_dp_download_data(dpid, value, length); // << download the data
    tuya_module.mcu_dp_update(VacuumControl_DPID, 0, 1);                                         // << update default value

    switch (Vacuum_NewCommand)
    {
    case 0:
      // Toggle(Button)
      if (Vac_PowerState) // make sure vacuum is powered on
      {
        ButtonBuild.setSequence(run_button, 0, true);
      }
      break;
    case 1:
      // Start(IR Auto)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_Auto_Standard, 48);
      }
      break;
    case 2:
      // Stop(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_Stop, 48);
      }
      break;
    case 3:
      // Auto Standard(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_Auto_Standard, 48);
      }
      break;
    case 4:
      // Auto Boost IQ(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_Auto_BoostIQ, 48);
      }
      break;
    case 5:
      // Auto Max(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_Auto_Max, 48);
      }
      break;
    case 6:
      // Spiral Clean(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_SpiralClean, 48);
      }
      break;
    case 7:
      // Edge Clean(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_EdgeClean, 48);
      }
      break;
    case 8:
      // Home(IR)
      if (Vac_PowerState) // << make sure vacuum is powered on
      {
        Vacuum_IR.stop(); // << make sure no other IR codes are running
        Vacuum_IR.writeByte_PROGMEM((uint8_t *)IR_Code_GoHome, 48);
      }
      break;
    }
  }
  // return:
  return TY_SUCCESS;
}

void dp_update_all()
{
  // trigger change interrupt on values and update Tuya Module:
  TY_StatusState.change();
  tuya_module.mcu_dp_update(VacuumStatus_DPID, TY_StatusList[TY_StatusState], TY_StatusListLen[TY_StatusState]);
  TY_BatteryVoltageState.change();
  tuya_module.mcu_dp_update(VacuumBattery_DPID, TY_BatteryVoltageState, 1);

  // Vacuum Control dpid should only be "send-only". Send default value just in case:
  tuya_module.mcu_dp_update(VacuumControl_DPID, 0, 1);

  // Reset all timers:
  dpid_updateAll_timer = millis();
  dpid_update_timer = millis();
}

/*
  ISR(TIMER1_COMPA_vect):
    Timer 1 Interrupt Vector for LED and Button Output
    Runs 100 timers-per-second (100Hz)

    Writes to LED and Button Outputs also
*/
ISR(TIMER1_COMPA_vect)
{
  // run all necessary handles first:
  LedBuild.run();
  ButtonBuild.run();
  Macro.run();
#ifdef Vacuum_DebugLedPin
  DebugLedBuild.run();
#endif

  VacuumSoftwarePWM.write(LedVal);
  if (ButtonVal)
  {
    Vac_ButtonOutput.digitalWrite(LOW);
    Vac_ButtonOutput.pinMode(OUTPUT);
  }
  else
  {
    Vac_ButtonOutput.pinMode(INPUT);
    Vac_ButtonOutput.digitalWrite(LOW);
  }

#ifdef Vacuum_DebugLedPin
  // write debug led val:
  if (DebugLedVal)
  {
    DebugLed.digitalWrite(HIGH);
  }
  else
  {
    DebugLed.digitalWrite(LOW);
  }
#endif
}

///////////////////////////////////////////////
/////////// LED and Button Sequences //////////
///////////////////////////////////////////////

// init led, slow fade in and small delay. Setup LED Init Also:
SB_FUNCT(init_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, map(3300 /*mV*/, 0, 5000, 0, 255), 60);) // set fade to around 3300mV
SB_STEP(Macro.delay(LedVal, 500);)
SB_STEP(LedBuild.stop(); // set led Init and stop led sequences
        LedInit = true;)
SB_END

// simply keep turning the led on:
SB_FUNCT(on_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, map(3300 /*mV*/, 0, 5000, 0, 255), 10);)
SB_END

// give the led a small pause, then fade out and disable led again:
SB_FUNCT(end_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, map(3300 /*mV*/, 0, 5000, 0, 255), 10);)
SB_STEP(Macro.delay(LedVal, 500);)
SB_STEP(Macro.quadEase(LedVal, 0, 20);)
SB_STEP(LedBuild.stop();
        LedInit = false;)
SB_END

// smart mode led sequence, fast pulse:
SB_FUNCT(smart_mode_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.quadEase(LedVal, map(3300 /*mV*/, 0, 5000, 0, 255), 10);)
SB_STEP(Macro.delay(LedVal, 100);)
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.delay(LedVal, 100);)
SB_STEP(LedBuild.loop(1);)
SB_END

// ap mode led sequence, slow pulse:
SB_FUNCT(ap_mode_led, Macro.ready(LedVal))
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.quadEase(LedVal, map(3300 /*mV*/, 0, 5000, 0, 255), 10);)
SB_STEP(Macro.delay(LedVal, 1400);)
SB_STEP(Macro.quadEase(LedVal, 0, 10);)
SB_STEP(Macro.delay(LedVal, 1400);)
SB_STEP(LedBuild.loop(1);)
SB_END

// Send a short press to the button:
SB_FUNCT(run_button, Macro.ready(ButtonVal))
SB_STEP(Macro.set(ButtonVal, 1, 300);)
SB_STEP(Macro.set(ButtonVal, 0, 300);) // delay again to prevent repeated, close button calls
SB_STEP(ButtonBuild.stop();
        ButtonBuild.resetSequence();)
SB_END

#ifdef Vacuum_DebugLedPin
//////// Debug Led Sequences ////////
// simply keeps debug led on:
SB_FUNCT(idle_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 1, 0);)
SB_END

// debug led error. Rapid Triple Flash
SB_FUNCT(error_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 50);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 50);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_STEP(Macro.set(DebugLedVal, 0, 1000);)
SB_STEP(DebugLedBuild.loop(0);)
SB_END

// Quick flash to indicate dp Proccess:
SB_FUNCT(dpProccess_debugLed, Macro.ready(DebugLedVal))
SB_STEP(Macro.set(DebugLedVal, 0, 100);)
SB_STEP(Macro.set(DebugLedVal, 1, 50);)
SB_END

#endif
