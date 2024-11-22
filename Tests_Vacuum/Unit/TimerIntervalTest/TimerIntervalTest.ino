#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinDriver\PinDriver_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\TimerInterval\TimerInterval_1.0.0.h"

PinDriver pin(3);          // set digital pin 3. pullup Mode automatic
InputMacro pinMacro(true); // set start state as high because pin is pullup
void blinkLed();

void setup()
{
  pinMode(13, OUTPUT);
  TimerInterval.setCallback(blinkLed);
}

void loop()
{
  if (pinMacro(pin))
  {
    if (!pinMacro)
    {
      TimerInterval.setTimer(1000000);
    }
  }
}

void blinkLed()
{
  volatile static bool ledState = false;
  if (!ledState)
  {
    digitalWrite(13, HIGH);
    TimerInterval.setTimer(500000);
  }
  else
  {
    digitalWrite(13, LOW);
    TimerInterval.disableTimer();
  }
  ledState = !ledState;
}