#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinDriver\PinDriver_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\VarPar\VarPar_1.0.1.h"

PinDriver inPin(3);
InputMacro pinMacro(HIGH);
/*
  0 = off
  1 = on
  2 = blink
  3 = pwm
*/
Par_uint8_t pinStatus = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println(115200);
}

void loop()
{
  {
    if (pinMacro(inPin))
    {
      if (pinMacro.prevInterval() < 100)
      {
        pinStatus = 3;
      }
      else if (pinMacro.prevInterval() < 500)
      {
        pinStatus = 2;
      }
      else
      {
        pinMacro ? pinStatus = 0 : pinStatus = 1;
      }
    }

    if (!pinMacro.triggered() && pinMacro.interval() >= 500)
    {
        pinMacro ? pinStatus = 0 : pinStatus = 1;
        pinMacro.trigger();
    }
  }

  if (pinStatus.change())
  {
    switch (pinStatus)
    {
    case 0:
      Serial.println("Off");
      break;
    case 1:
      Serial.println("On");
      break;
    case 2:
      Serial.println("Blinking");
      break;
    default:
      Serial.println("PWM");
      break;
    }
  }
}