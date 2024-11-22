#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\SimpleFilter\SimpleFilter_1.0.0.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\PinPort\PinPort_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\VacuumSoftwarePWM\VacuumSoftwarePWM_1.0.0.h"

PinPort dRead(A0);

uint8_t _averageBuff[(255 >> 3) + (bool)(255 & B111)];
BufferAverage_Bool average(_averageBuff, 255);
RollingAverage_16 rollingFilter(255);

void setup()
{
  Serial.begin(115200);

  VacuumSoftwarePWM.setPin(13);
  VacuumSoftwarePWM.enable();

  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
}

void loop()
{
  delayMicroseconds(1000);
  static uint8_t PotVal;

  average.write(dRead.digitalRead());

  Serial.print(0);
  Serial.print(",");
  Serial.print(4700);
  Serial.print(",");
  Serial.print(map(PotVal, 0, 255, 0, 4700));
  Serial.print(",");

  Serial.println(map(average.avg(), 0, 255, 0, 4700));

  {
    static uint32_t timer = millis();
    if (millis() - timer >= 10)
    {
      timer = millis();
      PotVal = map(analogRead(A3), 0, 1023, 0, 255);
      VacuumSoftwarePWM.write(PotVal);
    }
  }
}