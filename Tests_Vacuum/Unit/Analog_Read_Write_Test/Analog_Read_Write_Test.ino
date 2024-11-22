#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\SimpleFilter\SimpleFilter_1.0.0.h"

uint8_t _averageBuff[(400 >> 3) + (bool)(400 & B111)];
BufferAverage_Bool average(_averageBuff, 400);
RollingAverage_16 rollingFilter(50);

void setup()
{
  Serial.begin(115200);

  pinMode(11, OUTPUT);

  pinMode(A2, OUTPUT);
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH);
}

void loop()
{
  {
    static uint32_t timer = millis();
    // if (millis() - timer >= 50)
    {
      timer = millis();
      bool anVal = digitalRead(A0);

      average.write(anVal);
      rollingFilter.avg((anVal ? 1023 : 0));

      Serial.print((anVal ? 1023 : 0));
      Serial.print(",");
      Serial.println(map(average.avg(), 0, 400, 0, 1023));
      // Serial.println(average.avg());
      // Serial.print(",");
      // Serial.println(rollingFilter.avg());
    }
  }

  {
    static uint32_t timer = millis();
    if (millis() - timer >= 5)
    {
      timer = millis();
      analogWrite(11, map(analogRead(A3), 0, 1023, 0, 255));
    }
  }
}
