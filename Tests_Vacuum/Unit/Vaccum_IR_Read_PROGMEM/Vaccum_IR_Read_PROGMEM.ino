#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Core\Utility\bitArray.h"

void setup()
{
  Serial.begin(2000000);
  pinMode(9, INPUT_PULLUP);
}

void loop()
{
  if (!digitalRead(9))
  {
    uint32_t timer = micros();
    uint16_t bitCount = 0;
    bool bitPolarity = false;
    uint8_t ByteCount = 0;
    uint8_t bitBufferCount = 0;
    uint8_t byteArray[100];
    byteArray[ByteCount] = 0;

    while (1)
    {
      if (micros() - timer >= 100)
      {
        timer = micros();

        if (digitalRead(9) != bitPolarity)
        {
          bitPolarity = !bitPolarity;

          if (!bitPolarity)
          {
            if (bitCount < 20)
            {
              bitWrite(byteArray[ByteCount], bitBufferCount++, !(bool)(bitCount < 10));
              if (bitBufferCount >= 8)
              {
                bitBufferCount = 0;
                byteArray[++ByteCount] = 0;
              }
            }
          }
          // Serial.print(bitCount);
          // bitPolarity ? Serial.print('/') : Serial.print('\\');

          bitCount = 0;
        }
        else
          bitCount++;

        if (bitCount >= 60)
        {
          // Serial.print(bitCount);
          break;
        }

        // if (bitPolarity)
        //   Serial.print('-');
        // else
        //   Serial.print('_');
      }
    }

    for (uint8_t x = 0; x <= ByteCount; x++)
    {
      Serial.print(byteArray[x]);
      Serial.print(',');
    }
    Serial.println();
    Serial.println((ByteCount * 8) + bitBufferCount);
  }
}