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
              bitCount < 10 ? Serial.print('0') : Serial.print('1');
              Serial.print(',');
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
    Serial.println();
  }
}