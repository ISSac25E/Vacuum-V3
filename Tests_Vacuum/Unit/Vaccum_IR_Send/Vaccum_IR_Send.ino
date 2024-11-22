// const bool autoClean[] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0};
volatile const bool start[] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1};
volatile const bool stop[] = {0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0};

#define IR_Pin 11

void setup()
{
  pinMode(3, INPUT_PULLUP);
  pinMode(IR_Pin, INPUT_PULLUP);
  // digitalWrite(IR_Pin, LOW);
}

void loop()
{
  if (!digitalRead(3))
  {
    delay(20);
    if (!digitalRead(3))
    {
      IR_AutoCleanSend();
      delay(20);
      while (!digitalRead(3))
        delay(20);
    }
  }
}
volatile bool sig = false;
void IR_AutoCleanSend()
{
  for (uint8_t x = 0; x < 3; x++)
  {
    digitalWrite(IR_Pin, LOW);
    pinMode(IR_Pin, OUTPUT);
    delayMicroseconds(3080);
    pinMode(IR_Pin, INPUT_PULLUP);
    delayMicroseconds(2980);

    uint16_t sizeArr = (sig ? sizeof(start) : sizeof(stop));

    for (uint16_t x = 0; x < sizeArr; x++)
    {
      digitalWrite(IR_Pin, LOW);
      pinMode(IR_Pin, OUTPUT);
      delayMicroseconds(482);
      pinMode(IR_Pin, INPUT_PULLUP);
      if (sig ? start[x] : stop[x])
        delayMicroseconds(1500);
      else
        delayMicroseconds(518);
    }

    digitalWrite(IR_Pin, LOW);
    pinMode(IR_Pin, OUTPUT);
    delayMicroseconds(482);
    pinMode(IR_Pin, INPUT_PULLUP);

    delay(19);
  }
  delay(100);
  sig = !sig;
}